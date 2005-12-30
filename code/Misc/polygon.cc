#ifndef DEPEND
#include <stdlib.h>
#endif

#include "polygon.h"

//Gift wrap algorithm
extern int Convex_Hull_2D(int Nb, const VECT2 *Pts, int *Ind);


void findConvexHull(Polygon *p_out, const VECT2 *points, int np){
  int ind[np+1];
  int nconvex;
  VECT2 * P;
  int i;

  //  printf("findConvexHull %d\n",np);

  for(i = 0; i < np; ++i) ind[i] = i;

  nconvex = Convex_Hull_2D(np,points,ind);

  //  printf("Convex has %d points.\n",nconvex);

  P = new VECT2[nconvex];
  for(i = 0; i < nconvex; ++i){
    P[i][0] = points[ind[i]][0]; 
    P[i][1] = points[ind[i]][1]; 
  }


  p_out->destroy();
  p_out->set(P,nconvex);

  delete[] P;
}

#if 0
extern int ConvexIntersect(VECT2* out, 
                           const VECT2* const P, int n, 
                           const VECT2* const Q, int m );

//Finds intersect of two convex polygons p1 and p2
void findIntersect(Polygon *p_out, 
                   const Polygon *p1, const Polygon *p2){

  int nmax = p1->numPoints() + p2->numPoints();
  VECT2* out = new VECT2[nmax];

  int n = ConvexIntersect(out, (*p1), p1->numPoints()
                             , (*p2), p2->numPoints());

  if(n > 0){
    p_out->set(out,n);
  }else{
    p_out->destroy();
  }

  delete[] out;
}
#endif

////////////////////////////////////////////////////////////////////
// SeparatePoints function
////////////////////////////////////////////////////////////////////


int separateMap(const SimpleMap* m1, int *core1,
		const Polygon* poly){
  int i;
  VECT2 xx[m1->numElements()];
  int ind[m1->numElements()];
  int n = 0;

  for(i = 0; i < m1->numElements(); ++i){
    Gaussian2d g(m1->get(i)->getCenter());
    xx[n][0] = g.x;
    xx[n][1] = g.y;
    ind[n] = i;
    n += 1;
  }

  if(n <= 0) return 0;

  Line l;
  int n_good = separatePoints(&l,xx,n,poly);

  if(n_good == n) return n_good; //All points are good

  if(n_good == 0){ //All points were inside the polygon
    memset(core1, 0, m1->numElements()*sizeof(int));
    return 0;
  }

  for(i = 0; i < n; ++i){
    //If to the right mark as 0
    if(l.distance(xx[i][0],xx[i][1]) < -1e-06){
      core1[ind[i]] = 0;
    }
  }

  return n_good;
}


inline void swap(double* &a, double* &b){
  double *tmp = b;  b = a;  a = tmp;     
}

inline void computeDist(double *out, const Line &line,
                        const VECT2 *xx, int np){
  int i;
  for(i = 0; i < np; ++i){
    out[i] = line.distance(xx[i][0],xx[i][1]);
  }
}


struct Entry{
  double v;
  int ind;
  int pos;

  Entry():v(0),ind(0),pos(1){;}
  Entry(double V, int I, int P):v(V),ind(I),pos(P){;}
};

static int qsort_callback(const void *v1, const void *v2){
  double d1,d2;
  d1 = ((struct Entry*) v1)->v;
  d2 = ((struct Entry*) v2)->v;

  if(d1 < d2)      return -1;
  else if(d1 > d2) return  1;
  else             return  0;
}

inline int findMax(struct Entry* data, int n){
  int i;
  int iMax = 0;
  double vMax = data[0].v;

  for(i = 1; i < n; ++i) 
    if(data[i].v > vMax){
      iMax = i;
      vMax = data[i].v;
    }

  return iMax;
}

inline int findMin(struct Entry* data, int n){
  int i;
  int iMin = 0;
  double vMin = data[0].v;

  for(i = 1; i < n; ++i) 
    if(data[i].v < vMin){
      iMin = i;
      vMin = data[i].v;
    }

  return iMin;
}

//returns number of points successfully classified
// *ind = point that contributes to the line 0,0 -> x(ind),y(ind)
int findBestLine(int *ind, int *dir, 
                 const double *neg_x, const double*y, int np
                 , int MAX_SO_FAR){
  int n = 0;


  struct Entry data[np];

  int base_good = 0;
  int i;
  int nPos = 0;
  int nNeg = 0;

  //printf("%% Find Best Line (%d -- max)\n", MAX_SO_FAR);
  for(i = 0; i < np; ++i){
    double x = -neg_x[i];

    //printf("%% xx = [%e %e];\n", x,y[i]);

    if(y[i] > 0.0){
      if(x > 0.0){ //1st Quadrant
	data[n].v = y[i]/x;
	data[n].ind = i;
	data[n].pos = 1;
	n += 1;
	nPos += 1;

	//printf("%% 1st quadrant\n");
      }else{       //2nd Quadrant
	//printf("%% 2nd quadrant\n");
	base_good += 1;
      }
    }else if(x < 0.0){ //3rd Quadrant
	data[n].v   = y[i]/x;
	data[n].ind =  i;
	data[n].pos = -1;
	n += 1;
	nNeg += 1;

	//printf("%% 3rd quadrant\n");
    } //4th quadrant points are ignored 
  }

  //No point looking if we can't possibly reach the goal
  if(n + base_good < MAX_SO_FAR) return -1;

  //If no points in 3rd and 1st quadrant then any line will do
  if(n <= 0){ 
    *ind = -1;
    return base_good;
  }

  //If nPos == 0 Then find max of data.v
  if(nPos == 0){
    i = findMax(data,n);
    *ind = data[i].ind;
    *dir = -1;
    return base_good + nNeg;
  }

  //If nNeg == 0 Then  find min of data.v
  if(nNeg == 0){
    i = findMin(data,n);
    *ind = data[i].ind;
    *dir = 1;
    return base_good + nPos;
  }

  //Sort the data in increasing order of the y/x
  qsort(data,n, sizeof(Entry), qsort_callback);

  int iBest = 0;
  int prev  = data[0].pos;
  int nGood = nPos;

  if(prev < 0){
    nGood += 1;
  }
  int nMax = nGood;

  for(i = 1; i < n; ++i){
    if(prev == data[i].pos){
      nGood -= prev;

      if(nGood > nMax){
	iBest = i;
	nMax  = nGood;
      }
    }
    prev = data[i].pos;
  }


  *ind = data[iBest].ind;
  *dir = data[iBest].pos;

  //printf("%% I = %d, Dir = %d, N = %d\n", *ind, *dir, base_good + nMax);
  return base_good + nMax;
}

#define CLOCK_WISE 0 //Counter clock wise

int separatePoints(Line* l_out, const VECT2* xx, int np, const Polygon* poly){

  double *x;
  double *y;
  double store1[np];
  double store2[np];
  int i;
  Line axis1;
  Line axis2;
  int npoly = poly->numPoints();

#if CLOCK_WISE
  axis2.set(poly->x(npoly-1),poly->y(npoly-1),poly->x(0),poly->y(0));
#else
  axis2.set(poly->x(1),poly->y(1),poly->x(0),poly->y(0));
#endif

  x = store1;
  y = store2;
  computeDist(y, axis2, xx, np);

  int nMax = 0;
  int iMax = -1; //Index of the point
  int vMax = -1; //Index of the vertex
  int dirMax = 0;    //Direction -1 point->vertex, 1 vertex->point

  for(i = 0; i < npoly; ++i){
    //    int i0 = (i - 1)%npoly;
    int i1 = (i + 1)%npoly;

#if CLOCK_WISE
    int k = i;
#else
    int k = (npoly-i)%npoly;
    i1    = (npoly-i1)%npoly;
#endif


    axis1 = axis2;
    swap(x,y);

    axis2.set(poly->x(k),poly->y(k),poly->x(i1),poly->y(i1));
    computeDist(y, axis2, xx, np);

    int n, ind, dir;

    //printf("%% Vertex %d -- (%e, %e)\n", k, poly->x(i),poly->y(i));

    n = findBestLine(&ind,&dir, x, y, np, nMax);

    if(n > nMax){
      nMax = n;
      iMax = ind;
      vMax = k;
      dirMax = dir;

      if(nMax >= np) break; //All point are in, can't find any better
    }

  }

  //printf("convex1 = [");
  for(i = 0; i < npoly; ++i){
    //printf("%e, %e; ", poly->x(i),poly->y(i));
  }
  //printf("];\n");

  //printf("points = [");
  for(i = 0; i < np; ++i){
    //printf("%e, %e; ", xx[i][0],xx[i][1]);
  }
  //printf("];\n");


  if(nMax <= 0) return 0;

  if(iMax < 0){
    //No points in 1st or 3rd quadrant only 2 and 4
    //Line from vertex to vertex will do
    int v2 = (vMax+1)%npoly;
    //printf("%% Setting line along the edge: %d=>%d\n", vMax, v2);

    l_out->set(poly->x(vMax),poly->y(vMax), poly->x(v2), poly->y(v2));
  }else{
    if(dirMax > 0){
      l_out->set(poly->x(vMax), poly->y(vMax), xx[iMax][0],xx[iMax][1]);
    }else{
      l_out->set(xx[iMax][0],xx[iMax][1], poly->x(vMax), poly->y(vMax));
    }
  }       

  return nMax;
}
