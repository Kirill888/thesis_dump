#ifndef __POLYGON_H__
#define __POLYGON_H__

#ifndef DEPEND
#include <string.h>
#endif

#include "geometry.h"
#include "map.h"

typedef double VECT2[2];

extern double sin(double);
extern double cos(double);

extern char  SegSegInt( const VECT2 a, const VECT2 b, 
                        const VECT2 c, const VECT2 d, 
                        VECT2 p, VECT2 q );


class Polygon{
 private:
  int np;
  VECT2 *p;

  double xc,yc;
  double xmin, xmax;
  double ymin, ymax;

  void computeCenterPoint(){
    int i;
    xc = yc = 0.0;
    xmin = ymin = +Inf;
    xmax = ymax = -Inf;

    for(i = 0; i < np; ++i){
      xc += p[i][0];
      yc += p[i][1];

      if(xmin > p[i][0]) xmin = p[i][0];
      if(xmax < p[i][0]) xmax = p[i][0];

      if(ymin > p[i][1]) ymin = p[i][1];
      if(ymax < p[i][1]) ymax = p[i][1];

    }
    xc /= np;
    yc /= np;
  }

 public:
  Polygon():np(0),p(NULL),xc(0),yc(0){;}
  
  Polygon(const VECT2 *points, int n){
    set(points,n);
  }

  Polygon(const Polygon& other){
    set(&other);
  }

  Polygon & operator=(const Polygon& rhs){
    if(&rhs != this){
      destroy();
      set(&rhs);
    }
    return *this;
  }

  ~Polygon(){ destroy();}

  void set(const Polygon* other){
    if(other->p){
      np = other->np;
      p = new VECT2[np];
      memcpy(p,other->p,np*sizeof(VECT2));
      xc = other->xc; yc = other->yc;
      xmin = other->xmin; ymin = other->ymin;
      xmax = other->xmax; ymax = other->ymax;
    }else{
      np = 0;
      p = NULL;
      xc = yc = xmin = ymin = xmax = ymax = 0.0;
    }
  }

  void set(const VECT2* points, int n){
    if(points == NULL){
      p = NULL;
      np = 0;
      xc = yc = 0.0;
    }else{
      p = new VECT2[n];
      memcpy(p,points,n*sizeof(VECT2));
      np = n;
      computeCenterPoint();
    }
  }

  inline void expand(double dr);

  inline bool isInside(double x, double y)const;

  inline double area()const;

  void translateMe(const RobotPose& r){
    translateMe(r.x,r.y,cos(r.rot),sin(r.rot));
  }

  inline void translateMe(double x, double y, double ca, double sa);

  void destroy(){
    delete[] p;
    p = NULL;
    np = 0;
  }

  int numPoints()const{return np;}
  operator const VECT2*()const{return p;}
  double x(int i)const{return p[i][0];}
  double y(int i)const{return p[i][1];}

  void getCenter(double *x, double* y)const{
    *x = xc; *y = yc;
  }
  void getMin(double *x, double* y)const{
    *x = xmin; *y = ymin;
  }
  void getMax(double *x, double* y)const{
    *x = xmax; *y = ymax;
  }

  inline bool checkIntersect(const VECT2 a, const VECT2 b)const;

};

void findConvexHull(Polygon *p_out, const VECT2 *points, int np);

#if 0
//Finds intersect of two convex polygons p1 and p2
void findIntersect(Polygon *p_out, 
                   const Polygon *p1, const Polygon *p2);
#endif

//Finds the line that separates points from the convex hull
//  maximises number of points to the left of the line
//  convex hull is to the right of the line
int separatePoints(Line* l_out, const VECT2* xx, int np, const Polygon* poly);

int separateMap(const SimpleMap* m1, int *core1,
		const Polygon* poly);


//////////////////////////////////////////////////
//Inlines
/////////////////////////////////////////////////

inline bool Polygon::isInside(double x, double y)const{
  int	i, i1;		/* point index; i1 = i-1 mod n */
  double     q;	        /* q intersection of e with ray */
  int	 Rcross = 0; /* number of right edge/ray crossings */
  int    Lcross = 0; /* number of left edge/ray crossings */


  const int X = 0;
  const int Y = 1;

  VECT2 P[np];

  /* Shift so that (x,y) is the origin. */
  for( i = 0; i < np; i++ ) {
      P[i][0] = p[i][0] - x;
      P[i][1] = p[i][1] - y;
  }

  /* For each edge e=(i-1,i), see if crosses ray. */
  for( i = 0; i < np; i++ ) {

    if(P[i][X] == 0.0 && P[i][Y] == 0.0){
      //It is a vertex
      return true;
    }

    i1 = ( i + np - 1 ) % np;
 
    if( ( P[i][Y] > 0 ) != ( P[i1][Y] > 0 ) ) {
      
      /* e straddles ray, so compute intersection with ray. */
      q = (P[i][X] * (double)P[i1][Y] - P[i1][X] * (double)P[i][Y])
	/ (double)(P[i1][Y] - P[i][Y]);
      /* printf("straddles: x = %g\t", x); */
      
      /* crosses ray if strictly positive intersection. */
      if (q > 0) Rcross++;
    }
    /* printf("Right cross=%d\t", Rcross); */
    
    /* if e straddles the x-axis when reversed... */
    /* if( ( ( P[i] [Y] < 0 ) && ( P[i1][Y] >= 0 ) ) ||
       ( ( P[i1][Y] < 0 ) && ( P[i] [Y] >= 0 ) ) )  { */
    
    if ( ( P[i][Y] < 0 ) != ( P[i1][Y] < 0 ) ) { 
      /* e straddles ray, so compute intersection with ray. */
      q = (P[i][X] * P[i1][Y] - P[i1][X] * P[i][Y])
          / (double)(P[i1][Y] - P[i][Y]);
      /* printf("straddles: x = %g\t", x); */

      /* crosses ray if strictly positive intersection. */
      if (q < 0) Lcross++;
    }

  }

  /* q on the edge if left and right cross are not the same parity. */
  if( ( Rcross % 2 ) != (Lcross % 2 ) )
    return true;
  
  /* q inside iff an odd number of crossings. */
  if( (Rcross % 2) == 1 )
    return true;
  else	return false;
}

inline bool Polygon::checkIntersect(const VECT2 a, const VECT2 b)const{
  int i;
  VECT2 tmp1,tmp2;

  if(isInside(a[0],a[1]) || isInside(b[0],b[1])) return true;

  for(i = 0; i < np; ++i){
    int i2 = (i+1)%np;
    char res = SegSegInt(a,b,p[i],p[i2],tmp1,tmp2);
    if(res != '0') return true;
  }

  return false;
}

inline void Polygon::translateMe(double x, double y, double ca, double sa){
  int i;
 
  for(i = 0; i < np; ++i){
    double x0,y0;
    x0 = p[i][0];
    y0 = p[i][1];

    p[i][0] = x0*ca - y0*sa + x;
    p[i][1] = x0*sa + y0*ca + y;
  }

#if 0
  double tmp = xc*ca - yc*sa + xc;
  yc         = xc*sa + yc*ca + yc;
  xc = tmp;
#else
  computeCenterPoint();
#endif


}

inline  void Polygon::expand(double dr){
  int i;
  for(i = 0; i < np; ++i){
    double x = p[i][0] - xc;
    double y = p[i][1] - yc;

    double R = sqrt(x*x + y*y) + dr;
    double a = atan2(y,x);

    p[i][0] = R*cos(a) + xc;
    p[i][1] = R*sin(a) + yc;
  }
}

inline double Polygon::area()const{
  if(np < 3) return 0.0;
  int i,j;
  double area = 0.0;

  for(i = 0; i < np; ++i){
    j = (i+1) % np;
    area += p[i][0] * p[j][1];
    area -= p[i][1] * p[j][0];
  }

  area *= 0.5;

  if(area < 0) return - area;

  return area;
}

#endif
