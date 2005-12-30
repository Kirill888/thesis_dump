#ifndef DEPEND
#include <stdio.h>
#include <stdlib.h>
#endif
#include "geometry.h"
#include "polygon.h"
#include "matrix.h"
#include "util.h"
#include "gridmap.h"

bool readpoint(double p[2]){
  return (scanf("%lf %lf",&p[0],&p[1]) == 2);
}

bool read2points(double p1[2],double p2[2]){
  return (scanf("%lf %lf %lf %lf",&p1[0],&p1[1],&p2[0],&p2[1]) == 4);
}

bool readPolygon(VECT2 *poly, int *np){
  int npMax = *np;

  *np = 0;
  if(scanf("%d",np) != 1) return false;
  int i;

  if(*np > npMax){
    printf("Too many points: %d > %d\n", *np,npMax);
    return false;
  }

  for(i = 0; i < *np; ++i){
    if(!readpoint(poly[i])) return false;
  }
  return true;
}

void testLine(){
  Line line;
  double p1[2];
  double p2[2];

  printf("Enter line (2 points):");fflush(stdout);
  if(read2points(p1,p2)){
    line.set(p1[0],p1[1],p2[0],p2[1]);
    printf("line: %f %f %f\n",line.A(),line.B(),line.C());

    printf("Enter point:");fflush(stdout);
    while(readpoint(p1)){
      printf("Distance: %f\n",line.distance(p1[0],p1[1]));
      printf("Enter point:");fflush(stdout);
    }
  }
}

void testPolygon(){
  VECT2 points[] = {{0,1},{1,0},{2,0},{3,2},{2,3}};
  Polygon poly;
  double p[2];

  poly.set(points,5);
  printf("Poly.area() = %.3f\n", poly.area());

  printf("Enter point:");fflush(stdout);
  while(readpoint(p)){
    bool inside = poly.isInside(p[0],p[1]);

    printf("Inside: %s\n",inside ? "yes" : "no");
    printf("Enter point:");fflush(stdout);
  }

}

void testHull(){
  const int MAXP = 1000;

  VECT2 points[MAXP];
  int np = 0;
  int i;

  Polygon poly;

  //  printf("Enter point:");fflush(stdout);
  while(np < MAXP && readpoint(points[np])){
    np += 1;
    //    printf("Enter point:");fflush(stdout);
  }

  printf("%% Points\n");
  for(i = 0; i < np; ++i){
    printf("%.3f %.3f\n",points[i][0],points[i][1]);
  }

  printf("%%Computing Convex Hull: %d\n",np);

  findConvexHull(&poly, points, np);

  printf("%% Polygon\npoly = [ ...\n");

  for(i = 0; i < poly.numPoints(); ++i){
    printf("%.3f %.3f\n",poly.x(i),poly.y(i));
  }

  printf("];\n");
}


/*
 * gauss - Solves the system A*x = B for the vector x. x is returned
 * in the variable B. If you simply want to invert A, choose B equal
 * to the identity matrix.  A is an NxN matrix.  B is an NxM matrix
 * (or vector).  
 */

int gauss(double *AA, unsigned int N, double *BB, unsigned int M) {
#define A(r,c) AA[index(r,c,N)]
#define B(r,c) BB[index(r,c,M)]

  double big, dum, pivinv;
  
  unsigned int *ipiv;
  unsigned int *indxr;
  unsigned int *indxc;

  unsigned int i, j, k, l, ll;
  int lll;
  unsigned int irow = 0, icol = 0;

  ipiv = (unsigned int *) malloc(sizeof(unsigned int) * N);
  indxr = (unsigned int *) malloc(sizeof(unsigned int) * N);
  indxc = (unsigned int *) malloc(sizeof(unsigned int) * N);
  if ((ipiv == NULL) || (indxr == NULL) || (indxc == NULL)) {
    //DROS_DEBUGC(1, "Memory allocation failure in gauss_inv %d", N);
    return -1;
  }

  for (j = 0; j < N; j++)
    ipiv[j] = 0;

  for (i = 0; i < N; i++) {
    big = 0.0;
    for (j = 0; j < N; j++)
      if (ipiv[j] != 1) {
	for (k = 0; k < N; k++)	{
	  if (ipiv[k] == 0) {
	    if (fabs(A(j,k)) >= big) {
	      big = fabs(A(j,k));
	      irow = j;
	      icol = k;
	    } else if(ipiv[k] > 1) {
	      free(ipiv);
	      free(indxr);
	      free(indxc);
	      return -1;       /* Singular matrix */
	    }
	  }
	}
      }

    ipiv[icol] = ipiv[icol] + 1;
    if (irow != icol) {
      for (l = 0; l < N; l++) {
	dum = A(irow,l);
	A(irow,l) = A(icol,l);
	A(icol,l) = dum;
      }
      for (l = 0; l < M; l++) {
	dum = B(irow,l);
	B(irow,l) = B(icol,l);
	B(icol,l) = dum;
      }
    }
    indxr[i] = irow;
    indxc[i] = icol;
    if (A(icol,icol) == 0.0) {
      free(ipiv);
      free(indxr);
      free(indxc);
      return -1;       /* Singular matrix */
    }
    pivinv = 1.0 / A(icol,icol);
    A(icol,icol) = 1.0;

    for (l = 0; l < N; l++)
      A(icol,l) = A(icol,l) * pivinv;
    for (l = 0; l < M; l++)
      B(icol,l) = B(icol,l) * pivinv;

    for (ll = 0; ll < N; ll++) {
      if (ll != icol) {
	dum = A(ll,icol);
	A(ll,icol) = 0.0;
	for (l = 0; l < N; l++)
	  A(ll,l) = A(ll,l) - A(icol,l) * dum;
	for (l = 0; l < M; l++)
	  B(ll,l) = B(ll,l) - B(icol,l) * dum;
      }
    }
  }

  for (lll = N - 1; lll >= 0; lll--)
    if (indxr[lll] != indxc[lll])
      for (k = 0; k < N; k++) {
	dum = A(k,indxr[lll]);
	A(k,indxr[lll]) = A(k,indxc[lll]);
	A(k,indxc[lll]) = dum;
      }
  
  free(ipiv);
  free(indxr);
  free(indxc);
  
  return 0;

#undef A
#undef B
#undef C
}

void testSeparatePoints();

void print_grid(const GridMap& g, const char* format = NULL){
  int ix,iy,nc;

  if(format == NULL){
    format = "%e ";
  }

  nc  = printf(format,0.0);
  nc += printf("|");

  for(ix = 0; ix < g.numX(); ++ix){
    nc += printf(format, g.x(ix,0));
  }


  printf("\n");
  for(int i = 0; i < nc; ++i) printf("-");
  printf("\n");

  for(iy = g.numY()-1; iy >= 0; --iy){
    printf(format,g.y(0,iy));
    printf("|");
    for(ix = 0; ix < g.numX(); ++ix){
      printf(format,g.get(ix,iy));
    }
    printf("\n");
  }
}


void testGridMap(){
  const double x0 = -5;
  const double y0 = -1;
  const int nx = 10;
  const int ny = 15;
  const double xl = 10;
  const double yl = 15;

  GridMap map(x0,xl, y0,yl, nx,ny);
  map.setAll(-1);

  printf("Inited map:\n");
  print_grid(map,"%5g ");

  map(-5.0,-1.0) = 0;
  map(0.0, 0.0) += 5;
  map(3.0, 3.0) = 10;
  map(3.0, 4.0) = 11;
  map(4.99, 13.99) = 100;

  printf("Changed map:\n");
  print_grid(map,"%5g ");
  printf("Matlab dump:\n");
  map.dump(stdout,"%5g ");
}

int main(){

  testGridMap();

#if 0
  const int n = 2;
  const int m = 3;
  const int r = 4;

  double A[n][m]  = {{1, 2, 3},{4, 5, 6}};
  double B[m][r]  = {{0.1, 1, 2, 3},{0.2, 4, 5, 6},{0.3, 7, 8, 9}};
  double AB[n][r];


  matrix_multiply(AB,A,B,n,m,r);


  UniformRandomNumber dice(0,10 + 1);
  int k;
  double d;
  int i =0;

  do{
    k = dice.nextIntRandom();
    d = dice.nextRandom();
    printf("%d %e\n",k,d);
    i += 1;
  }while(i < 10000);
#endif
 
  //  testPolygon();

  return 1;
}





int separatePoints(Line* l_out, const VECT2* xx, int np
		   , const Polygon* poly);

void testSeparatePoints(){
  VECT2 poly[100];
  VECT2 points[100];
  int npoly = 100;
  int np = 0;
  int i;


  if(!readPolygon(poly,&npoly)) return;

  while(readpoint(points[np])){
    np += 1;
    if(np >= 100) break;
  }

  if(np <= 0) return;

  printf("poly = [");
  for(i = 0; i < npoly; ++i){
    printf("%e,%e\n",poly[i][0],poly[i][1]);
  }
  printf("];\n");

  printf("xx = [");
  for(i = 0; i < np; ++i){
    printf("%e,%e\n",points[i][0],points[i][1]);
  }
  printf("];\n");

  Polygon p(poly,npoly);
  Line line;

  int nGood = separatePoints(&line, points, np, &p);
  printf("nGood = %d\n", nGood);

  printf("l = [%e %e %e]\n", line.A(),line.B(),line.C());

  printf("ind_good = [");
  for(i = 0; i < np; ++i){
    if(line.distance(points[i][0],points[i][1]) >= -1e-06){
      nGood -= 1;
      printf(" %d",i+1);
    }
  }
  printf("];\n");

  if(nGood != 0){
    printf("%% nGood was wrong!!\n");
  }


}
