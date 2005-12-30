#include <stdio.h>
#include <stdlib.h>
#include "random.h"

inline double dist2(double *xy1, double *xy2){
  double dx,dy;
  dx = xy1[0] - xy2[0];
  dy = xy1[1] - xy2[0];
  return (dx*dx + dy*dy);
}
  
double compute_mse(double (*xy1)[2], double (*xy2)[2], int *ind, int n){
  register double d2 = 0;
  register int i;

  double dx,dy;

  for(i = 0; i < n; ++i){
    dx = xy1[i][0] - xy2[i][0];
    dy = xy1[i][1] - xy2[i][1];

    d2 += (dx*dx + dy*dy);
  }
  return d2;
}

int main(){
  const int n = 361;

  double (*xy1)[2] = (double (*)[2]) malloc(n*sizeof(double[2]));
  double (*xy2)[2] = (double (*)[2]) malloc(n*sizeof(double[2]));
  int *ind = (int*) malloc(n*sizeof(int));
  int i;

  for(i = 0; i < n; ++i){
    ind[i] = i;
    xy1[i][0] = i   ; xy1[i][1] = n - i;
    xy2[i][0] = i +1; xy2[i][1] = n - i - 1;
  }

  double mse = compute_mse(xy1, xy2, ind, n);

  printf("mse = %g\n", mse);

  printf("%d %d\n", xy1, xy2);
  free(xy1);
  free(xy2);
  free(ind);
}
