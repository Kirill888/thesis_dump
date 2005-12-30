#include "gridmap.h"
#include "image.h"

void GridMap::shrink(){

  if(data == NULL) return;

  int bounds[4];

  image_getNonZeroBounds(bounds, data, nr, nc);

  if(bounds[0] >= nr){ //Set to empty
    delete[] data; data = NULL;
    nr = nc = sz = 0;
    x0 = y0 = ymax = xmax = 0.0;
    dx = dy = 1.0;
    return;
  }

  if(bounds[0] == 0    && bounds[1] == 0 && 
     bounds[2] == nr-1 && bounds[3] == nc-1){
    //Same size no change required
    return;
  }

  double x0_, y0_, x1_, y1_;

  x0_ = x(bounds[1]); x1_ = x(bounds[3]);
  y0_ = y(bounds[0]); y1_ = y(bounds[2]);

  printf("Shrink: %d,%d => %d,%d -> %d,%d\n"
	 ,nr,nc, bounds[0], bounds[1]
	 ,bounds[2], bounds[3]);

  printf("Shrink: %.2f,%.2f - %.2f,%.2f => %.2f,%.2f - %.2f,%.2f\n"
	 ,x0 , y0 , xmax, ymax
	 ,x0_, y0_, x1_ , y1_ 
          );

  x0 = x0_; y0 = y0_; xmax = x1_; ymax = y1_;

  GM_type *data0 = data;

  data = image_subImage(data0, &nr,&nc, bounds);
  sz = nc*nr;

  delete[] data0;
}

bool GridMap::writePgm(char* fname)const{
  FILE * f= fopen(fname,"w");
  if(f == NULL) return false;

  fprintf(f,"P2\n%d %d\n255\n",nc,nr);
  int i,j;

  for (j = nr - 1; j >= 0; j--) {
    for (i = 0; i < nc; i++) {
      fprintf(f, "%i ", (int) (255 - get(i,j)*255));
    }
    fprintf(f, "\n");
  }

  fclose(f);
  return true;
};

