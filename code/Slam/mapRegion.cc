#include "mapRegion.h"

void RegionGridMap::computeTransform(){
  double scale_x = 1.0/grid->cellX();
  double scale_y = 1.0/grid->cellY();
  double X0,Y0;
  grid->getMin(&X0,&Y0);

  if(translated){
    A[0][0] =  scale_x*ca; A[0][1] =  scale_x*sa;
    A[1][0] = -scale_y*sa; A[1][1] =  scale_y*ca;

    b[0] = -scale_x*( x0*ca + y0*sa + X0);
    b[1] = -scale_y*(-x0*sa + y0*ca + Y0);
  }else{
    A[0][0] = scale_x; A[0][1] =   0;
    A[1][0] =    0   ; A[1][1] = scale_y;

    b[0] = -X0*scale_x;
    b[1] = -Y0*scale_y;
  }

//   printf("Transform: A = [%g %g; %g %g]; b = [%g %g]\n"
// 	 ,A[0][0], A[0][1], A[1][0], A[1][1], b[0], b[1]
//         );
}

void RegionGridMap::translateMe(double x, double y, double ca, double sa){
  if(translated){
    double x0_;
    x0_ = x0*ca - y0*sa + x; 
    y0  = x0*sa + y0*ca + y;
    x0 = x0_;
    double ca_;

    ca_      = this->ca*ca - this->sa*sa;
    this->sa = this->sa*ca + this->ca*sa;
    this->ca = ca_;
  }else{
    x0 = x;           y0 = y; 
    this->ca = ca;    this->sa = sa;
  }

  translated = true;
  computeTransform();
}

double RegionGridMap::probIsInside(double x, double y)const{
#if 0
  int  ix = (int)(A[0][0]*x + A[0][1]*y + b[0]);
  int  iy = (int)(A[1][0]*x + A[1][1]*y + b[1]);


  if(translated){
    double dx = x - x0;
    double dy = y - y0;

    x =  dx*ca + dy*sa;
    y = -dx*sa + dy*ca;
  }

  int ix2 = grid->x2ind(x);
  int iy2 = grid->y2ind(y);

  if(ix2 != ix || iy2 != iy2){
    printf("Error %d,%d != %d,%d\t",ix,iy, ix2, iy2);
    printf("Transform: A = [%g %g; %g %g]; b = [%g %g]\n"
	 ,A[0][0], A[0][1], A[1][0], A[1][1], b[0], b[1]
        );
  }

  if(grid->isInside(x,y)){
    return grid->get(x,y);
  }else{
    return 0.0;
  }

#else
  int ix,iy;

  if(translated){
    ix = int(A[0][0]*x + A[0][1]*y + b[0]);
    iy = int(A[1][0]*x + A[1][1]*y + b[1]);
  }else{
    ix = int(A[0][0]*x + b[0]);
    iy = int(A[1][1]*y + b[1]);
  }

  if(grid->isInside(ix,iy)){
    return grid->get(ix,iy);
  }else{
    return 0.0;
  }
#endif

}

bool RegionGridMap::getPixel(int* ix, int* iy, double x, double y)const{
#if 0
  if(translated){
    double dx = x - x0;
    double dy = y - y0;

    x =  dx*ca + dy*sa;
    y = -dx*sa + dy*ca;
  }

  if(grid->isInside(x,y)){
    *ix = grid->x2ind(x);
    *iy = grid->y2ind(y);
  }else{
    *ix = *iy = -1;
  }
#else

  if(translated){
    *ix = int(A[0][0]*x + A[0][1]*y + b[0]);
    *iy = int(A[1][0]*x + A[1][1]*y + b[1]);
  }else{
    *ix = int(A[0][0]*x + b[0]);
    *iy = int(A[1][1]*y + b[1]);
  }

  return grid->isInside(*ix,*iy);

//   if(!grid->isInside(*ix,*iy)){
//     *ix = -1;
//     *iy = -1;
//   }


#endif

}

inline bool IS_OCCUPIED(GM_type v){
  return v > 0.5;
}

int RegionGridMap::subtract(const RegionGridMap* other){

  int n_removed = 0;
  GM_type* dat = *grid;
  int i;

  for(i = 0; i < grid->size(); ++i){
    if(IS_OCCUPIED(dat[i])){
      double x,y;
      grid->ind2point(&x,&y,i);

      if(translated){
	double tmpx = x*ca - y*sa + x0;
	          y = x*sa + y*ca + y0;
	          x = tmpx;
      }

      if(other->probIsInside(x,y) > 0.5){
	dat[i] = 0;
	n_removed += 1;
      }
    }
  }

  return n_removed;
}

int RegionGridMap::subtract(const RegionGridMap* other, 
                            const RobotPose &odo){

  int n_removed = 0;
  GM_type* dat = *grid;
  int i;

  printf("Subtract Region: %.3f %.3f %.3f (deg)\n",
	 odo.x, odo.y, odo.rot*RADTODEG);

  double CA,SA;
  CA = cos(odo.rot); SA = sin(odo.rot);

  for(i = 0; i < grid->size(); ++i){
    if(IS_OCCUPIED(dat[i])){
      double x,y;
      grid->ind2point(&x,&y,i);

      if(translated){
	double tmpx = x*ca - y*sa + x0;
	          y = x*sa + y*ca + y0;
	          x = tmpx;
      }

      //Translate into coordinate of the other
      x = x - odo.x;
      y = y - odo.y;
      double tmpx =  x*CA + y*SA;
                y = -x*SA + y*CA;
                x = tmpx;

      if(other->probIsInside(x,y) > 0.5){
	dat[i] = 0;
	n_removed += 1;
      }
    }
  }

  return n_removed;
}


bool RegionGridMap::matlabDump(FILE*f, const char* name)const{
  bool res;

  if(grid == NULL){
    return fprintf(f,"%s = [];\n", name) > 0;
  }

  double xmin, ymin, xmax, ymax;

  grid->getMin( &xmin , &ymin);
  grid->getMax( &xmax , &ymax);

  res = fprintf(f,"%s.dim = [%g, %g, %g, %g];\n"
                  "%s.data = [ ...\n"
		,name, xmin, ymin, xmax,ymax
                ,name) > 0;

  grid->dump(f,"%g ");

  res &= fprintf(f,"]; %%end %s.data\n",name) > 0;

  return res;
}
