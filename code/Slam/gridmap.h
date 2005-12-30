#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__

#include "util.h"

typedef unsigned char GM_type;

class GridMap{
 private:

  GM_type * data;
  int nc,nr, sz;

  double x0,y0;
  double dx,dy; //step size
  double xmax, ymax;

  double toDouble(GM_type v)const{
    return double(v)/255.0;
  }

  GM_type fromDouble(double v)const{
    if(v >= 1.0) return 255;
    if(v <= 0.0) return 0;

    return (GM_type) (v*255);
  }

 public:

  GridMap():data(NULL),nc(0),nr(0),sz(0),x0(0),y0(0)
            ,dx(1),dy(1),xmax(0),ymax(0){;}

  GridMap(double X0, double x_length, 
          double Y0, double y_length,
          int nx, int ny){
    set(X0,x_length, Y0, y_length, nx, ny);
  }

  GridMap(const GridMap &other){
    set(&other);
  }

  const GridMap& operator=(const GridMap& other){
    if(&other != this){
      destroy();
      set(&other);
    }
    return *this;
  }

  GridMap* clone()const{
    return new GridMap(*this);
  }


  ~GridMap(){ if(data != NULL) delete[] data;}

  void set(const GridMap* other){
    nc = other->nc;     nr = other->nr;      sz = other->sz;
    x0 = other->x0;     y0 = other->y0;
    dx = other->dx;     dy = other->dy;
    xmax = other->xmax; ymax = other-> ymax;

    if(other->data != NULL){
      data = new GM_type[sz];
      memcpy(data, other->data, sz*sizeof(GM_type));
    }else data = NULL;
  }

  void set(double X0, double x_length, 
           double Y0, double y_length,
           int nx, int ny){
    nc = nx;
    nr = ny;
    sz = nc*nr;

    x0 = X0;
    y0 = Y0;

    dx = x_length/nx;
    dy = y_length/ny;

    xmax = x0 + x_length;
    ymax = y0 + y_length;

    data = new GM_type[sz];
  }

  void set(double X0, double x_length, double step_x, 
           double Y0, double y_length, double step_y
           ){
    dx = step_x;                 dy = step_y;
    x0 = X0;                     y0 = Y0;
    xmax = x0 + x_length;        ymax = y0 + y_length;
    nc = (int)(x_length/dx) + 1; nr = (int)(y_length/dy) + 1;

    sz = nc*nr;
    data = new GM_type[sz];
  }

  int ind(int ix, int iy)const{
#if GRID_MAP_SAFETY_CHECK
    if(ix < 0 || iy < 0 || ix >= nc || iy >= nr){
      printf("GridMap::SafetyCheck failed: %d,%d => %d,%d\n",ix,iy,nc,nr);
      exit(2);
    }
#endif
    return iy*nc + ix;
  }

  double operator()(int ix, int iy)const{
    return toDouble(data[ind(ix,iy)]);
  }

  double operator()(double x, double y)const{
    return toDouble(data[ind(x2ind(x),y2ind(y))]);
  }

  void set(int ix, int iy, double v){
    data[ind(ix,iy)] = fromDouble(v);
  }

  void setAll(double v){
    int i;
    GM_type v0 = fromDouble(v);

    for(i =0 ; i < sz; ++i){
      data[i] = v0;
    }
  }

  double get(int ix, int iy)const{
    return toDouble(data[ind(ix,iy)]);
  }

  double get(double x, double y)const{
    return toDouble(data[ind(x2ind(x),y2ind(y))]);
  }

  GM_type get_raw(int ind)const{
    return data[ind];
  }

  bool isInside(int ix, int iy)const{
    if(ix < 0   || iy < 0 ||
       ix >= nc || iy >= nr) return false;

    return true;
  }

  bool isInside(double x, double y)const{

#if 1
    int ix = x2ind(x); int iy = y2ind(y);
    if(ix < 0   || iy < 0 ||
       ix >= nc || iy >= nr) return false;

    return true;

#else
    if(x < x0)    return false;
    if(y < y0)    return false;
    if(x >= xmax) return false;
    if(y >= ymax) return false;

    return true;
#endif
  }

  int x2ind(double x)const{ return (int)((x - x0)/dx); }
  int y2ind(double y)const{ return (int)((y - y0)/dy); }

  double x(int ix, int iy)const{
    return x0 + ix*dx + 0.5*dx;
  }
  double x(int ix)const{
    return x0 + ix*dx + 0.5*dx;
  }

  double y(int ix, int iy)const{
    return y0 + iy*dy + 0.5*dy;
  }
  double y(int iy)const{
    return y0 + iy*dy + 0.5*dy;
  }

  void ind2point(int* ix, int* iy, int i)const{
    *ix = i%nc;
    *iy = i/nc;
  }

  void ind2point(double* x, double* y, int i)const{
    int ix = i%nc;
    int iy = i/nc;
    *x = this->x(ix); *y = this->y(iy);
  }

  int numX()const{ return nc;}
  int numY()const{ return nr;}
  int size()const{ return sz;}

  operator GM_type* (){ return data;}

  GM_type* getData()const{ return data;}

  void getMin(double *x, double* y)const{
    *x = x0; *y = y0;
  }

  void getMax(double *x, double* y)const{
    *x = xmax; *y = ymax;
  }

  double cellX()const{ return dx;}
  double cellY()const{ return dy;}

  //Shrink in size, without loosing any non-zero elements
  void shrink();

  void destroy(){
    DESTROY(data);
    nc = nr = sz = 0; 
  }

  void dump(FILE *f, const char* format)const{
    int i;
    for(i = 0; i < sz; ++i){
      if(i>0 && i%nc == 0) fprintf(f,"\n");

      fprintf(f,format,toDouble(data[i]));
    }
    fprintf(f,"\n");
  }

  bool writePgm(char *fname)const;
};


#endif
