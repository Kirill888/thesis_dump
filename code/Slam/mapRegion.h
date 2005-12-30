#ifndef __MAP_REGION_H__
#define __MAP_REGION_H__

#ifndef DEPEND
#include <stdlib.h>
#include <math.h>
#endif

#include "map.h"
#include "gridmap.h"
#include "geometry.h"


//====================================================================
//
//====================================================================
class RegionGridMap: public MapAreaInterface{
 private:
  GridMap* grid;
  double x0,y0;
  double ca,sa;
  bool translated;
  double A[2][2];
  double b[2];

  void computeTransform();

 public:
  RegionGridMap():grid(NULL),x0(0),y0(0),ca(1),sa(0),translated(false){;}
  RegionGridMap(GridMap *g):grid(g),
               x0(0),y0(0),ca(1),sa(0),translated(false){
    computeTransform();
  }

  RegionGridMap(const RegionGridMap &other){
    set(&other);
  }

  const RegionGridMap & operator=(const RegionGridMap &other){
    if(this != &other){
      DESTROY(grid);
      set(&other);
    }

    return *this;
  }

  void set(const RegionGridMap * other){
    grid = CLONE(other->grid);
    x0 = other->x0; y0 = other->y0;
    ca = other->ca; sa = other->sa;
    translated = other->translated;
    A[0][0] = other->A[0][0];
    A[0][1] = other->A[0][1];
    A[1][0] = other->A[1][0];
    A[1][1] = other->A[1][1];
    b[0] = other->b[0];
    b[1] = other->b[1];
  }

  MapAreaInterface* clone()const{
    //    printf("RegionGridMap::clone()\n");
    return new RegionGridMap(*this);
  }

  const GridMap* getGrid()const{ return grid;}

  ~RegionGridMap(){ DESTROY(grid);}


  void set(GridMap* g){
    DESTROY(grid); 
    grid = g;
    computeTransform();
  }

  double probIsInside(const MapEntryInterface *e)const{
    double x,y;
    e->getCenter(&x,&y);
    return probIsInside(x,y);
  }

  double probIsInside(const RobotPose *r)const{
    return probIsInside(r->x, r->y);
  }

  //Convert x,y coordinate to the ix,iy coordinate
  // in the grid map.
  // If robot pose is outside the grid map, row = col = -1;
  bool getPixel(int* ix, int* iy, double x, double y)const;

  void shrink(){ 
    grid->shrink(); 
    computeTransform();
  }

  /* // This can only be done with specific obs models
  double probIsInside(const ObservationInterface *o,
                      const RobotPose *r)const{
  }
  */

  double probIsInside(double x, double y)const;
  
  void setPose(double x, double y, double a){
    x0 = x;  y0 = y; ca = cos(a); sa = sin(a); translated = true;
    if(grid != NULL) computeTransform();
  }
  void translateMe(double x, double y, double ca, double sa);

  bool matlabDump(FILE*f, const char* name)const;

  int subtract(const RegionGridMap* other);
  int subtract(const RegionGridMap* other, const RobotPose& pose);
};

//====================================================================
//
//====================================================================
class RegionAll: public MapAreaInterface{
 public:
  RegionAll(){;}

  double probIsInside(double,double)const{return 1;}
  double probIsInside(const MapEntryInterface *e)const{
    return 1;
  }
  double probIsInside(const ObservationInterface *o,
                      const RobotPose *r)const{
    return 1;
  }

  void getCenter(double *x, double *y)const{*x =   0;  *y =   0;}
  void getMin(double *x, double* y)const{   *x = -Inf; *y = -Inf;}
  void getMax(double *x, double* y)const{   *x = +Inf; *y = +Inf;}

  MapAreaInterface* clone()const{ return new RegionAll();}
};


//====================================================================
//
//====================================================================
class RegionUnion: public MapAreaInterface{
 private:
  int sz;
  int n;
  const MapAreaInterface** regions;

  int byteSize(int s)const{ return s*sizeof(const MapAreaInterface*);}

 public:
  RegionUnion():sz(0),n(0),regions(NULL){;}

  RegionUnion(int max_sz):sz(max_sz),n(0){
    regions = (const MapAreaInterface**)malloc(byteSize(sz));
  }

  RegionUnion(const RegionUnion & other){
    set(&other);
  }

  const RegionUnion& operator=(const RegionUnion& other){
    if(this == &other) return *this;

    destroy();
    set(&other);
  }

  void set(const RegionUnion *other){
    n = other->n;
    sz = other->sz;

    if(other->regions != NULL){
      regions = (const MapAreaInterface**)malloc(byteSize(sz));
      memcpy(regions, other->regions, byteSize(sz));
    }
  }

  ~RegionUnion(){destroy();}

  void destroy(){
    if(regions != NULL) free(regions);
    regions = NULL;
    n = 0;   sz = 0;
  }

  void add(const MapAreaInterface *r){
    if(n >= sz){
      sz += 10;
      regions = (const MapAreaInterface**)realloc(regions, byteSize(sz));
    }
    regions[n] = r;
    n += 1;
  }

  //MapAreaInterface:

  //Prob is inside any region -- OR --> (1-AND(1-p[i]))
  // if no regions then 0
  double probIsInside(const MapEntryInterface *m)const{
    double pp = 1.0; double p; int i;

    for(i = 0; i < n; ++i){
      p   = regions[i]->probIsInside(m); pp *= (1 - p);
      if(pp < 0.05) break;}

    return 1 - pp;
  }

  double probIsInside(double x, double y)const{
    double pp = 1.0;
    double p;
    int i;

    for(i = 0; i < n; ++i){
      p   = regions[i]->probIsInside(x,y);
      pp *= (1 - p);

      if(pp < 0.05) break;
    }

    return 1 - pp;
  }

  double probIsInside(const ObservationInterface *o,
                      const RobotPose *r)const{
    double pp = 1.0;
    double p;
    int i;

    for(i = 0; i < n; ++i){
      p   = regions[i]->probIsInside(o,r);
      pp *= (1 - p);

      if(pp < 0.05) break;
    }

    return 1 - pp;
  }

};

//====================================================================
//
//====================================================================
class RegionIntersection: public MapAreaInterface{
 private:
  int sz;
  int n;
  const MapAreaInterface** regions;

  int byteSize(int s)const{ return s*sizeof(const MapAreaInterface*);}

 public:
  RegionIntersection():sz(0),n(0),regions(NULL){;}

  RegionIntersection(int max_sz):sz(max_sz),n(0){
    regions = (const MapAreaInterface**)malloc(byteSize(sz));
  }

  RegionIntersection(const RegionIntersection & other){
    set(&other);
  }

  const RegionIntersection& operator=(const RegionIntersection& other){
    if(this == &other) return *this;

    destroy();
    set(&other);
  }

  void set(const RegionIntersection *other){
    n = other->n;
    sz = other->sz;

    if(other->regions != NULL){
      regions = (const MapAreaInterface**)malloc(byteSize(sz));
      memcpy(regions, other->regions, byteSize(sz));
    }
  }

  ~RegionIntersection(){destroy();}

  void destroy(){
    if(regions != NULL) free(regions);
    regions = NULL;
    n = 0;   sz = 0;
  }

  void add(const MapAreaInterface *r){
    if(n >= sz){
      sz += 10;
      regions = (const MapAreaInterface**)realloc(regions, byteSize(sz));
    }
    regions[n] = r;
    n += 1;
  }

  //MapAreaInterface:

  //Prob is inside all regions AND(p[i])
  // if no regions then 0
  double probIsInside(const MapEntryInterface *m)const{
    if(n <= 0) return 0.0;

    double pp = 1.0;
    int i;

    for(i = 0; i < n; ++i){
      pp *= regions[i]->probIsInside(m);
      if(pp < 0.05) break;
    }

    return pp;
  }

  double probIsInside(const ObservationInterface *o,
                      const RobotPose *r)const{
    if(n <= 0) return 0.0;

    double pp = 1.0;
    int i;

    for(i = 0; i < n; ++i){
      pp *= regions[i]->probIsInside(o,r);
      if(pp < 0.05) break;
    }

    return pp;
  }

  double probIsInside(double x, double y)const{
    if(n <= 0) return 0.0;

    double pp = 1.0;
    int i;

    for(i = 0; i < n; ++i){
      pp *= regions[i]->probIsInside(x,y);
      if(pp < 0.05) break;
    }

    return pp;
  }

};

//====================================================================
//
//====================================================================
class RegionInverse: public MapAreaInterface{
 private: 
  const MapAreaInterface* reg;

 public:
  RegionInverse():reg(NULL){;}
  RegionInverse(const MapAreaInterface* rg):reg(rg){;}

  void set(const MapAreaInterface* rg){ reg = rg;}
  //MapAreaInterface:

  //Prob is outside
  double probIsInside(const MapEntryInterface *e)const{
    return reg ? 1 - reg->probIsInside(e): 1;
  }
  double probIsInside(const ObservationInterface *o,
                      const RobotPose *r)const{
    return reg ? 1 - reg->probIsInside(o,r): 1;
  }

  double probIsInside(double x, double y)const{
    return reg ? 1 - reg->probIsInside(x,y): 1;
  }
};



#endif
