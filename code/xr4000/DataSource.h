#ifndef __DATA_SOURCE_H_
#define __DATA_SOURCE_H_

#ifndef DEPEND
#include <stdio.h> //FILE*
#endif

#include "geometry.h"
#include "util.h"


class LaserSource{
 protected:
  int    npoints;
  double *rr;
  double *aa;
  double *xx;
  double *yy;
  int    *ii;
  double *cos_aa;
  double *sin_aa;

  double  tt;
  double maxRange;

 public:

  LaserSource():npoints(0),rr(NULL),aa(NULL)
                ,xx(NULL),yy(NULL),ii(NULL)
                ,cos_aa(NULL), sin_aa(NULL)
                ,tt(0.0),maxRange(8.183){;}

  LaserSource(double *range, double *angle, int *intensity, 
	      int n, double t):npoints(0),rr(NULL),aa(NULL)
              ,xx(NULL),yy(NULL),ii(NULL),cos_aa(NULL), sin_aa(NULL),
              tt(0.0),maxRange(8.183){
    setData(range,angle,intensity,n,t);
  }

  LaserSource(const LaserSource & other):npoints(0),rr(NULL),aa(NULL)
                ,xx(NULL),yy(NULL),ii(NULL){
    set(&other);
  }

  const LaserSource& operator=(const LaserSource& other){
    if(this != &other){
      set(&other);
    }
    return *this;
  }

  LaserSource* clone()const{
    return new LaserSource(*this);
  }

  void set(const LaserSource* other);

  void destroy(){
    DESTROY_ARRAY(rr);    DESTROY_ARRAY(aa);
    DESTROY_ARRAY(xx);    DESTROY_ARRAY(yy);
    DESTROY_ARRAY(ii);    npoints = 0;
  }

  virtual ~LaserSource(){   destroy();  }

  virtual bool nextScan(){return false;}

  void setData(double *range, double *angle, int *intensity, int n, double t);
  void setMaxRange(double mr){ maxRange = mr;}
  double getMaxRange()const{ return maxRange; }

  int NumOfPoints()const{return npoints;}
  int numPoints()const{return npoints;}
  double x(int i)const{return xx[i];}
  double y(int i)const{return yy[i];}

  double r(int i)const{return rr[i];}
  double a(int i)const{return aa[i];}
  int Intensity(int i)const{return ii[i];}
  
  bool isValidReading(int i)const{ return !isnan(rr[i]);}
  bool noReturn(int i)const{ return rr[i] >= maxRange;}

  double TimeStamp()const{return tt;}

  const double *getR()const{ return rr;}
  const double *getA()const{ return aa;}
  const int    *getI()const{ return ii;}
  const double *getX()const{ return xx;}
  const double *getY()const{ return yy;}
  const double *getCA()const{ return cos_aa;}
  const double *getSA()const{ return sin_aa;}
};

class OdoSource{
   
 protected:
  double xx,yy,aa,tt;

 public:
  OdoSource(){
    setData(0.0,0.0,0.0,0.0);
  };

  OdoSource(double x, double y, double a, double t){
    setData(x,y,a,t);
  };

  virtual ~OdoSource(){;}

  void setData(double x,double y,double a, double t){
    xx = x;
    yy = y;
    aa = a;
    tt = t;
  }
  
  virtual bool next(){return false;}

  double x()const{return xx;}
  double y()const{return yy;}
  double a()const{return aa;}

  double TimeStamp()const{return tt;}

  operator RobotPose()const{return RobotPose(xx,yy,aa);}
};

class FileLaserSource: public LaserSource{
 private:
  FILE *f;
  bool noEOL;

  bool nextScan_noEOL();

  double maxRange;

 public:
  FileLaserSource(char* fileName, double maxRange, bool noEOL = false);

  ~FileLaserSource();

  bool nextScan();

  bool isOk()const{return f!= NULL;}

};

class FileOdoSource: public OdoSource{
 private:
  FILE *f;

 public:
  FileOdoSource(char* fileName);
  ~FileOdoSource();

  bool next();
     
  bool isOk()const{return f!= NULL;}
};


void run_simulation(OdoSource &odo
                   , void (*odoEvent)(const OdoSource &odo)
                   , LaserSource &las
		   , void (*lasEvent)(const LaserSource &odo)
                   , const bool *STOP);


#endif
