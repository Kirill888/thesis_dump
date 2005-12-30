#ifndef RANDOM_H
#define RANDOM_H

#ifndef DEPEND
#include <stdlib.h>
#include <string.h>
#endif /* !DEPEND */

class RandomNumber{
 public:
  virtual double nextDoubleRandom()const{return 0.0;}
  virtual double nextRandom()const{return nextDoubleRandom();}
  virtual int    nextIntRandom()const{return 0;}
};

class UniformRandomNumber: public RandomNumber{
 private:
  int iMin;
  int iMax;

  double dMin;
  double dMax;
  double dScale;

 public:
  UniformRandomNumber(){
    iMin = 0;
    iMax = 1;

    dMin = 0.0;
    dMax = 1.0;
    dScale = (dMax-dMin)/RAND_MAX;
  }

  UniformRandomNumber(const UniformRandomNumber &copy){
    iMin = copy.iMin;
    iMax = copy.iMax;

    dMin = copy.dMin;
    dMax = copy.dMax;
    dScale = copy.dScale;
  }

  void operator=(const UniformRandomNumber &copy){
    iMin = copy.iMin;
    iMax = copy.iMax;

    dMin = copy.dMin;
    dMax = copy.dMax;
    dScale = copy.dScale;
  }

  UniformRandomNumber(int min, int max){
    iMin = min;
    iMax = max;

    dMin = min;
    dMax = max;
    dScale = (dMax-dMin)/RAND_MAX;
  }

  UniformRandomNumber(int min, int max,int seed){
    iMin = min;
    iMax = max;

    dMin = min;
    dMax = max;
    dScale = (dMax-dMin)/RAND_MAX;

    srand(seed);
  }

  UniformRandomNumber(double min, double max){
    iMin = (int)min;
    iMax = (int)max;

    dMin = min;
    dMax = max;
    dScale = (dMax-dMin)/RAND_MAX;
  }

  UniformRandomNumber(double min, double max,int seed){
    iMin = (int)min;
    iMax = (int)max;

    dMin = min;
    dMax = max;
    dScale = (dMax-dMin)/RAND_MAX;

    srand(seed);
  }
  double nextDoubleRandom()const{return dMin + dScale*rand();}
  double nextRandom()const{return nextDoubleRandom();}

  int    nextIntRandom()const{
    int ii = iMin + (int)(dScale*rand());
    while(ii > iMax) ii = iMin + (int)(dScale*rand());
    return ii;
  }
};

class GaussianRandomNumber: public RandomNumber{
 private:
  UniformRandomNumber m_uniRand;
  double m_mean;
  double m_standartDeviation;

  mutable double m_gauss1;
  mutable double m_gauss2;
  mutable bool   m_second;

  mutable double m_md1;
  mutable double m_md2;
  mutable double m_md;
 public:

  GaussianRandomNumber(){
    m_uniRand = UniformRandomNumber(0.0,1.0);
    m_mean = 0.0;
    m_standartDeviation = 1.0;
    m_second = false;
    m_gauss1 = m_gauss2 = 0;
  }

  GaussianRandomNumber(const GaussianRandomNumber &copy){
    m_uniRand = copy.m_uniRand;
    m_mean = copy.m_mean;
    m_standartDeviation = copy.m_standartDeviation;
    
    m_gauss1 = copy.m_gauss1;
    m_gauss2 = copy.m_gauss2;
    m_second = copy.m_second;
  }

  const GaussianRandomNumber& operator=(const GaussianRandomNumber &copy){
    m_uniRand = copy.m_uniRand;
    m_mean = copy.m_mean;
    m_standartDeviation = copy.m_standartDeviation;
    
    m_gauss1 = copy.m_gauss1;
    m_gauss2 = copy.m_gauss2;
    m_second = copy.m_second;

    return *this;
  }

  GaussianRandomNumber(double mean,
		       double standartDeviation){

    m_uniRand = UniformRandomNumber(0.0,1.0);
    m_mean = mean;
    m_standartDeviation = standartDeviation;
    m_second = false;
  }

  GaussianRandomNumber(double mean,
		       double standartDeviation,
		       int seed){

    m_uniRand = UniformRandomNumber(0.0,1.0,seed);
    m_mean = mean;
    m_standartDeviation = standartDeviation;
    m_second = false;
  }

  double nextDoubleRandom()const;
  double nextRandom()const{return nextDoubleRandom();}

  int    nextIntRandom()const{return (int)nextDoubleRandom();}

  double getMD()const{return m_md;}
  double getMD2()const{return m_md*m_md;}

  double mean()const{return m_mean;}
  double deviation()const{return m_standartDeviation;}

  void set(double mean,
	   double standartDeviation){
    m_mean = mean;
    m_standartDeviation = standartDeviation;
    m_second = false;
  }

};

/* Cholesky decomposition*/
bool choldc(double out[3][3], const double a[3][3]);
bool choldc(double out[2][2], const double a[2][2]);

#include "geometry.h"

class RobotPoseSampler{
private:
  double mx, my, ma;
  double L[3][3];
  GaussianRandomNumber gauss;

public:
  RobotPoseSampler():mx(0),my(0),ma(0){
    L[0][0] = 1.0;    L[0][1] = 0.0;    L[0][2] = 0.0;
    L[1][0] = 0.0;    L[1][1] = 1.0;    L[1][2] = 0.0;
    L[2][0] = 0.0;    L[2][1] = 0.0;    L[2][2] = 1.0;
  }

  RobotPoseSampler(const RobotPoseCov &pose):
          mx(pose.x),my(pose.y),ma(pose.rot){
    if(!choldc(L,pose.cov)){
      fprintf(stderr,"RobotPoseSampler: matrix is not positive definite\n");
    }
  }

  RobotPoseSampler(const RobotPoseCov &pose, bool &isOk):
          mx(pose.x),my(pose.y),ma(pose.rot){
    if(choldc(L,pose.cov)){
      isOk = true;
    }else{
      isOk = false;
    }
  }

  bool set(const RobotPoseCov &pose){
    if(!choldc(L,pose.cov)){
      return false;
    }
    mx = pose.x;
    my = pose.y;
    ma = pose.rot;

    return true;
  }

  RobotPose sample()const{
    double x,y,a;
    x = gauss.nextRandom();
    y = gauss.nextRandom();
    a = gauss.nextRandom();

    a = x*L[0][2] + y*L[1][2] + a*L[2][2] + ma;
    y = x*L[0][1] + y*L[1][1]             + my;
    x = x*L[0][0]                         + mx;


    return RobotPose(x,y,a);
  }

  void sample(RobotPose *p)const{
    p->x = gauss.nextRandom();
    p->y = gauss.nextRandom();
    p->rot = gauss.nextRandom();

    p->rot =  p->x*L[0][2] +  p->y*L[1][2] +  p->rot*L[2][2] + ma;
    p->y   =  p->x*L[0][1] +  p->y*L[1][1]                   + my;
    p->x   =  p->x*L[0][0]                                   + mx;
  }
};

class Gaussian3dSampler{
 private:
  double mx,my,mz;
  double L[3][3];
  GaussianRandomNumber gauss;

 public:
  Gaussian3dSampler():mx(0),my(0),mz(0){
    L[0][0] = 1.0;    L[0][1] = 0.0;    L[0][2] = 0.0;
    L[1][0] = 0.0;    L[1][1] = 1.0;    L[1][2] = 0.0;
    L[2][0] = 0.0;    L[2][1] = 0.0;    L[2][2] = 1.0;
  }

  Gaussian3dSampler(double x, double y, double z
		    ,const double cov[3][3]
		    ,bool *isOK = NULL):mx(x),my(y),mz(z){

    bool fine = choldc(L,cov);
    if(isOK != NULL){ *isOK = fine;}
  }

  void sample(double* x, double *y, double* z)const{
    double xx,yy,zz;
    xx = gauss.nextRandom();
    yy = gauss.nextRandom();
    zz = gauss.nextRandom();

    zz = xx*L[0][2] + yy*L[1][2] + zz*L[2][2] + mz;
    yy = xx*L[0][1] + yy*L[1][1]              + my;
    xx = xx*L[0][0]                           + mx;

    *x = xx; *y = yy; *z = zz;
  }

  bool set(double x, double y, double z, const double cov[3][3]){
    mx = x; my = y; mz = z;
    return choldc(L,cov);
  }

  void set(double x, double y, double z,
           double sxx, double syy, double saa){
    mx = x; my = y; mz = z;
    memset(L,0,9*sizeof(double));
    L[0][0] = sqrt(sxx);
    L[1][1] = sqrt(syy);
    L[2][2] = sqrt(saa);
  }
};

class Gaussian2dSampler{
 private:
  double mx, my;
  double L[2][2];
  GaussianRandomNumber gauss;

 public:

  Gaussian2dSampler():mx(0),my(0){
    L[0][0] = L[1][1] = 1;
    L[0][1] = L[1][0] = 0;
  }

  Gaussian2dSampler(const Gaussian2d g):mx(g.x),my(g.y){
    choldc(L,g.cov);
  }

  Gaussian2dSampler(double x, double y, const double cov[2][2]):mx(x),my(y){
    choldc(L,cov);
  }

  void set(const Gaussian2d &g){
    set(g.x, g.y, g.cov);
  }

  void set(double x, double y, const double cov[2][2]){
    mx = x; my = y;
    choldc(L,cov);
  }

  Point sample()const{
    double x;    double y;
    sample(&x,&y); 
    return Point(x,y);
  }

  void sample(Point *p)const{
    sample(&p->x, &p->y);
  }

  void sample(double *x, double *y)const{
    register double xx = gauss.nextRandom();
    register double yy = gauss.nextRandom();

    *x = xx*L[0][0]              + mx;
    *y = xx*L[0][1] + yy*L[1][1] + my;
  }

};


#endif








