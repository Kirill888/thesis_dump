#ifndef GEOMETRY_H
#define GEOMETRY_H

#ifndef DEPEND
#include <math.h>
#include <stdio.h>  //FILE*
#endif 

#define WITHIN_NN(lo,x,hi) ((x)>(lo) && (x)<(hi))
#define WITHIN_EN(lo,x,hi) ((x)>=(lo) && (x)<(hi))
#define WITHIN_NE(lo,x,hi) ((x)>(lo) && (x)<=(hi))
#define WITHIN_EE(lo,x,hi) ((x)>=(lo) && (x)<=(hi))

#define SQUARE(x) ((x)*(x))
#define POW2(x) ((x)*(x))
#define POW3(x) ((x)*(x)*(x))
#define POW4(x) ((x)*(x)*(x)*(x))

#include "AngleTools.h"
#include "matrix2.h"

extern const double DEGTORAD;
extern const double RADTODEG;

extern const double Inf;
extern const double NaN;

#define LN_2PI 1.8378770664093453390819377091247588L

void rotate_cov(double out[2][2], const double in[2][2], double a);
void rotate_cov(double out[2][2], const double in[2][2], double ca, double sa);
void rotate_cov(double cov[2][2],double a);
void rotate_cov(double cov[2][2],double ca, double sa);

class Point{
 public:
  double x;
  double y;

  Point(){ x=y=0;}
  Point(double X, double Y){ x = X; y = Y;}

  double distance(const Point &p)const;
  double dist2(const Point &p)const;

  void set(double xx, double yy){ x = xx; y = yy;}

  //Translate point 
  Point translate(double xc,double yc, double ca, double sa)const;
  Point translate(double xc,double yc, double theta)const{
    return translate(xc,yc,cos(theta),sin(theta));
  }
  Point translate(Point p, double theta)const{
    return translate(p.x,p.y,theta);
  }

  virtual void rotateMe(double a){ rotateMe(cos(a), sin(a));}
  virtual void rotateMe(double ca, double sa);

  virtual void translateMe(const Point &p){ translateMe(p.x, p.y); }
  virtual void translateMe(double xx, double yy){
    x += xx;
    y += yy;
  }

  virtual void translateMe(Point p, double theta){
    translateMe(p.x,p.y,theta);
  }

  virtual void translateMe(double x, double y, double theta){
    translateMe(x,y,cos(theta),sin(theta));
  }
  virtual void translateMe(double x, double y, double ca, double sa);

  void toPolar(double *radius, double *angle)const;

  virtual Point operator+(const Point &a){
    return Point(x + a.x , y + a.y);
  }

  virtual Point operator-(const Point &a){
    return Point(x - a.x , y - a.y);
  }

  virtual Point operator*(double a){
    return Point(x * a , y * a);
  }

  virtual Point operator/(double a){
    return Point(x / a , y / a);
  }

  const Point & operator+=(const Point &a){
    x += a.x; 
    y += a.y;
    return *this;
  }

  const Point & operator-=(const Point &a){
    x -= a.x; 
    y -= a.y;
    return *this;
  }

  const Point & operator*=(double a){
    x *= a; 
    y *= a;
    return *this;
  }

  const Point & operator/=(double a){
    x /= a; 
    y /= a;
    return *this;
  }


};



class RobotPose: public Point{
 public:
  double rot;

  RobotPose():Point(0,0){
    rot = 0;
  }

  RobotPose(double xx, double yy, double rr):
    Point(xx,yy){
    rot = rr;
  }

  RobotPose(Point p, double rr):Point(p.x,p.y){
    rot = rr;
  }

  RobotPose(const RobotPose &copy):Point(copy.x,copy.y){
    rot = copy.rot;
  }

  void set(Point p, double rr){
    set(p.x,p.y,rr);
  }

  void set(const RobotPose & p){
    set(p.x,p.y,p.rot);
  }

  void set(double xx, double yy, double rr){
    x   = xx;
    y   = yy;
    rot = rr;
  }

  void operator=(const RobotPose &copy){
    x   = copy.x;
    y   = copy.y;
    rot = copy.rot;
  }

  bool operator==(const RobotPose &other){
    return (other.x   == x &&
	    other.y   == y &&
	    other.rot == rot);
  }

  bool operator!=(const RobotPose &other){
    return (other.x   != x ||
	    other.y   != y ||
	    other.rot != rot);
  }

  RobotPose operator*(double scaler)const{
    return RobotPose(x*scaler, y*scaler, rot*scaler);  
  }

  RobotPose operator-(const RobotPose &a)const{
    return RobotPose(x - a.x, y - a.y, angleDiffRad(rot,a.rot));
  }

  RobotPose operator+(const RobotPose &a)const{
    return RobotPose(x + a.x, y + a.y, angleDiffRad(rot, -a.rot));
  }

  const RobotPose& operator+=(const RobotPose &a){
    x   += a.x;
    y   += a.y;
    rot += a.rot;

    return *this;
  }

  const RobotPose& operator-=(const RobotPose &a){
    x   -= a.x;
    y   -= a.y;
    rot -= a.rot;

    return *this;
  }

  const RobotPose& operator*=(double a){
    x   *= a;
    y   *= a;
    rot *= a;

    return *this;
  }

  const RobotPose& operator/=(double a){
    x   /= a;
    y   /= a;
    rot /= a;

    return *this;
  }

  virtual void translateMe(const RobotPose &p){ 
    translateMe(p.x, p.y, p.rot); 
  }

  virtual void translateMe(double xc, double yc, double theta){
    translateMe(xc,yc,theta,cos(theta),sin(theta));
  }

  virtual void translateMe(double xc, double yc, 
                           double theta, double ca, double sa);

  virtual RobotPose translate(double xc, double yc, 
		              double theta)const{
    RobotPose translated(*this);

    translated.translateMe(xc,yc,theta);
    return translated;
  }

};

class Gaussian2d: public Point{
 public:
  double cov[2][2];

  Gaussian2d(double X, double Y, const double COV[2][2]):Point(X,Y){
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
  }

  Gaussian2d(double X, double Y
		,double cov00
		,double cov01
		,double cov10
		,double cov11):Point(X,Y){
    cov[0][0] = cov00;
    cov[0][1] = cov01;
    cov[1][0] = cov10;
    cov[1][1] = cov11;
  }

  Gaussian2d():Point(0,0){
    cov[0][0] = 1;
    cov[0][1] = 0;
    cov[1][0] = 0;
    cov[1][1] = 1;
  }

  Gaussian2d(const Gaussian2d &copy):Point(copy.x,copy.y){
    cov[0][0] = copy.cov[0][0];
    cov[0][1] = copy.cov[0][1];
    cov[1][0] = copy.cov[1][0];
    cov[1][1] = copy.cov[1][1];
  }

  void set(const Gaussian2d &other){ set(other.x, other.y, other.cov); }

  void set(double X, double Y, const double COV[2][2]){
    x = X;
    y = Y;
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
  }


  Gaussian2d translate(double x, double y, double a)const{
    return translate(x,y,cos(a),sin(a));
  }

  Gaussian2d translate(const RobotPose &p)const{
    return translate(p.x,p.y,cos(p.rot),sin(p.rot));
  }

  Gaussian2d translate(double x, double y, 
			  double ca, double sa)const;

  void translateMe(double xx, double yy){
    x -= xx;
    y -= yy;
  }

  void translateMe(const Point & p){ translateMe(p.x, p.y);}

  void translateMe(const Point &p, double theta){
    translateMe(p.x,p.y,cos(theta),sin(theta));
  }

  void translateMe(const RobotPose &p){
    translateMe(p.x,p.y,cos(p.rot),sin(p.rot));
  }

  void translateMe(double x, double y, double theta){
    translateMe(x,y,cos(theta),sin(theta));
  }

  void translateMe(double x, double y, double ca, double sa);

  double mahalanobis2(const Point &xx)const{
    return mahalanobis2(xx.x,xx.y);
  }
  double mahalanobis2(double x, double y)const;

  Gaussian2d covIntersect(const Gaussian2d &p, double w)const;
  Gaussian2d covIntersect(const Gaussian2d &p)const;
  double probMatch(const Gaussian2d &p)const;
  double probMatchLog(const Gaussian2d &p)const;


  int debugDump(FILE *f,char *s = "")const{
    return fprintf(f,"%s %6.3f %6.3f\n"
		   "[%6.3f %6.3f]\n"
		   "[%6.3f %6.3f] / 1000\n"
		   ,s
		   ,x,y
		   ,cov[0][0]*1000
		   ,cov[0][1]*1000
		   ,cov[1][0]*1000
		   ,cov[1][1]*1000);
  }

 private:
  void Gaussian2d::covIntersect(double c[2][2], double cx[2]
				   ,const double a_inv[2][2],const double ax[2]
				   ,const double b_inv[2][2],const double bx[2]
				   ,double w)const;


};

class Gaussian3d{
 public:
  double x,y,z;
  double cov[3][3];

  Gaussian3d():x(0),y(0),z(0){
    cov[0][0] = 0;
    cov[0][1] = 0;
    cov[0][2] = 0;
    cov[1][0] = 0;
    cov[1][1] = 0;
    cov[1][2] = 0;
    cov[2][0] = 0;
    cov[2][1] = 0;
    cov[2][2] = 0;
  }

  Gaussian3d(double xx, double yy, double zz, const double COV[3][3]):
    x(xx),y(yy),z(zz){
    copy_cov(cov,COV);
  }

  void translateMe(const RobotPose &robot){
    translateMe(robot.x, robot.y, robot.rot);
  }

  void translateMe(const RobotPose *robot){
    translateMe(robot->x, robot->y, robot->rot);
  }

  void translateMe(double xc, double yc, 
			   double theta){
    translateMe(xc,yc,cos(theta),sin(theta));
  }

  void translateMe(double xc, double yc, 
                   double ca, double sa);

  double mahalanobis2(const Gaussian3d& z)const{
    return mahalanobis2(z.x,z.y,z.z);
  }

  double mahalanobis2(double x, double y, double z)const;

  void set(double xx, double yy, double zz, const double COV[3][3]){
    x = xx; y = yy; z = zz;
    copy_cov(cov,COV);
  }

  operator Gaussian2d ()const{
    double COV[2][2];
    COV[0][0] = cov[0][0];
    COV[0][1] = cov[0][1];
    COV[1][0] = cov[1][0];
    COV[1][1] = cov[1][1];

    return Gaussian2d(x,y,COV);
  }

  double probMatch(const Gaussian3d& other)const;
  double probMatchLog(const Gaussian3d &other)const;

 private:


  static void copy_cov(double cov[3][3], const double COV[3][3]){
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[0][2] = COV[0][2];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
    cov[1][2] = COV[1][2];
    cov[2][0] = COV[2][0];
    cov[2][1] = COV[2][1];
    cov[2][2] = COV[2][2];
  }
};

class RobotPoseCov: public RobotPose{
 public:
  double cov[3][3];

  RobotPoseCov():RobotPose(0,0,0){
    cov[0][0] = 0;
    cov[0][1] = 0;
    cov[0][2] = 0;
    cov[1][0] = 0;
    cov[1][1] = 0;
    cov[1][2] = 0;
    cov[2][0] = 0;
    cov[2][1] = 0;
    cov[2][2] = 0;
  }

  RobotPoseCov(double xx, double yy, double aa, const double COV[3][3]):
    RobotPose(xx,yy,aa){
    copy_cov(cov,COV);
  }

  RobotPoseCov(const RobotPose &p,const double COV[3][3]):RobotPose(p){
    copy_cov(cov,COV);
  }

  void propagateMe(const RobotPoseCov & odo0);

  RobotPoseCov propagate(const RobotPoseCov &odo0)const{
    RobotPoseCov odo = *this;
    odo.propagateMe(odo0);
    return odo;
  }

  Gaussian2d propagate(const Gaussian2d &obs)const{
    Gaussian2d o(obs);
    propagate(&o);
    return o;
  }

  void propagate(Gaussian2d *obs)const;

  void translateMe(double xc, double yc, 
			   double theta){
    translateMe(xc,yc,theta,cos(theta),sin(theta));
  }

  void translateMe(double xc, double yc, 
                   double theta, double ca, double sa);

  void set(const RobotPoseCov& o){
    RobotPose::set(o);
    copy_cov(cov,o.cov);
  }

  void set(double xx, double yy, double aa){
    RobotPose::set(xx,yy,aa);
  }

  void set(double xx, double yy, double aa, const double COV[3][3]){
    RobotPose::set(xx,yy,aa);
    copy_cov(cov,COV);
  }

  void set(const RobotPose &p, const double COV[3][3]){
    RobotPose::set(p.x, p.y, p.rot);
    copy_cov(cov, COV);
  }

  double mahalanobis2(const RobotPose& z)const;
  double prob(const RobotPose &x)const;

  double norm()const;
  void scaleCov(double scaler){
    cov[0][0] *= scaler;    cov[0][1] *= scaler;    cov[0][2] *= scaler;
    cov[1][0] *= scaler;    cov[1][1] *= scaler;    cov[1][2] *= scaler;
    cov[2][0] *= scaler;    cov[2][1] *= scaler;    cov[2][2] *= scaler;
  }

 private:

  static  void copy_cov(double cov[3][3], const double COV[3][3]){
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[0][2] = COV[0][2];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
    cov[1][2] = COV[1][2];
    cov[2][0] = COV[2][0];
    cov[2][1] = COV[2][1];
    cov[2][2] = COV[2][2];
  }
};


double mahalanobis2(const Point &x, const Gaussian2d &obs);


////////////////////////////////////////////////////////////
// Line Class
////////////////////////////////////////////////////////////

class Line{
 private:
  //  ax + by + c = 0
  double a, b, c;
		
 public:

  Line():a(0),b(0),c(0){;}

  Line(const Point &p1, const Point &p2){
    set(p1.x,p1.y,p2.x,p2.y);
  }

  inline void set(double x1, double y1, double x2, double y2);

  // (a*a + b*b) must equal 1
  void set(double A, double B, double C){ a = A; b = B; c = C; }

  double distance(const Point& p)const{return distance(p.x,p.y);}
  double distance(const Point* p)const{return distance(p->x,p->y);}
  double distance(double x,double y)const{
    return a*x + b*y + c;
  }

  double A()const{return a;}
  double B()const{return b;}
  double C()const{return c;}
  
  bool intersect(double *x, double *y, const Line &line)const;

  void project(double *xo, double *yo, double x, double y);

};

////////////////////////////////////////////////////////////
// Line Segment
////////////////////////////////////////////////////////////
class LineSegment{
 private:
  double x1,y1;
  double x2,y2;

  double angle;
  double ca;
  double sa;

  double length;

 public:

  LineSegment():x1(0),y1(0),x2(0),y2(0),angle(0),ca(1),sa(0),length(0){;}
  LineSegment(double x1, double y1
	      ,double x2, double y2){
    set(x1,y1,x2,y2);
  }

  void set(double x1, double y1, double x2, double y2);

  double X1()const{ return x1; }
  double X2()const{ return x2; }
  double Y1()const{ return y1; }
  double Y2()const{ return y2; }

  double Angle() const{return angle;  }
  double Length()const{return length; }

  double CA()const{ return ca; }
  double SA()const{ return sa; }
};

struct LineLaserMatchParams{
  double sigma_r;       //Sigma Range
  double sigma_a;       //Sigma Angle
  double sigma_r2;      //squared
  double sigma_a2;      //squared

  double maxRange;      //Maximum laser range
  double maxDistToLine; //Maximum distance to match point to line
};

//Returns: number of affected rays ( n - n_nomatch )
int matchScanToLine(double *wlog  //-- output
		    ,int *match   //-- output: 0-nomatch , 1-match,
		                  //           2-conflict (crosses the line)
		    ,const double *range
		    ,const double *angle
		    ,int n
		    ,const RobotPose* sensor
		    ,const LineSegment* line
		    ,const struct LineLaserMatchParams *params);

//Fit lines to: 1->nPointsPerLine, 2->nPointsPerLine+1 ... and so on
//x[i] y[i] can be NaN, then dist2 of all lines containing such point will 
// be set to NaN
// dist2 is a sum of distances from points to line
void fitLinesToScan(Line* lines, double *dist2
		    ,const double *x
		    ,const double *y
		    ,int n
		    ,int nPointsPerLine);


// Inlines
/////////////////////////////////////////////////////////////////

inline void rotate_cov(double out[2][2], const double in[2][2], 
                       double a){
  rotate_cov(out,in,cos(a),sin(a));
}

inline void rotate_cov(double out[2][2], const double in[2][2], 
                       double ca, double sa){
  register double p00 = in[0][0];
  register double p01 = in[0][1];
  register double p10 = in[1][0];
  register double p11 = in[1][1];

  register double caca = ca*ca;
  register double casa = ca*sa;
  double & saca = casa;
  register double sasa = sa*sa;
  
  out[0][0]  = caca*p00 - saca*p10 - casa*p01 + sasa*p11;
  out[1][0]  = saca*p00 + caca*p10 - sasa*p01 - casa*p11;
  out[0][1]  = casa*p00 - sasa*p10 + caca*p01 - saca*p11;
  out[1][1]  = sasa*p00 + casa*p10 + saca*p01 + caca*p11;
}

inline void rotate_cov(double cov[2][2],double a){
  rotate_cov(cov,cos(a),sin(a));
}

inline void rotate_cov(double cov[2][2],double ca, double sa){

  register double p00 = cov[0][0];
  register double p01 = cov[0][1];
  register double p10 = cov[1][0];
  register double p11 = cov[1][1];

  register double caca = ca*ca;
  register double casa = ca*sa;
  double & saca = casa;
  register double sasa = sa*sa;
  
  cov[0][0]  = caca*p00 - saca*p10 - casa*p01 + sasa*p11;
  cov[1][0]  = saca*p00 + caca*p10 - sasa*p01 - casa*p11;
  cov[0][1]  = casa*p00 - sasa*p10 + caca*p01 - saca*p11;
  cov[1][1]  = sasa*p00 + casa*p10 + saca*p01 + caca*p11;
}

inline void rotate_cov(double cov[3][3],double ca, double sa){
  double caca = ca*ca;
  double casa = ca*sa;
  double sasa = sa*sa;

  double sxx = cov[0][0];
  double syy = cov[1][1];
  //  double saa = cov[2][2];
  double sxy = cov[0][1];
  double sxa = cov[0][2];
  double sya = cov[1][2];

  cov[0][0] = caca*sxx	- 2*casa*sxy	+ sasa*syy;
  cov[1][1] = sasa*sxx	+ 2*casa*sxy	+ caca*syy;
  //  cov[2][2] = saa;

  //sxy
  cov[0][1] = cov[1][0] = casa*sxx + (caca-sasa)*sxy	- casa*syy;

  //sxa
  cov[0][2] = cov[2][0] = ca*sxa   - sa*sya;

  //sya
  cov[1][2] = cov[2][1] = sa*sxa   + ca*sya;

}

//inline void inverse3x3cov(double inv[3][3]

extern double volumeOfTheProduct(const Gaussian2d &l1, 
                                 const Gaussian2d &l2);
extern double logVolumeOfTheProduct(const Gaussian2d &l1, 
                                    const Gaussian2d &l2);
extern double volumeOfTheProduct(const Gaussian3d &l1, 
                                 const Gaussian3d &l2);
extern double logVolumeOfTheProduct(const Gaussian3d &l1, 
                                    const Gaussian3d &l2);

// RobotPose inlines
///////////////////////////////////////////////////////////////////

inline void RobotPose::translateMe(double xc, double yc, double theta, 
                            double ca, double sa){
  double xn;

  xn = x*ca - y*sa + xc;
  y =  x*sa + y*ca + yc;

  x = xn;
  rot = angleDiffRad(rot,-theta);
#if 0
  printf("TranslateMeBy: %.3f %.3f %.3f deg\n",
	 xc,yc,theta*RADTODEG);

  printf("TranslateMe: %.3f %.3f %.3f deg\n",
	 x,y,rot*RADTODEG);
#endif

}

// Gaussian2d inlines
///////////////////////////////////////////////////////////////////
inline double Gaussian2d::probMatchLog(const Gaussian2d &p)const{
    return logVolumeOfTheProduct(*this,p);
}

////////////////////////////////////////////////////////////////
// Gaussian3d inlines
////////////////////////////////////////////////////////////////

inline void Gaussian3d::translateMe(double xc, double yc, 
                            double ca, double sa){
  double xn;

  xn = x*ca - y*sa + xc;
  y  = x*sa + y*ca + yc;
  x = xn;

  rotate_cov(cov,ca,sa);
}

inline double Gaussian3d::probMatchLog(const Gaussian3d &p)const{
    return logVolumeOfTheProduct(*this,p);
}
inline double Gaussian3d::probMatch(const Gaussian3d &p)const{
    return volumeOfTheProduct(*this,p);
}

inline double Gaussian3d::mahalanobis2(double xx, double yy, double zz)const{
  const double &sxx = cov[0][0];
  const double &syy = cov[1][1];
  const double &saa = cov[2][2];
  const double &sxy = cov[0][1];
  const double &sxa = cov[0][2];
  const double &sya = cov[1][2];

  double b = 1.0/(sxx*syy*saa - sxx*sya*sya - sxy*sxy*saa 
		  + 2*sxy*sxa*sya - sxa*sxa*syy);

  double sxx_ =  syy*saa - sya*sya;
  double syy_ =  sxx*saa - sxa*sxa;
  double saa_ =  sxx*syy - sxy*sxy;

  double sxy_ = -sxy*saa + sxa*sya;
  double sxa_ =  sxy*sya - sxa*syy;
  double sya_ = -sxx*sya + sxy*sxa;

  double dx   = x - xx;
  double dy   = y - yy;
  double dz   = z - zz;

  double md2 = dx*dx*sxx_ + 2*dx*dy*sxy_ + 2*dx*dz*sxa_ 
             + dy*dy*syy_ + 2*dy*dz*sya_ + dz*dz*saa_;

  return b*md2;
}


////////////////////////////////////////////////////////////////
// RobotPoseCov inlines
////////////////////////////////////////////////////////////////
inline void RobotPoseCov::translateMe(double xc, double yc, double theta, 
                            double ca, double sa){
  RobotPose::translateMe(xc,yc,theta,ca,sa);
  rotate_cov(cov,ca,sa);
}

inline void RobotPoseCov::propagateMe(const RobotPoseCov & odo0){
  double ca = cos(odo0.rot);
  double sa = sin(odo0.rot);
  double a = -x*sa - y*ca;
  double b = +x*ca - y*sa;

  double cov0[3][3];

  cov0[0][0] = odo0.cov[0][0] + a*odo0.cov[0][2] + 
               (odo0.cov[0][2] + a*odo0.cov[2][2])*a;

  cov0[1][1] = odo0.cov[1][1] + b*odo0.cov[1][2] + 
              (odo0.cov[1][2] + b*odo0.cov[2][2])*b;

  cov0[2][2] = odo0.cov[2][2];

  cov0[0][1] = cov0[1][0] = odo0.cov[0][1] + b*odo0.cov[0][2]+
                           (odo0.cov[1][2] + b*odo0.cov[2][2])*a;

  cov0[0][2] = cov0[2][0] = odo0.cov[0][2] + a*odo0.cov[2][2];
  cov0[1][2] = cov0[2][1] = odo0.cov[1][2] + b*odo0.cov[2][2];



  translateMe(odo0.x, odo0.y, odo0.rot, ca,sa);

  cov[0][0] += cov0[0][0];
  cov[0][1] += cov0[0][1];
  cov[0][2] += cov0[0][2];
  //cov[1][0] += cov0[1][0];
  cov[1][1] += cov0[1][1];
  cov[1][2] += cov0[1][2];
  //  cov[2][0] += cov0[2][0];
  //  cov[2][1] += cov0[2][1];
  cov[2][2] += cov0[2][2];

  cov[1][0] = cov[0][1];
  cov[2][0] = cov[0][2];
  cov[2][1] = cov[1][2];
}

inline void RobotPoseCov::propagate(Gaussian2d *obs)const{
  double ca = cos(rot);
  double sa = sin(rot);

  double a = -obs->x*ca - obs->y*sa;
  double b = +obs->x*sa - obs->y*ca;

  obs->translateMe(x,y,ca,sa);

  obs->cov[0][0] += cov[0][0] + 2*a*cov[0][2] + a*a*cov[2][2];
  obs->cov[1][1] += cov[1][1] + 2*b*cov[1][2] + b*b*cov[2][2];
  obs->cov[0][1] += cov[0][1] + b*cov[0][2] + a*cov[1][2] + a*b*cov[2][2];

  obs->cov[1][0] = obs->cov[0][1];
}

inline double RobotPoseCov::mahalanobis2(const RobotPose &z)const{
  const double &sxx = cov[0][0];
  const double &syy = cov[1][1];
  const double &saa = cov[2][2];
  const double &sxy = cov[0][1];
  const double &sxa = cov[0][2];
  const double &sya = cov[1][2];

  double b = 1.0/(sxx*syy*saa - sxx*sya*sya - sxy*sxy*saa 
		  + 2*sxy*sxa*sya - sxa*sxa*syy);

  double sxx_ = ( syy*saa - sya*sya);
  double syy_ = ( sxx*saa - sxa*sxa);
  double saa_ = ( sxx*syy - sxy*sxy);

  double sxy_ = (-sxy*saa + sxa*sya);
  double sxa_ = ( sxy*sya - sxa*syy);
  double sya_ = (-sxx*sya + sxy*sxa);

  double dx   = z.x   - x;
  double dy   = z.y   - y;
  double da   = angleDiffRad(z.rot, rot);

  double md2 = dx*dx*sxx_ + 2*dx*dy*sxy_ + 2*dx*da*sxa_ 
             + dy*dy*syy_ + 2*dy*da*sya_ +   da*da*saa_;

  return md2*b;

}

inline double RobotPoseCov::norm()const{
  double COV_xy[2][2] = {{cov[0][0],cov[0][1]},
                         {cov[1][0],cov[1][1]}};
  return matrix2_norm(COV_xy);
}

inline double RobotPoseCov::prob(const RobotPose &z)const{
  double scaler;

  double sxx = cov[0][0];
  double sxy = cov[0][1];
  double sxz = cov[0][2];
  double syy = cov[1][1];
  double syz = cov[1][2];
  double szz = cov[2][2];
 
  //sxx*syy*szz - sxx*syz^2 - sxy^2*szz + 2*sxy*sxz*syz - sxz^2*syy
 
  double det = sxx*syy*szz - 
               sxx*syz*syz - 
               sxy*sxy*szz +
             2*sxy*sxz*syz -
               sxz*sxz*syy;

  scaler = 1.0/sqrt(2*M_PI*det);
  return scaler*exp(-0.5*mahalanobis2(z));
}

////////////////////////////////////////////////////////////
// Line Class Inlines
////////////////////////////////////////////////////////////
inline void Line::set(double x1, double y1, double x2, double y2){
  double dx = x2 - x1;
  double dy = y2 - y1;

  if(dy == 0){
    if(dx == 0){
      a = b = c = 0;
    }else{
      a =  0;   
      if(x2 > x1){
        b =  1;    c = -y1;
      }else{
	b = -1;    c = y1;
      }
    }
  }else if(dx == 0){
    b = 0;
    if(y2 < y1){
       a =  1;  c = -x1;
    }else{
       a = -1;  c =  x1;
    }
  }else{
    double dy_dx = dy/dx;
    if(x2 > x1){
       b =  1.0/sqrt(1 + dy_dx*dy_dx);
    }else{
       b = -1.0/sqrt(1 + dy_dx*dy_dx);
    }

    a = -dy_dx*b;
    c = -a*x1 - b*y1;
  }
}

#endif




