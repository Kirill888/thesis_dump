#ifndef __LASER_CORNER_EXTRACTOR_H__
#define __LASER_CORNER_EXTRACTOR_H__

#include "DataSource.h"
#include "LeastSquares2D.h" //DROS: 
//used for LineFittingParamType
//         Line2DCartDataType


extern void initLaserCornerExtractor();

extern int extractCorners(double *range, double *bearing, 
                          const LaserSource &las); 

class ObservationInterface;

extern int extractCorners(ObservationInterface **obs, 
                           const LaserSource &las); 

double FindAdjLine2D(const double *x, const double *y, unsigned int start,
                     unsigned int nData, 
                     int debug, LineFittingParamType *p,
                     Line2DCartDataType *l);

struct CornerDetectorParams{
  int n_fit;
  double maxAvgDistPerPoint;

  double maxConvexDist;
  double minJumpDist;
  double minRayPenetration;

  double angleConvexThresh;    //For Convex/Concave
  double angleCollinearThresh; //For Jump
  double angleMinRayAngle;     //For convex hidden

  double minLineLength;
  double minLineProb;

  CornerDetectorParams(){ //Default values
    n_fit                 = 10;
    maxAvgDistPerPoint    = 0.005;

    maxConvexDist         = 0.2;
    minJumpDist           = 0.05;
    minRayPenetration     = 0.4;

    angleConvexThresh     = deg2rad(30);
    angleCollinearThresh  = deg2rad(10);
    angleMinRayAngle      = deg2rad(20);

    minLineLength         = 0.3;
    minLineProb           = 0.3;
  }

};

class LaserCornerDetector{
 private:
  //Data structures
  struct Corner{
    int type;

    double x;
    double y;
    double range;
    double angle;

    double xt1, yt1;
    double xt2, yt2;
  };

  //Input data
  const double* range;
  const double* angle;
  const double* ca;
  const double* sa;
  const double* x;
  const double* y;
  int n_data;

  //Derived input
  int     sz_lines;
  int     n_lines;
  Line   *lines;
  double *dist2;

  //Parameters
  struct CornerDetectorParams params;
  int n_fit;

  double goodLineThresh;
  double maxConvexDist2;
  double minLineLen2;
  double CONVEX_ANGLE_THRESH;
  double COLLINEAR_THRESH;
  double HIDDEN_RAY_THRESH;

  LineFittingParamType lineFitParams;

  //Corners 
  int sz_corners;
  int n_corners;
  struct Corner *corners;

  //Methods

  void lookForCorners();

  bool checkRay(const Line *l, int i_ray);
  void checkConvexHidden(int iline, bool before, bool after);

  bool checkConvexConcave(int il1, int il2, int i1, int i2,
                          double d1, double d2);

  bool checkJump(int il1, int il2, int i1, int i2,
                 double d1, double d2);

  void addCorner(int type, int i_data, int il, double xt1, double yt1);
  void addCorner(int type, double x, double y, double xt1, double yt1, 
                                               double xt2, double yt2);

  void initLineParams();

  bool checkLineLength(int il, int dir, double *xt, double *yt);

 public:
  LaserCornerDetector();
  LaserCornerDetector(const CornerDetectorParams* p);
  ~LaserCornerDetector(){ destroy(); }

  void setParams(const struct CornerDetectorParams* p);

  int ExtractCorners(const LaserSource &las);

  //Access methods
  double X(int i)const{ return corners[i].x; }
  double Y(int i)const{ return corners[i].y; }
  double R(int i)const{ return corners[i].range; }
  double A(int i)const{ return corners[i].angle; }

  int CornerType(int i)const { return corners[i].type; }

  double XTangentLeft (int i)const{ return corners[i].xt1; }
  double XTangentRight(int i)const{ return corners[i].xt2; }
  double YTangentLeft (int i)const{ return corners[i].yt1; }
  double YTangentRight(int i)const{ return corners[i].yt2; }

  void destroy();
};

#endif
