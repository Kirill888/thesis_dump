#ifndef __STEREO_PAIR_H__
#define __STEREO_PAIR_H__

#ifndef DEPEND
#include <math.h>
#include <string.h>

#endif

class CameraModel{
 public:
  double focal_u;
  double focal_v;
  double centre_u;
  double centre_v;
  unsigned int n_u;
  unsigned int n_v;

  CameraModel():focal_u(0),focal_v(0),centre_u(0),centre_v(0)
               ,n_u(0),n_v(0){;}

  CameraModel(double fu,double fv, double cu, double cv
	      , unsigned int nu, unsigned int nv):
    focal_u(fu),focal_v(fv),centre_u(cu),centre_v(cv)
    ,n_u(nu),n_v(nv){;}

};

class StereoPair{
 public:
  CameraModel camera1;
  CameraModel camera2;
  RobotPose pose2in1; //Pose of 2nd camera relative to 1st.
  double tilt_angle;
  double tilt_cos;
  double pose2in1_cov[3][3];


  StereoPair(){ memset(pose2in1_cov,0,9*sizeof(double));}

  StereoPair(const CameraModel &c1, const CameraModel &c2,
	     const RobotPose & p, double tilt):
    camera1(c1), camera2(c2),pose2in1(p)
    ,tilt_angle(tilt),tilt_cos(cos(tilt)){;

    memset(pose2in1_cov,0,9*sizeof(double));
  }

  //ShortCut methods
  double X2()const{return pose2in1.x;}
  double Y2()const{return pose2in1.y;}
  double A2()const{return pose2in1.rot;}

  double F1u()const{return camera1.focal_u;}
  double F2u()const{return camera2.focal_u;}
  double F1v()const{return camera1.focal_v;}
  double F2v()const{return camera2.focal_v;}

  double C1u()const{return camera1.centre_u;}
  double C2u()const{return camera2.centre_u;}
  double C1v()const{return camera1.centre_v;}
  double C2v()const{return camera2.centre_v;}

};

#endif
