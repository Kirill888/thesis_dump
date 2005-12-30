#ifndef __ODO_XR4000_H__
#define __ODO_XR4000_H__

#include "odoModel.h"
#include "random.h"
#include "util.h"
#include "odoHolonomic.h"

//XR4000 specific: odometric model.


class MotionModel: public MotionModelInterface{
 private:
  RobotPose odo_curr;
  double t_curr;
  Gaussian3dSampler sampler;
  Gaussian3dSampler slippage;

  UniformRandomNumber dice;
  bool movedD;
  bool movedR;


  OdoControlInput u;
  double Vt;
  double dir;
  double Vr;
  double dt;
  double acc_t;
  double acc_r;

  double P1[4][3];
  double P2[4];

 public:
  MotionModel();
  ~MotionModel(){;}

  MotionModel(const MotionModel &other);

  void newOdometry(const OdometryReadingInterface*);

  const OdoControlInputInterface* getControlInput()const;
  OdoControlInputInterface* sampleControlInput()const;

  RobotPose getTruePose(double t)const{
    RobotPose robot(odo_curr);
    u.advance(&robot, t - t_curr); 
    return robot;
  }
      
  MotionModelInterface* clone()const{return new MotionModel(*this);}

  double getDT(double Time)const{return Time - t_curr;}

  double getRotVelocity()const{ return Vr;}
  double getTransVelocity()const{ return Vt;}

  bool hasMoved()const{ return movedD||movedR;}
  bool hasMovedD()const{ return movedD;}
  bool hasMovedR()const{ return movedR;}

  void setP1(int r, int c, double v){
    P1[r][c] = v;
  }

  void setP2(int i, double v){
    P2[i] = v;
  }

  //Odometer & odometer(){ return odometer_;}

  static double const MIN_MOVE_DIST;
  static double const MIN_MOVE_ROT;

};

/////////////////////////////////////////////////////////////////////////////
//INLINES
/////////////////////////////////////////////////////////////////////////////
inline const OdoControlInputInterface* MotionModel::getControlInput()const{
  return &u;
}


#endif
