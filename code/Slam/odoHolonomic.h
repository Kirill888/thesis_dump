#ifndef __ODO_HOLONOMIC_H__
#define __ODO_HOLONOMIC_H__

#include "util.h"
#include "random.h"
#include "odoModel.h"

class RobotPoseOdo: public OdometryReadingInterface, public RobotPose{
 public:
  double TimeStamp;

  RobotPoseOdo(const RobotPose &r, double t):RobotPose(r){
    TimeStamp = t;
  }

  RobotPose getPose()const{
    return RobotPose(*this);
  }
};

class OdoControlInput: public OdoControlInputInterface{
 private:
  double Vt_;
  double dir_;
  double Vr_;
  double dt_;

  double slip[3];

 public:
  OdoControlInput():Vt_(0),dir_(0),Vr_(0),dt_(1){
    slip[0] = slip[1] = slip[2] = 0.0;
  }

  OdoControlInput(double Vt, double dir, double Vr
		  ,double dt):Vt_(Vt),dir_(dir),Vr_(Vr),dt_(dt){
    slip[0] = slip[1] = slip[2] = 0.0;
  }

  ~OdoControlInput(){;}

  void set(double Vt, double dir, double Vr, double dt){
    Vt_ = Vt;
    dir_  = dir;
    Vr_  = Vr;
    dt_ = dt;
  }

  void advance(RobotPose *prev)const{  advance(prev,dt_); }
  void advance(RobotPose *prev, double dt)const;

  OdoControlInputInterface* clone()const{
    OdoControlInput *u = new OdoControlInput(Vt_,dir_,Vr_,dt_);
    memcpy(u->slip,slip,3*sizeof(double));
    return u;
  }

  void reset(){
    Vt_ = dir_ = Vr_ =  0.0;
    dt_ = 1.0;
    slip[0] = slip[1] = slip[2] = 0.0;
  }

  double Vt()const{return Vt_;}
  double dir()const{return dir_;}
  double Vr()const{return Vr_;}
  double dt()const{return dt_;}

  void setSlippage(double xx, double yy, double rr){
    slip[0] = xx; slip[1] = yy; slip[2] = rr;
  }
};

class HolonomicNoiseParams{
 public:
  double minMoveDistance;
  double minMoveRotation;

  double error_time_proportion;
  double error_time_constant;
  double error_translation_proportion;
  double error_translation_constant;
  double error_rotation_proportion;
  double error_rotation_constant;
  double error_rotation_per_meter;
  double error_heading_angle_constant;

  bool model_slippage;
  double slippage_magnitude_xy;
  double slippage_magnitude_rotation;
  double slippage_frequency;

  HolonomicNoiseParams():
    minMoveDistance(0.001),
    minMoveRotation(0.0001),
    error_time_proportion(0.1),
    error_time_constant(0.0),
    error_translation_proportion(0.1),
    error_translation_constant(0.00001),
    error_rotation_proportion(0.1),
    error_rotation_constant(0.0087),
    error_rotation_per_meter(0.0087),
    error_heading_angle_constant(0.0087),

    model_slippage(false),
    slippage_magnitude_xy(0.0),
    slippage_magnitude_rotation(0.0),
    slippage_frequency(0.0)
    {;}
};

class HolonomicMotionModel: public MotionModelInterface{
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

  HolonomicNoiseParams params;

 public:
  HolonomicMotionModel();
  HolonomicMotionModel(const HolonomicMotionModel &other);

  ~HolonomicMotionModel(){;}

  void setParams(const HolonomicNoiseParams& p);

  void newOdometry(const OdometryReadingInterface*);

  const OdoControlInputInterface* getControlInput()const{return &u;}
  OdoControlInputInterface* sampleControlInput()const;

  RobotPose getTruePose(double t)const{
    RobotPose robot(odo_curr);
    u.advance(&robot, t - t_curr); 
    return robot;
  }
      
  MotionModelInterface* clone()const{return new HolonomicMotionModel(*this);}

  double getDT(double Time)const{return Time - t_curr;}

  double getRotVelocity()const{ return Vr;}
  double getTransVelocity()const{ return Vt;}

  bool hasMoved()const{ return movedD||movedR;}
  bool hasMovedD()const{ return movedD;}
  bool hasMovedR()const{ return movedR;}
};

#endif
