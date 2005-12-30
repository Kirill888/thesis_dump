#ifndef __ODO_MODEL_H__
#define __ODO_MODEL_H__

#include "geometry.h"



class OdoControlInputInterface{
 public:

  virtual void advance(RobotPose *prev)const{;}
  virtual void advance(RobotPose *prev, double dt)const{;}

  virtual OdoControlInputInterface* clone()const{return NULL;}

  virtual void reset(){;}

  virtual ~OdoControlInputInterface(){;}

};

class OdometryReadingInterface{
 public:
  virtual OdometryReadingInterface * clone()const{return NULL;}
  virtual ~OdometryReadingInterface(){;}
};

class Odometer{

 public:

  Odometer(){ dist_ = 0.0;}

  void reset(){ dist_ = 0.0; }

  double distanceTravelled() const { return dist_; }
  double dist() const { return dist_;}

  void move(double dist){ dist_ += dist;}

 private:
  double dist_;
};

class MotionModelInterface{
 protected:
  Odometer odometer_;

 public:

  virtual void newOdometry(const OdometryReadingInterface*){;}

  virtual const OdoControlInputInterface* getControlInput()const{return NULL;}
  virtual OdoControlInputInterface* sampleControlInput()const{return NULL;}

  virtual RobotPose getTruePose(double t)const{return RobotPose();}
     
  virtual MotionModelInterface* clone()const{return NULL;}

  virtual double getDT(double Time)const{return 0;}

  virtual double getRotVelocity()const{ return 0;}
  virtual double getTransVelocity()const{ return 0;}

  virtual bool hasMoved()const{  return false;}
  virtual bool hasMovedD()const{ return false;}
  virtual bool hasMovedR()const{ return false;}

  Odometer & odometer(){ return odometer_;}

  virtual ~MotionModelInterface(){;}

};

#endif
