#ifndef __ODO_CAR_H__
#define __ODO_CAR_H__

#include "odoModel.h"
#include "random.h"
#include "util.h"

class CarOdoReading: public OdometryReadingInterface{
 public:

  double TimeStamp; //TimeStamp
  double speed;     //Velocity at the center of the rear axis.
  double steer;     //Steering angle

  CarOdoReading():TimeStamp(0),speed(0),steer(0){;} 

  CarOdoReading(double tt, double Speed, double Steer):
    TimeStamp(tt),speed(Speed),steer(Steer){;}

  CarOdoReading(const CarOdoReading *other):
    TimeStamp(other->TimeStamp),speed(other->speed),steer(other->steer){;}

  CarOdoReading(const CarOdoReading &other):
    TimeStamp(other.TimeStamp),speed(other.speed),steer(other.steer){;}

  OdometryReadingInterface * clone()const{
    return new CarOdoReading(this);
  }

};

class CarMotionModelParameters{
 public:
  //Model Parameters
  double L; //Distance between front and rear wheels
  double H; //Wheel to center distance

  double b; //Sensor center x
  double a; //Sensor center y (in a coordinate frame centered 
            //                 in the middle of the rear axis)

  //Defaults to Ute in Victoria park
  CarMotionModelParameters():L(2.83),H(0.76),b(0.5),a(3.78){;}

  CarMotionModelParameters(double LL, double HH, double bb, double aa){
    L = LL;
    H = HH;
    b = bb;
    a = aa;
  }

 void set(double LL, double HH, double bb, double aa){
    L = LL;
    H = HH;
    b = bb;
    a = aa;
  }

};

class CarOdoControlInput: public OdoControlInputInterface{
 public:
  double speed;
  double steer;
  double dt;
  double slip[3];

  CarOdoControlInput():speed(0),steer(0),dt(0),car(NULL){
    slip[0] = slip[1] = slip[2] = 0.0;
  }

  CarOdoControlInput(const CarMotionModelParameters *Car):
     speed(0),steer(0),dt(0),car(Car){
    slip[0] = slip[1] = slip[2] = 0.0;
 }

  CarOdoControlInput(double Speed, double Steer, double DT,
                     const CarMotionModelParameters *Car):
          speed(Speed),steer(Steer),dt(DT),car(Car){
      slip[0] = slip[1] = slip[2] = 0.0;
  }

  CarOdoControlInput(const CarOdoControlInput &other):
    speed(other.speed),steer(other.steer),dt(other.dt), car(other.car)
  {    slip[0] = slip[1] = slip[2] = 0.0; }

  CarOdoControlInput(const CarOdoControlInput *other):
    speed(other->speed),steer(other->steer),dt(other->dt),car(other->car)
  {
    slip[0] = other->slip[0];
    slip[1] = other->slip[1];
    slip[2] = other->slip[2];
  }
  
  OdoControlInputInterface* clone()const{return new CarOdoControlInput(this);}

  void reset(){steer = speed = dt = 0.0;}

  void advance(RobotPose *prev)const;
  void advance(RobotPose *prev, double dt)const;

  void set(double Speed, double Steer, double DT){
    speed = Speed;
    steer = Steer;
    dt = DT;
  }

  void setCar(const CarMotionModelParameters *Car){ car = Car;}

 private:
  const CarMotionModelParameters* car;

};

class CarMotionModel: public MotionModelInterface{
 private:
  CarMotionModelParameters car;
  CarOdoControlInput u;
  double t_curr;
  RobotPose odo_curr;

  bool movedD;
  bool movedR;

  GaussianRandomNumber rand_Speed;
  GaussianRandomNumber rand_Steer;
  Gaussian3dSampler    slippage;
  UniformRandomNumber  dice;

 public:

  CarMotionModel();
  CarMotionModel(const CarMotionModel* other);

  void newOdometry(const OdometryReadingInterface*);

  const OdoControlInputInterface* getControlInput()const{return &u;}
  OdoControlInputInterface* sampleControlInput()const;

  RobotPose getTruePose(double t)const{
    RobotPose robot(odo_curr);
    u.advance(&robot,t-t_curr);
    return robot;
  }
     
  MotionModelInterface* clone()const{return new CarMotionModel(this);}

  double getDT(double Time)const{return Time - t_curr;}

  double getRotVelocity()const{ return u.steer;}
  double getTransVelocity()const{ return u.speed;}

  bool hasMoved()const{ return movedD;}
  bool hasMovedD()const{ return movedD;}
  bool hasMovedR()const{ return movedR;}

  void setCarModel(double LL, double HH, double bb, double aa){
    car.set(LL,HH,bb,aa);
  }

};


//=====================================================
// Inlines
//=====================================================


#endif


