#include "odoCar.h"
#include "AngleTools.h"

//math.h routines
extern double cos(double);
extern double sin(double);
extern double tan(double);


//----------------------------------------------------------------
//  CarOdoControlInput
//----------------------------------------------------------------

void CarOdoControlInput::advance(RobotPose *prev)const{
  advance(prev,dt);

  prev->x   += slip[0];
  prev->y   += slip[1];
  prev->rot += slip[2];
}

void CarOdoControlInput::advance(RobotPose *prev, double dt)const{

  double ca = cos(prev->rot);
  double sa = sin(prev->rot);
  double TanSteer = tan(steer);

  double aux1 = dt*speed*TanSteer/car->L;

  double x,y,rot;


  x   = prev->x   + dt*speed*ca;
  y   = prev->y   + dt*speed*sa;
  rot = prev->rot + aux1;

  prev->set(x,y,rot);
}

//----------------------------------------------------------------
//  CarMotionModel
//----------------------------------------------------------------
CarMotionModel::CarMotionModel(){
  t_curr = -1.0;
  movedD = movedR = false;

  u.setCar(&car);
  rand_Steer.set(0.0, 7.0*DEGTORAD/3.0);

  double sxx,syy,saa;

  sxx = POW2(0.1/3);
  syy = sxx;
  saa = POW2(0.3*DEGTORAD/3);
  slippage.set(0,0,0,sxx,syy,saa);
}

CarMotionModel::CarMotionModel(const CarMotionModel* other){
  car = other->car;

  u = other->u;
  u.setCar(&car);

  t_curr = other->t_curr;
  odo_curr = other->odo_curr;

  movedD = other->movedD;
  movedR = other->movedR;

  rand_Speed = other->rand_Speed;
  rand_Steer = other->rand_Steer;

  slippage = other->slippage;
}

const double MIN_SPEED = 0.001;
const double MIN_STEER = 0.1*RADTODEG;

void CarMotionModel::newOdometry(const OdometryReadingInterface* odo){
  const CarOdoReading * o = (const CarOdoReading*)odo;

  //If first time
  if(t_curr < 0.0){
    t_curr = o->TimeStamp;
    odo_curr.set(0,0,0);
    u.set(o->speed, o->steer, 0.0);

    movedD = movedR = false;

    return;
  }

  double dt = o->TimeStamp - t_curr;
  t_curr = o->TimeStamp;

#if 0
  printf("newOdometry: %.3f m/s %.2f deg %.1f ms\n"
        ,o->speed, o->steer*RADTODEG, dt*1000);
#endif

  u.set(o->speed, o->steer, dt);
  u.advance(&odo_curr);

  if(fabs(o->speed) > MIN_SPEED){
    movedD = true;
    movedR = fabs(o->steer) > MIN_STEER;

    double sigma_speed = (0.2*fabs(o->speed) + 0.1)/3.0;
    rand_Speed.set(0.0,sigma_speed);

    double dist = o->speed*dt;
    odometer_.move(dist);

  }else{
    movedD = false;
    movedR = false;
    rand_Speed.set(0.0,0.0);
  }

}


OdoControlInputInterface* CarMotionModel::sampleControlInput()const{
  double speed_err, steer_err;

  speed_err = rand_Speed.nextRandom();
  steer_err = rand_Steer.nextRandom();

  CarOdoControlInput *U = new CarOdoControlInput(u.speed + speed_err, 
						 u.steer + steer_err, 
						 u.dt, 
						 &car);

  if(dice.nextRandom() < 0.01){
    const double SLIP_XY = 0.1;
    const double SLIP_A  = 2*DEGTORAD;

    U->slip[0] = (0.5 - dice.nextRandom())*2*SLIP_XY;
    U->slip[1] = (0.5 - dice.nextRandom())*2*SLIP_XY;
    U->slip[2] = (0.5 - dice.nextRandom())*2*SLIP_A;
  }else{
    slippage.sample(U->slip, U->slip + 1, U->slip + 2);
  }

  return U;
}

