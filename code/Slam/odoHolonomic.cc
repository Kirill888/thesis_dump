#ifndef DEPEND
#include <math.h>
#endif

#include "odoHolonomic.h"
#include "AngleTools.h"
#include "debug.h"
#include "matrix.h"
#include "matrix3.h"

//=======================================================
// ControlInput
//=======================================================

void OdoControlInput::advance(RobotPose *prev, double dt)const{
  double x, y, rot;

  double dir = prev->rot + dir_;

  x = prev->x + dt*Vt_*cos(dir);
  y = prev->y + dt*Vt_*sin(dir);
  rot = prev->rot + dt*Vr_;

  prev->set(x, y, rot);
}

//=======================================================
// Holonomic Motion Model
//=======================================================

HolonomicMotionModel::HolonomicMotionModel():
  t_curr(-1), movedD(false), movedR(false), Vt(0),dir(0),Vr(0)
  ,dt(-1),acc_t(0), acc_r(0){

  setParams(params);
}

HolonomicMotionModel::HolonomicMotionModel(const HolonomicMotionModel &other){
  odo_curr = other.odo_curr;
  t_curr   = other.t_curr;
  sampler  = other.sampler;
  movedD   = other.movedD;
  movedR   = other.movedR;
  u        = other.u;
  Vt       = other.Vt;
  dir      = other.dir;
  Vr       = other.Vr;
  dt       = other.dt;
  acc_t    = other.acc_t;
  acc_r    = other.acc_r;

  setParams(other.params);
}

void HolonomicMotionModel::setParams(const HolonomicNoiseParams& p){
  params = p;

  if(params.model_slippage){
    double cov[3][3] = ZEROS_3x3_INIT;
    cov[0][0] = POW2(params.slippage_magnitude_xy);
    cov[1][1] = POW2(params.slippage_magnitude_xy);
    cov[2][2] = POW2(params.slippage_magnitude_rotation);

    if(!slippage.set(0,0,0, cov)){
      ABORT("Failed to init Gaussian3dSampler!\n"
	    "COV = \n"
	    "%e %e %e\n"
	    "%e %e %e\n"
	    "%e %e %e\n"
	    ,cov[0][0], cov[0][1], cov[0][2]
	    ,cov[1][0], cov[1][1], cov[1][2]
	    ,cov[2][0], cov[2][1], cov[2][2]
	    );
    }
  }
}


OdoControlInputInterface* HolonomicMotionModel::sampleControlInput()const{
  double a,b,c;

  sampler.sample(&a,&b,&c);

  OdoControlInput* u = new OdoControlInput(Vt + a, dir + c, Vr + b, dt);

  double x,y,r;

  if(params.model_slippage && 
     dice.nextRandom() < params.slippage_frequency){

    slippage.sample(&x,&y,&r);
    u->setSlippage(x,y,r);
  }

  return u;
}

void HolonomicMotionModel::newOdometry(const OdometryReadingInterface *o){
  const RobotPoseOdo *odo = (const RobotPoseOdo*) o;

  if(t_curr < 0.0){
    odo_curr = odo->getPose();
    t_curr = odo->TimeStamp;
    dt = 0.0;
    movedD = movedR = false;
    acc_r = acc_t = 0.0;

  }else{
    double DT   = odo->TimeStamp - t_curr;
    //Sometimes we get the same odometry reading,
    //Simply ignore it.
    if(DT < 0.0001){
      return;
    }

    //Compute Control Input
    double dx, dy, rot, r, r2;
    dx  = odo->x   - odo_curr.x;
    dy  = odo->y   - odo_curr.y;
    rot = angleDiffRad(odo->rot , odo_curr.rot);

    r2 = dx*dx + dy*dy;
    r  = sqrt(r2);


    u    = OdoControlInput(r/DT,dir,rot/DT,DT);


    movedD = r > params.minMoveDistance;
    movedR = fabs(rot)  > params.minMoveRotation;
    //    printf("Dist = %e,  A = %e, DT = %e\n",dist,rot,DT);
   
    if(movedD || movedR){
      dt = DT;
      double Vt_now = r/dt;
      double Vr_now = rot/dt;

      acc_t = (Vt_now - Vt)/dt;
      acc_r = (Vr_now - Vr)/dt;
      Vt = Vt_now;
      Vr = Vr_now;

      if(movedD){
         dir  = angleDiffRad(atan2(dy,dx),odo_curr.rot);
      }else{ 
         dir = 0;
      }

      //Update current pose
      odo_curr = odo->getPose();
      odometer_.move(r);

      //Compute Sigmas
      double cov[3][3] = ZEROS_3x3_INIT;

      double dt2_inv = 1.0/(dt*dt);
      double dt4_inv = dt2_inv*dt2_inv;

      //Time
      double stt = POW2(dt*params.error_time_proportion) + 
                   POW2(params.error_time_constant);
      //Rotation
      double saa = POW2(params.error_rotation_proportion * rot) 
                 + POW2(params.error_rotation_constant   * dt)
	         + POW2(params.error_rotation_per_meter  * r);

      //Displacement
      double srr = POW2(params.error_translation_proportion * r) 
                 + POW2(params.error_translation_constant   * dt);

      // Direction of travel
      cov[2][2] = POW2(params.error_heading_angle_constant);

      //Vt,Vr
      cov[0][0] = srr*dt2_inv + r2*stt*dt4_inv;
      cov[1][1] = saa*dt2_inv + rot*rot*stt*dt4_inv;
      cov[0][1] = 
      cov[1][0] = r*rot*stt*dt4_inv;

      //Set to zero in declaration
      //cov[2][0] = cov[0][2] = cov[1][2] = cov[2][1] = 0.0;

      if( !sampler.set(0,0,0, cov) ){
	ABORT("Failed to init Gaussian3dSampler!\n"
	      "COV = \n"
              "%e %e %e\n"
              "%e %e %e\n"
              "%e %e %e\n"
	      ,cov[0][0], cov[0][1], cov[0][2]
	      ,cov[1][0], cov[1][1], cov[1][2]
	      ,cov[2][0], cov[2][1], cov[2][2]
             );
      }
    }

    t_curr   = odo->TimeStamp;
  }
}
