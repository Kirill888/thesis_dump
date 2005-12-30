#ifndef DEPEND
#include <math.h>
#endif

#include "odoXR4000.h"
#include "AngleTools.h"
#include "debug.h"
#include "matrix.h"


MotionModel::MotionModel(){
  t_curr = -1.0;
  Vt = dir = Vr = 0.0;
  dt = -1.0;
  movedD = movedR = false;
  acc_t = acc_r = 0.0;

  // Input:  I = [dt^2;rot^2;dist^2]
  // Output: O = [s_t^2;s_rot^2,s_dist^2,sigma_dir^2]
  //         O = P1*I + P2

  //sigma_t^2
  P1[0][0] = POW2(0.1); P1[0][1] = P1[0][2] = 0; 
  P2[0]    = 0;

  //sigma_rot^2
  P1[1][0] = POW2(deg2rad(0.5)/3); //time
  P1[1][1] = POW2(0.25);           //rotation
  P1[1][2] = POW2(deg2rad(1.0));   //distance
  P2[1]    = 0;                    //constant
  
  //sigma_dist^2
  P1[2][0] = POW2(0.00001);         //time
  P1[2][1] = POW2(0.01/deg2rad(1)); //rotation
  P1[2][2] = POW2(0.5);             //distance
  P2[2]    = 0;                     //constant

  //sigma_dir^2
  P1[3][0] = 0;                    //time
  P1[3][1] = POW2(0.1/3);          //rotation
  P1[3][2] = POW2(deg2rad(1)/3);   //distance
  P2[3]    = POW2(deg2rad(0.5)/3); //constant
}

MotionModel::MotionModel(const MotionModel &other){
  odo_curr = other.odo_curr;
  t_curr = other.t_curr;
  sampler = other.sampler;
  movedD = other.movedD;
  movedR = other.movedR;
  u = other.u;
  Vt = other.Vt;
  dir = other.dir;
  Vr = other.Vr;
  dt = other.dt;
  acc_t = other.acc_t;
  acc_r = other.acc_r;

  memcpy(P1,other.P1,sizeof(P1));
  memcpy(P2,other.P2,sizeof(P2));
}



OdoControlInputInterface* MotionModel::sampleControlInput()const{
  double a,b,c;

  sampler.sample(&a,&b,&c);

  OdoControlInput* u = new OdoControlInput(Vt + a, dir + c, Vr + b, dt);

  //  printf("%e %e %e %%noise\n",a,b,c);

  double x,y,r;

  if(dice.nextRandom() < 0.0001){
    const double MAX_XY = 0.01;
    const double MAX_RR = 0.5*DEGTORAD;

    x = (0.5 - dice.nextRandom())*2*MAX_XY;
    y = (0.5 - dice.nextRandom())*2*MAX_XY;
    r = (0.5 - dice.nextRandom())*2*MAX_RR;

    u->setSlippage(x,y,r);
  }else{
    slippage.sample(&x,&y,&r);
    u->setSlippage(x,y,r);
  }

  return u;
}

double const MotionModel::MIN_MOVE_DIST = 0.001;
double const MotionModel::MIN_MOVE_ROT  = 0.010*DEGTORAD;

void MotionModel::newOdometry(const OdometryReadingInterface *o){

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


    movedD = r > MIN_MOVE_DIST;
    movedR = fabs(rot)  > MIN_MOVE_ROT;
    //    printf("Dist = %e,  A = %e, DT = %e\n",dist,rot,DT);
   
    if(movedD || movedR){
      
      //      DEBUG_print("ODO  : dist = %6.3fmm, dir = %9.5fdeg,"
      //                  " rot = %8.5fdeg, dt = %5.3f\n"
      //                  ,dist*1000, dir*RADTODEG, rot*RADTODEG,dt);

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

      //Compute Bias for rotation
      double Vr_bias  = 0;
      double Dir_bias = 0;
#if 1
      if(fabs(Vr) < 1*DEGTORAD){
        Vr_bias  = -deg2rad(0.46)*Vt;
	Dir_bias = -deg2rad(0.46)*Vt;
      }
#endif

      //Update current pose
      odo_curr = odo->getPose();
      odometer_.move(r);

      //Compute Sigmas
      double cov[3][3];

      double dt2_inv = 1.0/(dt*dt);
      double dt4_inv = dt2_inv*dt2_inv;

      double in[3] = {dt*dt,rot*rot,r2};
      double sigmas[4];
      matrix_multiply(sigmas,P1,in,4,3,1);
      matrix_add(sigmas,sigmas,P2,4,1);
      double srr,saa,stt;
      stt = sigmas[0];
      saa = sigmas[1];
      srr = sigmas[2];

      cov[2][2] = sigmas[3];

      cov[0][0] = srr*dt2_inv + r2*stt*dt4_inv;
      cov[1][1] = saa*dt2_inv + rot*rot*stt*dt4_inv;
      cov[0][1] = 
      cov[1][0] = r*rot*stt*dt4_inv;

      cov[2][0] = cov[0][2] = cov[1][2] = cov[2][1] = 0.0;

      if( !sampler.set(0,Vr_bias, Dir_bias, cov) ){
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

      /*
      printf("ControlInput =  %g m/s, %g deg/s, %g deg\ncov = [...\n"
	     "%e %e %e\n"
	     "%e %e %e\n"
	     "%e %e %e\n];\n"
	     ,Vt,Vr*RADTODEG, dir*RADTODEG
	     ,cov[0][0], cov[0][1], cov[0][2]
	     ,cov[1][0], cov[1][1], cov[1][2]
	     ,cov[2][0], cov[2][1], cov[2][2]
             );
      */


      //Set slippage
      double sxx,syy;
      //5mm + 1cm/per 1deg of rotation
      sxx = POW2(0.005/3) + POW2(0.01*RADTODEG*rot/3); 
      syy = sxx;
      saa = POW2(0.1*DEGTORAD/3);

      if(fabs(acc_r) > 0.1*DEGTORAD){
	sxx += POW2(0.005/2);
	syy += POW2(0.005/2);
      }

      slippage.set(0,0,0,sxx,syy,saa);

      //printf("Acc: %g m/ss %g deg/ss\n", acc_t, acc_r*RADTODEG);

    }else{
      extern const double ZEROS_3x3[3][3];
      sampler.set(0,0,0, ZEROS_3x3);
      acc_t = acc_r = 0.0;
    }

    t_curr   = odo->TimeStamp;
  }

}

double LAS_OFFSET = 0.229;

RobotPose LASER_SENSOR_POSE(LAS_OFFSET,0.024,-0.0023);

