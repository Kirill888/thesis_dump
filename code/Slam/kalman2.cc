#include "kalman2.h"
#include "matrix2.h"
#include "matrix3.h"
#include "util.h"


void kalman_update(Gaussian2d *x, const Gaussian2d *o){
     
  double I[2][2] = {{1.0, 0.0} ,
		    {0.0, 1.0}};
  double K[2][2];
  double aux1[2][2];

  //Compute Kalman gain
  // K = P*inv(P + R)
  matrix2_add(aux1, x->cov, o->cov);
  matrix2_inverse(aux1, aux1);
  matrix2_multiply(K, x->cov, aux1);

  //Update landmark location
  // x' = x + K*(z - x)
  double dx = o->x - x->x;
  double dy = o->y - x->y;

  x->x = x->x + K[0][0]*dx + K[0][1]*dy;
  x->y = x->y + K[1][0]*dx + K[1][1]*dy;

  //Update landmark covariance
  // P' = (I - K)*P
  matrix2_subtract(aux1,I,K);
  matrix2_multiply(x->cov, aux1, x->cov);
}

void kalman_update(Gaussian2d *x, const Gaussian2d *obs, int nobs){
  int i;

  for(i = 0; i < nobs; ++i){
    kalman_update(x,obs + i);
  }

}

#include "matrix.h"

void kalman_update(RobotPoseCov *x, const RobotPoseCov *obs){
  double K[3][3];
  double aux[3][3];

  //Compute Kalman Gain
  // K = P*inv(P + R);
  matrix3_add(aux, x->cov, obs->cov);
  matrix3_inverse(aux,aux);
  matrix3_multiply(K,x->cov,aux);

  //Update state: x = x + K*(z - x);
  double dz[3];
  double dx[3];

  dz[0] = obs->x   - x->x;
  dz[1] = obs->y   - x->y;
  dz[2] = angleDiffRad(obs->rot,x->rot);

  matrix3_multiply(dx,K,dz);
  x->x  += dx[0];
  x->y  += dx[1];
  x->rot = angleDiffRad(x->rot, -dx[2]);

  //Update covariance
  // P = (I - K)*P;

  //  printf("K\n");   matrix_print(K,3,3);
  matrix3_subtract(aux, I_3x3, K);
  //  printf("I-K\n"); matrix_print(aux, 3, 3);

  //  printf("P\n"); matrix_print(x->cov, 3, 3);
  matrix3_multiply(x->cov, aux, x->cov);
  //  printf("P'\n"); matrix_print(x->cov, 3, 3);

}


void ekf_update(Gaussian2d *x, double range, double bearing, 
                const double Szz[2][2], const RobotPose *robot,
		const RobotPose *sensor){
  //1st translate
  RobotPose sens_xy(*sensor);
  sens_xy.translateMe(*robot);

  printf("EKF2:");
  print_robot("  robot",robot);
  print_robot("  sensor0",sensor);
  print_robot("  sensor1",&sens_xy); 

  ekf_update(x,range,bearing,Szz,&sens_xy);
}

//Incorporate range/bearing observation
void ekf_update(Gaussian2d *x, double range, double bearing, 
                const double Szz[2][2], const RobotPose *robot){

  double dx,dy, r, r2;
  double I[2][2] = {{1.0, 0.0} ,
		    {0.0, 1.0}};

  dx = x->x - robot->x;
  dy = x->y - robot->y;

  r2 = dx*dx + dy*dy;
  r = sqrt(r2);

  double H[2][2];
  H[0][0] =  dx/r;  H[0][1] =  dy/r;
  H[1][0] = -dy/r2; H[1][1] =  dx/r2;

  //Compute Kalman gain
  //K = Sxx*H'*inv ( H*Sxx*H' + Szz )
  double K[2][2];
  double Ht[2][2];
  //Sxx_ = H*Sxx*H'
  double Sxx_[2][2]; //Covariance of the map element in polar coords
  double aux1[2][2];

  matrix2_transpose(Ht,H);

  matrix2_multiply(aux1,H, x->cov);
  matrix2_multiply(Sxx_,aux1,Ht);
  matrix2_add(aux1,Sxx_,Szz);
  matrix2_inverse(aux1,aux1);
  matrix2_multiply(aux1,Ht,aux1);
  matrix2_multiply(K,x->cov,aux1);

  double dr = range - r;
  double a_expect = atan2(dy,dx) - robot->rot;
  double da = angleDiffRad(bearing, a_expect);

#if 0
  printf("EKF: da = %+.3f; dr = %.3f (%.3f%+.3f))\n"
         ,da*RADTODEG,dr, bearing*RADTODEG, -a_expect*RADTODEG); 
  if(fabs(da) > 5.0*DEGTORAD){
    printf("EKF: Warning possible angle error!!!\n");
  }
#endif

  //Update the estimate
  //  x = x + K*dz
  x->x += (K[0][0]*dr + K[0][1]*da);
  x->y += (K[1][0]*dr + K[1][1]*da);

  //Update Covariance
  //Sxx = (eye(2) - K*H)*Sxx        % Update covariance
  matrix2_multiply(aux1,K,H);
  matrix2_subtract(aux1, I, aux1);
  matrix2_multiply(x->cov, aux1, x->cov);
}
