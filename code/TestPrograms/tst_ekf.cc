#include "kalman2.h"
#include "landmark2d.h"

int main(){

  double Srr = 1.11111111e-03; double Saa = 8.46159499e-04;
  RobotPose robot; 
  double range, bearing;

  double Szz[2][2] = {{Srr, 0.0},{0.0, Saa}};

  range=+3.1001854e+00; bearing=-1.1203607e+00;
  robot.set(+0.0000000e+00, +0.0000000e+00, +7.8539816e-01);
  //Init observation
  Landmark2dObs obs0(range, Srr, bearing, Saa);
  Gaussian2d x(obs0.toCartesian(&robot));
  range=+3.0796408e+00; bearing=-1.1346697e+00;
  robot.set(+1.0000000e-01, +9.9334665e-02, +7.7533146e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+3.0368746e+00; bearing=-1.1557413e+00;
  robot.set(+2.0000000e-01, +1.9470917e-01, +7.4432991e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.9645825e+00; bearing=-1.1352712e+00;
  robot.set(+3.0000000e-01, +2.8232124e-01, +6.8999971e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.8854660e+00; bearing=-1.0722390e+00;
  robot.set(+4.0000000e-01, +3.5867805e-01, +6.0851228e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.8313813e+00; bearing=-1.0313919e+00;
  robot.set(+5.0000000e-01, +4.2073549e-01, +4.9536729e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.8074619e+00; bearing=-8.8877782e-01;
  robot.set(+6.0000000e-01, +4.6601954e-01, +3.4764126e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.7393842e+00; bearing=-6.9347643e-01;
  robot.set(+7.0000000e-01, +4.9272486e-01, +1.6835822e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.7166510e+00; bearing=-5.5093923e-01;
  robot.set(+8.0000000e-01, +4.9978680e-01, -2.9191228e-02);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.5761797e+00; bearing=-4.1619711e-01;
  robot.set(+9.0000000e-01, +4.8692382e-01, -2.2340944e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.4576775e+00; bearing=-2.7537993e-01;
  robot.set(+1.0000000e+00, +4.5464871e-01, -3.9434811e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.3501229e+00; bearing=-1.1825103e-01;
  robot.set(+1.1000000e+00, +4.0424820e-01, -5.3192153e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.3010370e+00; bearing=+1.8169846e-02;
  robot.set(+1.2000000e+00, +3.3773159e-01, -6.3538415e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+2.1168617e+00; bearing=+6.2988044e-02;
  robot.set(+1.3000000e+00, +2.5775069e-01, -7.0847977e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.9834279e+00; bearing=+1.4757487e-01;
  robot.set(+1.4000000e+00, +1.6749408e-01, -7.5565871e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.9174124e+00; bearing=+1.7001208e-01;
  robot.set(+1.5000000e+00, +7.0560004e-02, -7.8036929e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.7324428e+00; bearing=+1.9799916e-01;
  robot.set(+1.6000000e+00, -2.9187072e-02, -7.8454482e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.5839900e+00; bearing=+2.0669568e-01;
  robot.set(+1.7000000e+00, -1.2777055e-01, -7.6851862e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.4725157e+00; bearing=+1.5666899e-01;
  robot.set(+1.8000000e+00, -2.2126022e-01, -7.3102128e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.2901957e+00; bearing=+1.1292032e-01;
  robot.set(+1.9000000e+00, -3.0592895e-01, -6.6920913e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.2106716e+00; bearing=+5.8152082e-02;
  robot.set(+2.0000000e+00, -3.7840125e-01, -5.7893238e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.0441412e+00; bearing=-7.7604526e-02;
  robot.set(+2.1000000e+00, -4.3578789e-01, -4.5582595e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+9.5070566e-01; bearing=-2.9165813e-01;
  robot.set(+2.2000000e+00, -4.7580104e-01, -2.9817055e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+8.8011942e-01; bearing=-4.8442728e-01;
  robot.set(+2.3000000e+00, -4.9684550e-01, -1.1168582e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+7.6324561e-01; bearing=-8.2749094e-01;
  robot.set(+2.4000000e+00, -4.9808230e-01, +8.7276704e-02);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+7.2009717e-01; bearing=-1.0658346e+00;
  robot.set(+2.5000000e+00, -4.7946214e-01, +2.7640141e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+6.8956443e-01; bearing=-1.3413419e+00;
  robot.set(+2.6000000e+00, -4.4172733e-01, +4.3814525e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+6.7200183e-01; bearing=-1.6584741e+00;
  robot.set(+2.7000000e+00, -3.8638224e-01, +5.6553913e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+6.8683211e-01; bearing=-1.9828792e+00;
  robot.set(+2.8000000e+00, -3.1563332e-01, +6.5966351e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+7.9640632e-01; bearing=-2.2065333e+00;
  robot.set(+2.9000000e+00, -2.3230109e-01, +7.2475699e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+8.1694017e-01; bearing=-2.3534772e+00;
  robot.set(+3.0000000e+00, -1.3970775e-01, +7.6508144e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+9.1403909e-01; bearing=-2.4421736e+00;
  robot.set(+3.1000000e+00, -4.1544701e-02, +7.8366622e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.0677623e+00; bearing=-2.5771910e+00;
  robot.set(+3.2000000e+00, +5.8274602e-02, +7.8197899e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.1644566e+00; bearing=-2.6133405e+00;
  robot.set(+3.3000000e+00, +1.5577068e-01, +7.5988500e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.3072502e+00; bearing=-2.6670699e+00;
  robot.set(+3.4000000e+00, +2.4705668e-01, +7.1564807e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.3963248e+00; bearing=-2.6172320e+00;
  robot.set(+3.5000000e+00, +3.2849330e-01, +6.4599388e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.5312333e+00; bearing=-2.5056351e+00;
  robot.set(+3.6000000e+00, +3.9683393e-01, +5.4653755e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.6144484e+00; bearing=-2.4369703e+00;
  robot.set(+3.7000000e+00, +4.4935405e-01, +4.1328917e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.5980311e+00; bearing=-2.3105791e+00;
  robot.set(+3.8000000e+00, +4.8395984e-01, +2.4616405e-01);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.7194507e+00; bearing=-2.1730557e+00;
  robot.set(+3.9000000e+00, +4.9927167e-01, +5.3903154e-02);
  ekf_update(&x,range,bearing,Szz,&robot);
  range=+1.7874316e+00; bearing=-2.0496461e+00;
  robot.set(+4.0000000e+00, +4.9467912e-01, -1.4448612e-01);
  ekf_update(&x,range,bearing,Szz,&robot);



  printf("x = [%.8e, %.8e]; Sxx = [%.8e, %.8e; %.8e %.8e];\n",
         x.x, x.y
	 ,x.cov[0][0]
	 ,x.cov[0][1]
	 ,x.cov[1][0]
	 ,x.cov[1][1]);

  return 1;
}
