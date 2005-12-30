#ifndef __EDGE_COMMON_H__
#define __EDGE_COMMON_H__

#ifndef DEPEND
#include <ctype.h>
#endif

/* inline StereoPair* initStereoPair(){ */

/* #if 1   */
/*   //original calibration */
/*   CameraModel c1(737.5912, 757.7743, 314.2927, 312.8159, 640, 480); */
/*   CameraModel c2(711.0105, 737.6519, 333.1539, 306.8209, 640, 480); */
/* #else */
/*   //one camera at a time calibration (dodgy) */
/*   CameraModel c1(746.58, 765.68, 315.40, 275.76, 640, 480); */
/*   CameraModel c2(743.60, 762.20, 327.79, 264.68, 640, 480); */
/* #endif */

/*   //Pose of the right camera relative to the left one */
/*   RobotPose pose(-0.0481795,-0.2958511,-0.0394); */

/*   return new StereoPair(c1,c2,pose,-0.0211050453); */
/* } */

inline StereoPair* initStereoPair(){
  CameraModel c1(743.47, 381.71, 321.74, 133.33, 640, 240);
  CameraModel c2(742.53, 381.23, 316.79, 139.89, 640, 240);

  //Pose of the right camera relative to the left one
  RobotPose pose(-0.0035,-0.295,-8.7266e-04);

  StereoPair *p = new StereoPair(c1,c2,pose,0);

  p->pose2in1_cov[0][0] = POW2(0.02);
  p->pose2in1_cov[1][1] = POW2(0.02);
  p->pose2in1_cov[2][2] = POW2(deg2rad(2));

  return p;
}


const RobotPose SENSOR_POSE(0.095,0.155,deg2rad(-0.2));

//dodge: should be inline
void robot2sensor(RobotPose* sensor, const RobotPose *robot){
  sensor->set(SENSOR_POSE);
  sensor->translateMe(*robot);
}
const double SIGMA_CAM_CENTRE = POW2(7);
const double SIGMA_FOCAL      = POW2(7); 

inline void normaliseObs(double *u, double *suu, double pu
                  , const CameraModel &cam){

  double c = cam.centre_u;
  double f = cam.focal_u;
  double f_inv = 1.0/f;
  double f_inv2 = f_inv*f_inv;

  *u = (c-pu)*f_inv;

  *suu = f_inv2*(SIGMA_CAM_CENTRE + *suu + (*u)*f_inv*SIGMA_FOCAL);

}


inline void normaliseObs(double *u1, double *u2, double COV[2][2],
                         const StereoPair* cam){
  normaliseObs(u1,&(COV[0][0]), *u1, cam->camera1);
  normaliseObs(u2,&(COV[1][1]), *u2, cam->camera2);

  COV[0][1] = COV[1][0] = 0;
}

inline signed char hex2byte(const char hex[2]){
  unsigned char ub;
  signed char b;

  if(isdigit(hex[0])){
    ub = hex[0]&0xF;
  }else{
    ub = (hex[0]&0xF) + 9;
  }

  ub = ub << 4;

  if(isdigit(hex[1])){
    ub |= hex[1]&0xF;
  }else{
    ub |= (hex[1]&0xF) + 9;
  }

  b = (signed char)((int)ub - 128);

  return b;
}

inline int hex2byte(signed char* b, const char* hex, int maxSz){
  int i;
  int n = 0;

  for(i = 0; i < maxSz*2; i += 2){
    if(!isxdigit(hex[i]) || !isxdigit(hex[i+1])) return n;
    b[n] = hex2byte(hex+i);
    n += 1;
  }
  return n;
}

inline EdgeObservation* readObs(FILE *f, const StereoPair* cam){
  double u1,u2,h1,h2,corr;
  double COV[2][2];
  char s[1024]; //really all you need is 7*3*2+1

  if(fscanf(f,"%lf %lf %lf %lf %lf %s"
	    ,&u1, &u2, &corr
	    ,&h1, &h2, s
             ) != 6){ 
    return NULL;
  }


  COV[0][0] = POW2(7);
  COV[1][1] = POW2(9);

  normaliseObs(&u1,&u2,COV,cam);

/*   printf("OBS: sigma: %g %g\n"  */
/* 	 ,atan(sqrt(COV[0][0]))*RADTODEG  */
/* 	 ,atan(sqrt(COV[1][1]))*RADTODEG);  */

  EdgeObservation *o = new EdgeObservation(u1,u2,h1,h2,corr, COV);

  if(hex2byte(o->Template, s, EDGE_TEMPLATE_SIZE) != EDGE_TEMPLATE_SIZE){
    printf("Warning: bad hex template: %s\n", s);
  }

#if 0
  printf("OBS: template %s\n", s);
  printf("OBS: t:");
  for(int i = 0; i < EDGE_TEMPLATE_SIZE; ++i){
    printf(" %d",(int)o->Template[i]);
  }
  printf("\n");
#endif

  return o;
}



#endif
