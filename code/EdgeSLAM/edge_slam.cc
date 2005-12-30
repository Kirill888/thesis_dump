#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "odoXR4000.h"
#include "simpleSLAM.h"
#include "edgeLandmark.h"
#include "edge_common.h"

FILE *f_man = NULL;
FILE *f_odo = NULL;
FILE *f_obs = NULL;
//Output
FILE *f_mat = NULL;

bool stop = false;

//To catch ^C
void onKillSignal(int signal){
  printf("Received kill signal: exiting.\n");
  stop = true;
}

const int SENSOR_ODO = 1;
const int SENSOR_OBS = 2;

ObservationStore obs0(100000, 10000);
int obsCount = 0;

MotionModel robot;
OdometryStore odoStore;

EdgeMappingProcess mapper;

MapBuilder DEFAULT_BUILDER;

SimpleSLAM Slam;

double T0 = -1;


bool odoEvent(FILE *f, double){
  double TimeStamp;
  double x,y,rot;

  if(fscanf(f,"%lf %lf %lf %lf", 
            &TimeStamp, &x, &y, &rot) != 4) return false;


  printf("ODO: %9.2fs/%9.2fs -- %9.3f %9.3f %9.2f deg\n"
         , TimeStamp, GetCurrentTime() -T0
         , x, y, angleDiffRad(rot,0)*RADTODEG);
  

  RobotPoseOdo odo(RobotPose(x,y,rot),TimeStamp);

  robot.newOdometry(&odo);
  Slam.odometry(&odo);

  return true;
}

#ifdef MAX_OBS
#undef MAX_OBS
#endif

const int MAX_OBS = 100;
int *obs_ind = new int[MAX_OBS];


bool obsEvent(FILE *f, double TimeStamp){

  static int last_frame = -1;

  int numObs = 0;
  int n = 0;
  int frame = 0;

  if(last_frame < 0){
    if(fscanf(f,"%d", &frame) != 1) return false;

    last_frame = frame;
  }else{
    frame = last_frame;
  }

  obs0.startScan();

  while(frame == last_frame){
    EdgeObservation *obs = readObs(f, mapper.cam);
    if(obs == NULL) return false;

    numObs += 1;

    obs_ind[n] = obs0.add(obs,frame);
    n += 1;

    if(fscanf(f,"%d", &frame) != 1){
      frame = last_frame + 1;
    }
  }

  last_frame = frame;

  if(n <= 0) return false;

  RobotPose p = robot.getTruePose(TimeStamp);
  odoStore.add(p);

  printf("OBS: %d (%10.2f sec) [# %d/%d]", frame, TimeStamp, n, numObs);


  if(!Slam.hasMoved()){
    Slam.storeOdo(TimeStamp, obs0.getLastScanId());
    printf("[no move]\n");
    return true;
  }

  printf("\n");

  if(n > 0) Slam.observation(TimeStamp, obs0, obs_ind, n);
  else Slam.storeOdo(TimeStamp, obs0.getLastScanId());

  return true;
}

void run_simulation(){
  double Time;
  int id;
  bool ok = true;

  while(!stop && ok && fscanf(f_man, "%lf %d", &Time, &id) == 2){
    switch(id){
    case SENSOR_ODO:
      ok = odoEvent(f_odo,Time);
      break;
    case SENSOR_OBS:
      ok = obsEvent(f_obs,Time);
      break;
    }
  }
}
void run_ekf_test(int argc, char** argv);
void run_ekf_test0(int argc, char** argv);

int main(int argc, char** argv){

#if 0
  if(1){  
    mappingProcess             = &mapper;
    run_ekf_test(argc,argv);
    return 1;
  }
#endif

  char *man_file = NULL;
  char *odo_file = NULL;
  char *obs_file = NULL;

  char *map_out = NULL;
  char *movie_mask = NULL;

  int numP = 100;
  int seed = time(NULL);

  if(argc > 1) man_file   = argv[1];
  if(argc > 2) odo_file   = argv[2];  
  if(argc > 3) obs_file   = argv[3];
  if(argc > 4) map_out    = argv[4];
  if(argc > 5) numP       = atoi(argv[5]);
  if(argc > 6) seed       = atoi(argv[6]);
  if(argc > 7) movie_mask = argv[7];

  if(odo_file == NULL || obs_file == NULL || man_file == NULL 
     || map_out == NULL){
    fprintf(stderr,"Usage: %s manager odo obs out [numP seed]\n"
            ,argv[0]);
    return 1;
  }

  f_man = fopen(man_file,"r");
  if(f_man == NULL){
    fprintf(stderr,"Failed to open: %s\n", man_file);
    return 2;
  }

  f_odo = fopen(odo_file,"r");
  if(f_odo == NULL){
    fprintf(stderr,"Failed to open: %s\n", odo_file);
    return 2;
  }

  f_obs = fopen(obs_file,"r");
  if(f_obs == NULL){
    fprintf(stderr,"Failed to open: %s\n", obs_file);
    return 2;
  }

  char mat_file[1024];
  char map_file[1024];
  sprintf(mat_file,"%s_dat.m",map_out);
  sprintf(map_file,"%s_map.m",map_out);

  f_mat = fopen(mat_file,"w");
  if(f_mat == NULL){
    fprintf(stderr,"Failed to open: %s\n", mat_file);
    return 2;
  }

  printf("Starting SLAM:\n"
         "---manager     file: %s\n"
         "---odometry    file: %s\n"
         "---observation file: %s\n"
         "---Num. Particles  : %d\n"
         "---Seed            : %d\n"
         "---Output          : %s\n"
         ,man_file, odo_file, obs_file, numP, seed, map_out
	 );

  fprintf(f_mat, "%%Starting SLAM:\n"
         "%%---manager     file: %s\n"
         "%%---odometry    file: %s\n"
         "%%---observation file: %s\n"
         "%%---Num. Particles  : %d\n"
         "%%---Seed            : %d\n"
         "%%---Output          : %s\n"
         ,man_file, odo_file, obs_file, numP, seed, map_out
	 );
  fflush(f_mat);

  signal(SIGINT, onKillSignal);

//   robot.setP1(0,0,POW2(0.1));
//   robot.setP1(1,1,0.15/3);
//   robot.setP1(1,2,POW2(deg2rad(1)/3));
//   robot.setP1(2,2,0.25/3);

  robot.setP1(0,0,POW2(0.1));             //time    =f(t)
  robot.setP1(1,1,POW2(0.25));            //rotaion =f(rot)
  robot.setP1(1,2,POW2(deg2rad(1)));      //        =f(dist)
  robot.setP1(2,2,POW2(0.5));             //distance=f(dist)
  robot.setP1(2,1,POW2(0.01/deg2rad(1))); //distance=f(rot)

  //Set Mapping Process parameters
  mappingProcess             = &mapper;
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;
  mappingProcess->setRobot2SensorFunction(robot2sensor);

  mapper.cam = initStereoPair();

  Slam.NP_MIN = numP;
  Slam.NP_MAX = 10*numP;
  Slam.RESAMPLE_STEP = 15;

  Slam.setup(&robot, &DEFAULT_BUILDER);
  Slam.setSeed(seed);

  T0 = GetCurrentTime();
  Slam.start();
  run_simulation();

  printf("Storing Map\n");
  Slam.finalise();

  FILE* f = fopen(map_file,"w");
  if(f != NULL){
    Slam.getMap()->store(f);
    dumpDataAssociations(f, Slam.getMap()->getMap());

    Slam.getPath()->matlabDump(f,"odo");
    Slam.dumpOdo2Scan(f,"odo2scan");

    fclose(f);
  }else{
    printf("Failed to open: %s\n", map_file);
    Slam.getMap()->store(stdout);
    Slam.dumpOdo2Scan(stdout,"odo2scan");
  }

  printf("Storing Observations:\n");
  obs0.matlabDump(f_mat,"obs");

  printf("Storing Odometry:\n");
  odoStore.getReverse().matlabDump(f_mat, "odo");

  return 0;
}


void run_ekf_test(int argc, char** argv){
  if(argc < 2){
    fprintf(stderr,"ekf_test: file\n");
    return;
  }
  FILE *f = fopen(argv[1],"r");

  if(f == NULL){ 
    fprintf(stderr,"Can't open: %s\n", argv[1]);
    return;
  }

  int n, ind;
  double u1,u2,x,y,a;
  RobotPose robot;
  EdgeObservation *o;
  EdgeLandmark *landmark = NULL;

  const double cov0[2][2] = {{9  ,  0 },
                            { 0  , 25}};
  double cov[2][2];

  mapper.cam = initStereoPair();
  n = 0;

  while(fscanf(f,"%lf %lf %lf %lf %lf", &u1,&u2,&x,&y,&a) == 5){
    robot.set(x,y,a);

    printf("%% Obs: %g %g\n", u1,u2);

    memcpy(cov,cov0,4*sizeof(double));
    normaliseObs(&u1,&u2,cov, mapper.cam);

    printf("%% Obs_norm: %g %g %g %g %g %g\n", u1,u2
           ,cov[0][0],cov[0][1], cov[1][0],cov[1][1]);

    o = new EdgeObservation(u1,u2,1,640,1,cov);

    ind = obs0.add(o,n);

    if( n == 0){
      //First one -- init landmark
      double tmp;
      landmark = (EdgeLandmark*)mapper.makeNewEntry(&tmp, obs0, ind, &robot);

      double u[2];
      double Szz[2][2];
      double H[2][2];

      landmark->projectToImagePlane(u,Szz,H, &robot, mapper.cam);
      printf("%%u = [%.8e %.8e]\n", u[0],u[1]);
      printf("%%H = [%.8e,%.8e; %.8e,%.8e]\n"
	     , H[0][0],H[0][1],H[1][0],H[1][1]);

      printf("%%Szz = [%.8e,%.8e; %.8e,%.8e]\n"
	     , Szz[0][0],Szz[0][1],Szz[1][0],Szz[1][1]);

      printf("%%xx = [%.8e %.8e, %.8e,%.8e %.8e,%.8e;\n"
             "%%      %.8e %.8e, %.8e,%.8e %.8e,%.8e];\n"
	     ,u[0],u[1], Szz[0][0],Szz[0][1],Szz[1][0],Szz[1][1]
	     ,u1,u2, o->cov[0][0],o->cov[0][1],o->cov[1][0],o->cov[1][1]
            );
    }else{
      //Update landmark
      landmark->update(obs0,ind,&robot);
    }

    printf("%g %g %g %g %g %g\n",
	   landmark->point.x,landmark->point.y
	   ,landmark->point.cov[0][0]
	   ,landmark->point.cov[0][1]
	   ,landmark->point.cov[1][0]
	   ,landmark->point.cov[1][1]
	   );

    n += 1;
  }

  fclose(f);
}

void run_ekf_test0(int argc, char** argv){
  double l1,l2;
  double x,y,sxx,sxy,syy;


  const double sigma1 =  3*3;
  const double sigma2 =  5*5;
  const double F1     =  737.5912;
  const double F2     =  711.0105;
  const double CX1    =  314.2927;
  const double CX2    =  333.1539;
  const double x2     = - 48.1795; 
  const double y2     = -295.8511;

  if(argc < 3){
    printf("ekf_test: l1 l2\n");
    return;
  }

  l1 = atof(argv[1]);
  l2 = atof(argv[2]);

  printf("Convert l1,l2 -> x,y\n");
  printf("Input: %g,%g (%g,%g)\n",l1,l2,sigma1,sigma2);

  double z1 = CX1 - l1;
  double z2 = CX2 - l2;

  double den = 1.0/(F1*z2 - F2*z1);
  double den2 = den*den;

  double aux1 = z2*x2 - F2*y2;

  x = F1*aux1*den;
  y = z1*aux1*den;


  double jb1,jb2,jb3,jb4;

  double aux2 = F1*y2 - x2*z1;

  jb1 = F1*F2*aux1*den2;  jb2 = F1*F2*aux2*den2;
  jb3 = F1*z2*aux1*den2;  jb4 = z1*F2*aux2*den2;

  printf("jb = [%g,%g;%g,%g]\n",jb1,jb2,jb3,jb4);

  sxx = jb1*jb1*sigma1 + jb2*jb2*sigma2;
  sxy = jb1*jb3*sigma1 + jb2*jb4*sigma2;
  syy = jb3*jb3*sigma1 + jb4*jb4*sigma2;

  printf("%%Result\n xy = [%g,%g,%g,%g,%g,%g]\n",
	 x,y,sxx,sxy,sxy,syy);

  //Test makeNewEntry
  mapper.cam = initStereoPair();

  double cov[2][2] = {{sigma1,   0   },
                      {  0   , sigma2}};
  double tmp;
  RobotPose pose(0,0,1000.87687);
  EdgeObservation *edge = new EdgeObservation(l1,l2,1,640, 1, cov);
  obs0.startScan();
  int ind = obs0.add(edge,0);
  EdgeLandmark *lndm = (EdgeLandmark*) 
                       mapper.makeNewEntry(&tmp, obs0, ind, &pose);

  printf("%%Result2\n xy = [%g,%g,%g,%g,%g,%g]\n",
	 lndm->point.x,lndm->point.y
        ,lndm->point.cov[0][0]
        ,lndm->point.cov[0][1]
        ,lndm->point.cov[1][0]
        ,lndm->point.cov[1][1]
        );


  printf("Projecting Back\n");

  double l1_ = F1*y/x;
  double l2_ = F2*(y-y2)/(x-x2);

  jb1 = -l1_ / x;
  jb2 =  F1  / x;
  jb3 = -l2_ / (x - x2);
  jb4 =  F2  / (x - x2);

  double s11, s12, s22;

  s11 = jb1*jb1*sxx + 2*jb1*jb2*sxy +               jb2*jb2*syy;
  s12 = jb1*jb3*sxx +   jb1*jb4*sxy + jb2*jb3*sxy + jb2*jb4*syy;
  s22 = jb3*jb3*sxx + 2*jb3*jb4*sxy +               jb4*jb4*syy;

  l1_ = CX1 - l1_;
  l2_ = CX2 - l2_;

  printf(" %g (%g)  %g (%g) sigma: %g,%g,%g\n"
	 ,l1_, (l1_ - l1)
	 ,l2_, (l2_ - l2)
	 ,s11, s12, s22);

  printf("Project Back2\n");
  double u[2];
  double Szz[2][2];
  double H[2][2];

  lndm->projectToImagePlane(u,Szz,H,&pose,mapper.cam);
  printf("%g (%g) %g (%g) sigma:\n uu = [%g, %g,    %g,%g,%g,%g]\n"
	 ,u[0],(u[0] - l1)
	 ,u[1],(u[1] - l2)
	 ,u[0],u[1]
	 ,Szz[0][0], Szz[0][1]
	 ,Szz[1][0], Szz[1][1]
  );

}


