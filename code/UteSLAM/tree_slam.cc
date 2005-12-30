#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "odoCar.h"
#include "simpleSLAM.h"
#include "tree.h"

//Input
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

void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  double a = 3.78;
  double b = 0.5;

  double ca = cos(robot->rot);
  double sa = sin(robot->rot);

  double x   = robot->x + a*ca - b*sa;
  double y   = robot->y + a*sa + b*sa;
  double rot = robot->rot;

  sensor->set(x,y,rot);
}

const int SENSOR_ODO = 1;
const int SENSOR_LAS = 2;
const int SENSOR_GPS = 3;

double T0 = -1;
int obsCount = -1;

ObservationStore obs0(100000, 10000);
OdometryStore    odoStore;

CarMotionModel             robot;
MapBuilder                 DEFAULT_BUILDER;
// TreeLandmarkFactory        factory;
// RangeAndBearingSensorRange sensorRange(0, 82, -100*DEGTORAD, 100*DEGTORAD);
TreeMappingProcess  mapper;

SimpleSLAM Slam;

bool odoEvent(FILE *f){
  double TimeStamp;
  double speed;
  double steer;

  if(fscanf(f,"%lf %lf %lf", &TimeStamp, &speed, &steer) != 3) return false;

  speed = speed/(1 - tan(steer)*0.76/2.83);

  printf("ODO: %9.2fs (%9.2fs) %9.3f m/s %9.2f deg\n"
	 , TimeStamp, GetCurrentTime() - T0
         , speed, steer*RADTODEG);
  

  CarOdoReading odo(TimeStamp, speed, steer);

  robot.newOdometry(&odo);

  Slam.odometry(&odo);

  return true;
}


TreeObservation* readObs(FILE *f){
  double r, a, d;
  double sr, sa, sd;

  if(fscanf(f,"%lf %lf %lf",&r, &a, &d) != 3) return NULL;

  sr = (0.1*r + 0.5)/3;
  sa = 5.0*DEGTORAD/3.0;
  sd = 2.0/3;

  return new TreeObservation(r, sr*sr,a - M_PI/2, sa*sa, d, sd*sd);
}

#ifndef MAX_OBS
const int MAX_OBS = 361;
#endif

int *obs_ind = new int[MAX_OBS];

bool obsEvent(FILE *f){
  double TimeStamp;
  int numObs;
  int i;

  if(fscanf(f,"%lf %d", &TimeStamp, &numObs ) != 2) return false;

  obs0.startScan();

  for(i = 0; i < numObs; ++i){
    TreeObservation * obs = readObs(f);
    if(obs == NULL) return false;

    obs_ind[i] = obs0.add(obs,obsCount);
  }

  obsCount += 1;

  printf("OBS: %d (%10.2f sec) [# %d]", obsCount, TimeStamp, numObs);

  if(!Slam.hasMoved()){
    printf("[no move]\n");
    return true;
  }

  RobotPose p = robot.getTruePose(TimeStamp);
  odoStore.add(p);

  printf("\n");

  if(numObs > 0) Slam.observation(TimeStamp, obs0, obs_ind, numObs);


  return true;
}

void run_simulation(){
  double Time;
  int id;
  bool ok = true;

  while(!stop && ok && fscanf(f_man, "%lf %d", &Time, &id) == 2){
    switch(id){
    case SENSOR_ODO:
      ok = odoEvent(f_odo);
      break;
    case SENSOR_LAS:
      ok = obsEvent(f_obs);
      break;
    }
  }
}

int main(int argc, char** argv){
  //For simulation only
  //  robot.setCarModel(3,0,0,3);

  char *man_file = NULL;
  char *odo_file = NULL;
  char *obs_file = NULL;

  char *map_out = NULL;

  char map_file[1024];

  int numP = 100;
  int seed = time(NULL);

  if(argc > 1) man_file = argv[1];
  if(argc > 2) odo_file = argv[2];  
  if(argc > 3) obs_file = argv[3];
  if(argc > 4) map_out  = argv[4];
  if(argc > 5) numP  = atoi(argv[5]);
  if(argc > 6) seed  = atoi(argv[6]);

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
  sprintf(mat_file,"%s_mat.m",map_out);
  sprintf(map_file,"%s_map.m", map_out);

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

  signal(SIGINT, onKillSignal);

  Slam.NP_MIN        = numP;
  Slam.NP_MAX        = numP*5;
  Slam.RESAMPLE_STEP = 5;
  Slam.setSeed(seed);

  //Set Mapping Process parameters
  mappingProcess             = &mapper;
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;
  mappingProcess->setRobot2SensorFunction(robot2sensor);

  DEFAULT_BUILDER.setSensorRange(mappingProcess->sensorRange);

  Slam.setup(&robot, &DEFAULT_BUILDER);
  Slam.start();

  T0 = GetCurrentTime();

  run_simulation();

  printf("Storing Maps\n");
  Slam.finalise();

  FILE* f = fopen(map_file,"w");
  if(f != NULL){
    Slam.getMap()->store(f);
    Slam.getPath()->matlabDump(f,"odo");
    fclose(f);
  }else{
    printf("Failed to open: %s\n", map_file);
    Slam.getMap()->store(stdout);
    Slam.getPath()->matlabDump(stdout,"odo");
  }

  printf("Storing Observations:\n");
  obs0.matlabDump(f_mat,"obs");

  printf("Storing Odometry:\n");
  odoStore.getReverse().matlabDump(f_mat, "odo");

  return 0;
}

