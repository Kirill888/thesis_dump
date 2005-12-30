#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "multiSLAM.h"
#include "simpleSLAM.h"
#include "tree.h"
#include "odoCar.h"

//Input
FILE *f_man = NULL;
FILE *f_odo = NULL;
FILE *f_obs = NULL;
//Output
FILE *f_mat = NULL;

bool stop = false; //Control-C exit flag.

//To catch ^C
void onKillSignal(int signal){
  printf("Received kill signal: exiting.\n");
  stop = true;
}

void robot2sensor(RobotPose* sensor, const RobotPose *robot){
  double a = 3.0;
  double b = 0.0;

  double ca = cos(robot->rot);
  double sa = sin(robot->rot);

  double x   = robot->x + a*ca - b*sa;
  double y   = robot->y + a*sa + b*sa;
  double rot = robot->rot;

  sensor->set(x,y,rot);
}

const int SENSOR_ODO = 1;
const int SENSOR_OBS = 2;

#ifndef MAX_OBS
const int MAX_OBS    = 100;   //CHNG: adjust to be sure
#endif

ObservationStore obs0(10000, 1000);    //Storage for observations
int obsCount = 0;                      //Scan/frame/observation count
int obs_ind[MAX_OBS];

OdometryStore odoStore;                 //This stores raw path.

MapBuilder DEFAULT_BUILDER;
SimpleSLAM Slam;

double T0; //Starting CPU time

CarMotionModel      robot;
TreeLandmarkFactory factory;
TreeSensorRange     sensorRange(0,90,-M_PI/2, M_PI/2);


bool odoEvent(FILE *f){
  double TimeStamp;
  double speed;
  double steer;

  if(fscanf(f,"%lf %lf %lf", &TimeStamp, &speed, &steer) != 3) return false;

  printf("ODO: %9.2fs (%9.2fs) %9.3f m/s %9.2f deg\n"
	 , TimeStamp, GetCurrentTime() - T0
         , speed, steer*RADTODEG);
  

  CarOdoReading odo(TimeStamp, speed, steer);


  robot.newOdometry(&odo);
  Slam.odometry(&odo);

  return true;
}

ObservationInterface* readObs(FILE *f){
  double r, a, d;
  double sr, sa, sd;

  if(fscanf(f,"%lf %lf %lf",&r, &a, &d) != 3) return NULL;

  sr = (0.05*r + 0.5)/3.0;
  sa = (5.0*DEGTORAD)/3.0;
  sd = 0.3/2.0;

  return new TreeObservation(r, sr*sr,a , sa*sa, d, sd*sd);
}

bool obsEvent(FILE *f){
  double TimeStamp;
  int numObs;
  int i;

  if(fscanf(f,"%lf %d", &TimeStamp, &numObs ) != 2) return false;

  for(i = 0; i < numObs; ++i){
    if(i != 0) fscanf(f,"%*f %*d "); //skip TimeStamp, numObs

    ObservationInterface * obs = readObs(f);
    if(obs == NULL) return false;

    obs_ind[i] = obs0.add(obs,obsCount);
  }

  //Do not change, or change with care
  obsCount += 1;

  RobotPose p = robot.getTruePose(TimeStamp);
  odoStore.add(p);

  printf("OBS: %d (%10.2f sec) [# %d]", obsCount, TimeStamp, numObs);

  if(!Slam.hasMoved()){
    Slam.storeOdo(TimeStamp, obs0.getLastScanId());
    printf("[no move]\n");
    return true;
  }

  printf("\n");

  if(numObs > 0) Slam.observation(TimeStamp, obs0, obs_ind, numObs);
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
      ok = odoEvent(f_odo);
      break;
    case SENSOR_OBS:
      ok = obsEvent(f_obs);
      break;
    }
  }
}

int main(int argc, char** argv){
  char *man_file = NULL;
  char *odo_file = NULL;
  char *obs_file = NULL;

  char *map_out = NULL;
  int numP = 100;


  if(argc > 1) man_file   = argv[1];
  if(argc > 2) odo_file   = argv[2];  
  if(argc > 3) obs_file   = argv[3];
  if(argc > 4) map_out    = argv[4];
  if(argc > 5) numP       = atoi(argv[5]);

  if(odo_file == NULL || obs_file == NULL || man_file == NULL 
     || map_out == NULL){
    fprintf(stderr,"Usage: %s manager odo obs out [numP]\n"
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
         "---Output          : %s\n"
         ,man_file, odo_file, obs_file, numP, map_out
	 );

  signal(SIGINT, onKillSignal);

  Slam.NP_MIN = numP;
  Slam.NP_MAX = 10*numP;

  DEFAULT_BUILDER.setFactory(&factory);
  DEFAULT_BUILDER.setSensorRange(&sensorRange);
 
  robot.setCarModel(3,0,0,3);

  Slam.setup(&robot, &DEFAULT_BUILDER);
  Slam.setSeed(time(NULL));

  T0 = GetCurrentTime();
  Slam.start();

  run_simulation();

  printf("Storing Maps\n");
  Slam.finalise();

  FILE* f = fopen(map_file,"w");
  if(f != NULL){
    Slam.getMap()->store(f);
    fclose(f);
  }else{
    printf("Failed to open: %s\n", map_file);
    Slam.getMap()->store(stdout);
  }

  printf("Storing Observations:\n");
  obs0.matlabDump(f_mat,"obs");

  printf("Storing Odometry:\n");
  odoStore.getReverse().matlabDump(f_mat, "odo");

  return 0;
}
