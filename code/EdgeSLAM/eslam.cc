#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "odoXR4000.h"
#include "edgeLandmark.h"
#include "edge_common.h"
#include "multiSLAM.h"

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
MultiSLAM Slam;

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
    EdgeObservation *obs = readObs(f,mapper.cam);
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


int main(int argc, char** argv){

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
  sprintf(mat_file,"%s_mat.m",map_out);

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

  //Set Mapping Process parameters
  mapper.cam = initStereoPair();

  Slam.NUMP_PER_AGENT     = numP;
  Slam.CHECK_NOBS         = 5;
  Slam.CHECK_DIST         = 1.0;
  Slam.MAX_HYPE_PER_AGENT = 10;
  Slam.MAX_AGENTS         = 7;
  Slam.RESAMPLE_STEP      = 5;

  Slam.setSeed(seed);

  mappingProcess             = &mapper;
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;
  mappingProcess->setRobot2SensorFunction(robot2sensor);


  T0 = GetCurrentTime();

  Slam.start(&robot, &DEFAULT_BUILDER);
  run_simulation();

  printf("Storing Maps\n");
  Slam.finalise();

  MappingAgent *bestAgent = Slam.BestAgent();

  GlobalMap gmap(bestAgent->getMap());
  gmap.setReferenceFrame(0);
  gmap.store(map_out,Edge_storeLocalMap);

  char fname[1024];
  sprintf(fname,"%s_obs.txt",map_out);

  bestAgent->dumpPath(f_mat,"odo");
  Slam.dumpOdo2Scan(f_mat,"odo2scan");

  obs0.dumpToFile(fname);

  return 0;
}

