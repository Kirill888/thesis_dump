#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "odoXR4000.h"
#include "localmap.h"
#include "simpleSLAM.h"
#include "landmark2d.h"
#include "LaserCornerExtractor.h"
#include "mappingProcess.h"
#include "corner.h"

#define REPORT(format, args...) printf(format, ##args)

//Output
FILE *f_mat = NULL;

double T0  = -1.0;
double Ts0 = -1.0;
double TlastMove = -1.0;
int scanCount = 0;

bool stop = false; //Control-C exit flag.

//To catch ^C
void onKillSignal(int signal){
  printf("Received kill signal: exiting.\n");
  stop = true;
}

extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose *robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

#ifndef MAX_OBS
const int MAX_OBS    = 181; 
#endif

ObservationStore obs0(10000, 1000);    //Storage for observations
int obs_ind[MAX_OBS];

OdometryStore odoStore;                 //This stores raw path.

MapBuilder DEFAULT_BUILDER;
SimpleSLAM Slam;

MotionModel                 robot;       
Landmark2dMappingProcess    mapper;
CornerMappingProcess        mapperCorner;


void odoEvent(const OdoSource &odo){
  TLOG_ODO_START;

  if(T0 < 0){
    T0 = odo.TimeStamp();
    Ts0 = GetCurrentTime();
  }

  RobotPoseOdo o((RobotPose)odo, odo.TimeStamp());

  robot.newOdometry(&o);

  REPORT("ODO: %6.3f (%6.3f)\t[%7.3f m]\n"
	 , odo.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
        );


  if(scanCount > 6){
     Slam.odometry(&o);
     if(Slam.hasMoved()){ TlastMove = odo.TimeStamp(); }
  }

  TLOG_ODO_STOP;
}

typedef enum {Unknown = -1, Shiny=0, Corner} SensorType;

SensorType sensType = Shiny;

SensorType str2SensorType(const char* type){
  if(strcasecmp(type,"shiny")  == 0) return Shiny;
  if(strcasecmp(type,"corner") == 0) return Corner;

  return Unknown;
}

extern int extractShinyFeatures(ObservationInterface **, 
                                const LaserSource &las);

int extractFeatures(ObservationInterface **obs, 
		    const LaserSource &las){
  switch(sensType){
  case Shiny:
    return extractShinyFeatures(obs, las);
  case Corner:
    return extractCorners(obs, las);
  default:
    return 0;
  }

}

static ObservationInterface **obs = new ObservationInterface*[181];


void lasEvent(const LaserSource &las){
  //static int numF = 0;
  TLOG_LAS_START;
  int n;
  int i;

  if(T0 < 0){
    T0  = las.TimeStamp();
    Ts0 = GetCurrentTime();
  }

  int scanId = scanCount;
  scanCount += 1;

  RobotPose odo   = robot.getTruePose(las.TimeStamp());

  REPORT("LAS: %6.3f,(%6.3f) sec\t%6.2fm -- (%04d)"
	 , las.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
	 , scanCount
	 );

  if(!Slam.hasMoved()){
      REPORT("[no move]\n");
      TLOG_LAS_STOP;
      return;
  }


  double rotVel = robot.getRotVelocity();
  if(rotVel > 8*DEGTORAD){
    REPORT("[skipped]\n");
    Slam.storeOdo(las.TimeStamp(), scanId);
    TLOG_LAS_STOP;
    return;
  }else{
    //    REPORT("\n");
  }


  n = extractFeatures(obs,las);

  REPORT("[%02d]\n",n);

  if(n <= 0){
    Slam.storeOdo(las.TimeStamp(), scanId);
    TLOG_LAS_STOP;
    return;
  }

  int obs_ind[n];

  obs0.startScan();
  for(i = 0; i < n; ++i){
    obs_ind[i] = obs0.add(obs[i], scanId);
  }

  Slam.observation(las.TimeStamp(), obs0, obs_ind, n);


  TLOG_LAS_STOP;

  RobotPose odo_l;
  robot2sensor(&odo_l, &odo);
  odoStore.add(odo_l);

  print_memstat(stdout);
}


int main(int argc, char** argv){
  char *odo_file = NULL;
  char *obs_file = NULL;

  char *map_out = NULL;
  int numP = 100;
  int seed = time(NULL);
  char* type = "shiny";


  if(argc > 1) odo_file   = argv[1];  
  if(argc > 2) obs_file   = argv[2];
  if(argc > 3) map_out    = argv[3];
  if(argc > 4) numP       = atoi(argv[4]);
  if(argc > 5) type       = argv[5];
  if(argc > 6) seed       = atoi(argv[6]);

  extern int dros_debug_level;
  extern char* dros_program_name;

  dros_program_name = "shiny_slam";
  dros_debug_level = 1;

  if(odo_file == NULL || obs_file == NULL || map_out == NULL){
    fprintf(stderr,"Usage: %s odo obs out [numP] [type] [seed]\n"
            ,argv[0]);
    return 1;
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
  sensType = str2SensorType(type);

  if(sensType == Unknown){
    fprintf(stderr,"Bad Type: %s\n", type);
    return -1;
  }

  printf("Starting SLAM:\n"
         "---Type            : %s\n"
         "---odometry    file: %s\n"
         "---observation file: %s\n"
         "---Num. Particles  : %d\n"
         "---Seed            : %d\n"
         "---Output          : %s\n"
         , type, odo_file, obs_file, numP, seed, map_out
	 );



  signal(SIGINT, onKillSignal);

  Slam.NP_MIN = numP;
  Slam.NP_MAX = 10*numP;
  Slam.RESAMPLE_STEP = 3;

  if(sensType == Corner){
    mappingProcess  = &mapperCorner;
    mapperCorner.useNegativeInfo = false;

    initLaserCornerExtractor();
  }else{
    mappingProcess = &mapper;
    mapper.useNegativeInfo = false;
  }

  //Set Mapping Process parameters
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;


  DEFAULT_BUILDER.setSensorRange(mappingProcess->sensorRange);

  Slam.setup(&robot, &DEFAULT_BUILDER);
  Slam.setSeed(seed);

  Slam.start();


  FileOdoSource fodo(odo_file);
  FileLaserSource flas(obs_file,8.18);

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);

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
