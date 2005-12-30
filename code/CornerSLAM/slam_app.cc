#ifndef DEPEND
#include <math.h>
#include <time.h>
#include <signal.h>
#endif

#include "TimeTools.h"
#include "DataSource.h"
#include "debug.h"
#include "geometry.h"
#include "odoModel.h"
#include "DataStore.h"
#include "memstat.h"


#include "odoXR4000.h"
#include "landmark2d.h"
#include "multiSLAM.h"
#include "LaserGridMapper.h"

#include "LaserCornerExtractor.h"
#include "corner.h"

extern MappingProcessInterface *mappingProcess;
extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

static ObservationInterface **obs = new ObservationInterface*[181];

#define M_log(format, args...) fprintf(mfile, format, ##args)
#define REPORT(format, args...) printf(format, ##args)

FILE *mfile;


void logTime(FILE* f, double ts);
FILE *timeLog=NULL;
double t_lasStart;

#define TLOG_ODO_START
#define TLOG_ODO_STOP

#define TLOG_LAS_START {t_lasStart=GetCurrentTime();}
#define TLOG_LAS_STOP  {logTime(timeLog,t_lasStart);}


bool stop = false;

MotionModel           robot;
MapBuilder            DEFAULT_BUILDER;
CornerMappingProcess  mapper;

const double HALF_BEAM_WIDTH = 0.5*DEGTORAD;
const double DROS_GridCellSize = 0.05;

MultiSLAM Slam;

OdometryStore odoStore;
ObservationStore obs0(5000,1000);

double T0  = -1.0;
double Ts0 = -1.0;
double TlastMove = -1.0;

int scanId = -1;

char *animMask = NULL;

bool dumpAnim = false;

static void storeAnimation(){
  int i;
  for(i = 0; i < Slam.numAgents(); ++i){
    char buf[256];
    sprintf(buf,"agent(%d).",i+1);
    Slam.agent(i)->dumpCurrentState(mfile,buf);
  }
  fprintf(mfile, "map_draw_callback(agent,%d,%d);\n"
          ,Slam.numAgents(), scanId);
}

static void dumpAnimation(const LaserSource& las){
  int i;
  char fname[1024];

  sprintf(fname,animMask,scanId);

  FILE *f = fopen(fname,"w");

  if(f == NULL) ABORT("Can't store animation: %s\n",fname);

  if(scanId == obs0.getLastScanId()){
    fprintf(f,"obs = [...\n");
    const ObservationInterface *const* scan = obs0.getLastScan();

    for(i = 0; i < obs0.numObsLastScan(); ++i){
      scan[i]->matlabDump(f);
      fprintf(f,"\n");
    }
    fprintf(f,"];\n");
  }else{
    fprintf(f,"obs = [];\n");
  }

  fprintf(f,"las = [...\n");
  for(i = 0; i < las.numPoints(); ++i)
    fprintf(f," %e",las.r(i));
  fprintf(f,"];\n");

  Slam.animationDump(f);

  fclose(f);
}

inline void doAnimation(const LaserSource& las){
  if(dumpAnim)  dumpAnimation(las);
}

void logTime(FILE* f, double ts){
  if(f != NULL){
    fprintf(f,"%04d %.10f\n", scanId, GetCurrentTime() - ts);
  }
}

//Signal Handler to catch ^C
void onKillSignal(int signal){
  static int count = 0;

  count += 1;

  if(count < 5){
    REPORT("Received kill signal: exiting.\n");
    stop = true;
  }else{
    ABORT("Received %d kill signals, aborting\n", count);
  }
}


void lasEvent(const LaserSource &las){
  //static int numF = 0;
  int n;
  int i;

  if(T0 < 0){
    T0  = las.TimeStamp();
    Ts0 = GetCurrentTime();
  }

  TLOG_LAS_START;

  RobotPose odo   = robot.getTruePose(las.TimeStamp());

  scanId += 1;

  REPORT("LAS: %6.3f,(%6.3f) sec\t%6.2fm -- (%04d)"
	 , las.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
	 , scanId + 1
	 );

  mapper.gridMapper->addReading(las, &odo);

  //Skip first 5 scans, so that grid mapper fills up the buffer
  if(scanId < 5){    
    REPORT("[skipping]\n");
    return;  
  }else if(scanId == 5){
    Slam.start(&robot, &DEFAULT_BUILDER);
  }

  if(!Slam.hasMoved()){
    REPORT("[no move]\n");
    doAnimation(las);
    return;
  }


  double rotVel = robot.getRotVelocity();
  if(rotVel > 8*DEGTORAD){
    REPORT("[skipped]\n");
    Slam.storeOdo(las.TimeStamp(), scanId);
    doAnimation(las);
    return;
  }else{
    //    REPORT("\n");
  }


  n = extractCorners(obs,las);

  REPORT("[%02d]\n",n);

  if(n <= 0){
    Slam.storeOdo(las.TimeStamp(), scanId);
    doAnimation(las);
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

  doAnimation(las);
}

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


  if(scanId > 5){
     Slam.odometry(&o);
     if(Slam.hasMoved()){ TlastMove = odo.TimeStamp(); }
  }

  TLOG_ODO_STOP;
}


int main(int argc, char **argv){

  if(argc < 5){
    fprintf(stderr,"Usage: %s odo_file las_file out_prefix numP "
                   "[seed anim_mask]\n"
	    ,argv[0]);
    return 1;
  }


  char *odo_file = NULL;
  char *obs_file = NULL;
  char *map_out = NULL;
  int numP;
  int seed = time(NULL);

  char buf[1024];

  odo_file = argv[1];
  obs_file = argv[2];
  map_out = argv[3];
  numP = atoi(argv[4]);

  if(argc > 5) seed = atoi(argv[5]);
  if(argc > 6) {
    animMask = argv[6];
    dumpAnim = true;
  }

  //Init input
  FileOdoSource fodo(odo_file);
  FileLaserSource flas(obs_file,8.18);

  if(!fodo.isOk()){
    fprintf(stderr,"Failed to open: %s\n",argv[1]);
    return -1;
  }

  if(!flas.isOk()){
    fprintf(stderr,"Failed to open: %s\n",argv[2]);
    return -1;
  }

  //Misc. setup
  signal(SIGINT, onKillSignal);

  //Init SLAM parameters

  Slam.CHECK_DIST = 0.2;
  Slam.CHECK_NOBS = 5;
  Slam.MAX_HYPE_PER_AGENT = 10;
  Slam.MAX_AGENTS = 11;
  Slam.NUMP_PER_AGENT = numP;
  Slam.RESAMPLE_STEP = 5;
  Slam.setSeed(seed);      //Randomise

  //Init grid mapper
  initDROSGridMap(-10,-10,10,10, DROS_GridCellSize);
  mapper.gridMapper = new LaserGridMapper(5, HALF_BEAM_WIDTH, 0.00,true);


  //Init logs
  sprintf(buf,"%s_mat.m",map_out);
  mfile = fopen(buf,"w");
  timeLog = fopen("laser_time.log","w");

  sprintf(buf,"%s_pfilt",map_out);
  Slam.setDebugLog(buf);


  //Set Mapping Process parameters
  mapper.useNegativeInfo     = false;
  mappingProcess             = &mapper;
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;

  mappingProcess->setRobot2SensorFunction(robot2sensor);

  DEFAULT_BUILDER.setSensorRange(mappingProcess->sensorRange);

  initLaserCornerExtractor();

  printf("Starting SLAM:\n"
         "---odometry    file: %s\n"
         "---observation file: %s\n"
         "---Num. Particles  : %d\n"
         "---Seed            : %d\n"
         "---Output          : %s\n"
         ,odo_file, obs_file, numP,seed, map_out
	 );

  fprintf(mfile, 
         "%%Starting SLAM:\n"
         "%%---odometry    file: %s\n"
         "%%---observation file: %s\n"
         "%%---Num. Particles  : %d\n"
         "%%---Seed            : %d\n"
         "%%---Output          : %s\n"
         ,odo_file, obs_file, numP,seed, map_out
	 );



  //Run Simulation

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);


  //Store results
  Slam.finalise();

  printf("Storing Results\n"); fflush(stdout);
  MappingAgent *bestAgent = Slam.BestAgent();

#if 1
  GlobalMap gmap(bestAgent->getMap());
  gmap.setReferenceFrame(0);
  gmap.store(map_out,Landmark2d_storeLocalMap);
#else
  bestAgent->getMap().store(map_out,Landmark2d_storeLocalMap);
#endif

  bestAgent->dumpPath(mfile,"odo");
  Slam.dumpOdo2Scan(mfile,"odo2scan");

  //  obs0.matlabDump(mfile,"obs0");

  fclose(mfile);


  return 0;
}
