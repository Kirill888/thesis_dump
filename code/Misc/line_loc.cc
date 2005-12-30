#ifndef DEPEND
#include <math.h>
#include <time.h>
#include <signal.h>
#endif

#include "TimeTools.h"
#include "DataSource.h"
#include "DataStore.h"

#include "odoXR4000.h"
#include "LineLocaliser.h"
#include "localiser.h"

#define REPORT(format, args...) printf(format, ##args)

bool stop = false;
double lastOdoTimeStamp = -1;
int scanId = -1;
int lastScanResample = -1;
FILE *f_poses;

//Time: robot and simulation
double T0  = -1.0;
double Ts0 = -1.0;

//Robot: odometry and sensor
MotionModel   robot;

//RangeAndBearingSensorRange  sensorRange;

//Localiser
LineLocaliser loc;


//Signal Handler to catch ^C
void onKillSignal(int signal){
  REPORT("Received kill signal: exiting.\n");
  stop = true;
}

extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

ObservationStore obs0;
LineObservation  scanObs;
LineMap          lmap;

void displayLineReport(){
  int i;
  int minReport = loc.numParticles()*3;

  REPORT("LL:");
  for(i = 0; i < lmap.numLines(); ++i){
    if(lmap.numObserved(i) >= minReport){
      REPORT(" %02d", i+1);
    }
  }
  REPORT(" CC:");
  for(i = 0; i < lmap.numLines(); ++i){
    if(lmap.numConflict(i) > minReport){
      REPORT(" %02d", i+1);
    }
  }
}

//Laser Event
void lasEvent(const LaserSource &las){
  //set the scanObs variables, call evaluate, normalise, resample
  int i;
  int n = 0;
  scanId += 1;

  REPORT("LAS: %6.3f,(%6.3f sec)\t[%7.3f m] -- (%04d)"
	 , las.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
	 , scanId
	 );

  if(!loc.hasMoved()){
      REPORT("[no move]\n");
      return;
  }

  double rotVel = robot.getRotVelocity();

  if(rotVel > 9.0*DEGTORAD){
    REPORT("[skipped]\n");
    return;
  }

  //
  for(i = 0; i < las.numPoints(); i += 1){
    if(las.isValidReading(i)){
      scanObs.range[n] = las.r(i);
      scanObs.angle[n] = las.a(i) - M_PI_2;
      n += 1;
    }
  }
  scanObs.n = n;

  lmap.resetScores();

  loc.evaluate(las.TimeStamp(), obs0);


  if(scanId - lastScanResample > 5){
    lastScanResample = scanId;
    loc.resample();
    REPORT("[RL] ");
  }else{
    REPORT("[  ] ");
  }

  displayLineReport();

  REPORT("\n");
}

void logOdo(FILE *f){
  RobotPoseCov p;

  loc.getPose(&p);

  fprintf(f,"%+.9e %+.9e %+.9e "
	  "%+.9e %+.9e %+.9e "
	  "%+.9e %+.9e %+.9e "
	  "%+.9e %+.9e %+.9e\n"
	  , p.x, p.y, p.rot
	  , p.cov[0][0], p.cov[0][1]  ,p.cov[0][2]
	  , p.cov[1][0], p.cov[1][1]  ,p.cov[1][2]
	  , p.cov[2][0], p.cov[2][1]  ,p.cov[2][2] );

}

//Odometry Event
void odoEvent(const OdoSource &odo){
  if(T0 < 0){
    T0 = odo.TimeStamp();
    Ts0 = GetCurrentTime();
  }

  RobotPoseOdo o((RobotPose)odo, odo.TimeStamp());

  robot.newOdometry(&o);

  REPORT("ODO: %6.3f (%6.3f sec)\t[%7.3f m]\n"
	 , odo.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
        );

  loc.advance(&o);
  lastOdoTimeStamp = odo.TimeStamp();

  logOdo(f_poses);
}

bool loadMap(LineMap* lmap, const char* map_file){
  bool res;

  FILE* f = fopen(map_file,"r");
  if(f == NULL) return false;

  res = lmap->load(f);

  fclose(f);

  return res;
}

void initLocaliser(LineLocaliser *loc, RobotPoseCov &pose0){
  RobotPoseSampler sampler(pose0);
  int i;

  for(i = 0; i < loc->numParticles(); ++i){
    loc->setPose(i, sampler.sample());
  }
}

void globalLocalise(LineLocaliser *loc){
  REPORT("Global Init not supported\n");
  exit(1);
}



int main(int argc, char **argv){
  if(argc < 5){
    fprintf(stderr,"Usage: %s map odo las out_prefix [np, pose, seed]\n"
	    ,argv[0]);
    return 1;
  }

  char buf[1024];

  char *varMap  = NULL;
  char *varLas  = NULL;
  char *varOdo  = NULL;
  char *varPose = NULL;
  RobotPoseCov pose0;

  int np = 1000;
  int seed = time(NULL);

  //Parse command line
  varMap     = argv[1];
  varOdo     = argv[2];
  varLas     = argv[3];

  char *out_prefix = argv[4];

  if(argc > 5) np   = atoi(argv[5]);
  if(argc > 7) seed = atoi(argv[7]);

  if(argc > 6){
    varPose = argv[6];
    double x,y,a;

    if(sscanf(varPose,"%lf:%lf:%lf",&x,&y,&a) != 3){
      REPORT("Invalid pose argument: %s | expect x:y:rot (deg)\n",varPose);
      return 1;
    }
    double Sxx = POW2(0.5/3);
    double Syy = Sxx;
    double Saa = POW2(5*DEGTORAD/3);

    double COV0[3][3] = {{Sxx, 0, 0},{0, Syy, 0},{0, 0, Saa}};
    pose0.set(x,y,a*DEGTORAD,COV0);
  }

  //Load the map
  if(!loadMap(&lmap, varMap)){
    fprintf(stderr,"Failed to load: %s\n", varMap);
    return 1;
  }


  struct LineLaserMatchParams params;
  params.sigma_a  = deg2rad(0.3);  params.sigma_a2 = POW2(params.sigma_a);
  params.sigma_r  = 0.05;          params.sigma_r2 = POW2(params.sigma_r);

  params.maxRange = 8.131;
  params.maxDistToLine = params.sigma_r*3;

  //Set kill signal 
  signal(SIGINT, onKillSignal);

  //Init logs
  sprintf(buf,"%s_mat.m",out_prefix);
  FILE *mfile = fopen(buf,"w");

  sprintf(buf,"%s_poses.txt",out_prefix);
  f_poses = fopen(buf,"w");
  if(f_poses == NULL){
    REPORT("Failed to open: %s\n", buf);
    return 1;
  }

  //Randomise
  loc.setSeed(seed);

  //Init
  obs0.startScan();
  scanObs.setMaxSize(361);
  obs0.add(&scanObs,0);

  loc.init(np, &robot, &lmap, &params);

  if(varPose == NULL){
    globalLocalise(&loc);
  }else{
    initLocaliser(&loc, pose0);
  }

#if PFILT_DEBUG_LOG
  sprintf(buf,"%s_pfilt",out_prefix);
  loc.setDebugLog(buf);
#endif

  //Run Simulation
  FileOdoSource fodo(varOdo);
  FileLaserSource flas(varLas);

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);


  //Store Results
  //  mon.dumpOdoStack(f_poses);

//   int i;
//   fprintf(mfile,"odo_t = [ ...\n");
//   for(i = 0; i < n_odo_t; ++i){
//     fprintf(mfile, "%.3f\n", odo_t[i]);
//   }
//   fprintf(mfile,"];\n");

  fclose(mfile);
  fclose(f_poses);

  return 0;
}
