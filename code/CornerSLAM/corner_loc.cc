#ifndef DEPEND
#include <math.h>
#include <time.h>
#include <signal.h>
#endif

#include "TimeTools.h"
#include "DataSource.h"
#include "debug.h"
#include "DataStore.h"
#include "timeLog.h"

#include "odoXR4000.h"
#include "corner.h"
#include "localiser.h"
#include "LaserCornerExtractor.h"


#define M_log(format, args...) fprintf(mfile, format, ##args)
#define REPORT(format, args...) printf(format, ##args)

extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

CornerMappingProcess    mapper; //for load map

//Robot: odometry and sensor
MotionModel                 robot;
RangeAndBearingSensorRange  sensorRange;

//Localiser
//Landmark2dLocGuts           gutsShiny;
CornerLocGuts               guts;
LocaliserFilter             loc;

//Storage for observations
ObservationStore obs0(5000,1000);
int scanId = -1;
int lastScanResample = -1;

double *odo_t    = NULL;
int     sz_odo_t = 0;
int     n_odo_t  = 0;
double lastOdoTimeStamp = -1;

//Time: robot and simulation
double T0  = -1.0;
double Ts0 = -1.0;

// Other stuff
char *out_prefix;
FILE *mfile;
bool stop = false;

//Signal Handler to catch ^C
void onKillSignal(int signal){
  REPORT("Received kill signal: exiting.\n");
  stop = true;
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

  scanId += 1;

  REPORT("LAS: %6.3f,(%6.3f) sec\t%6.2fm -- (%04d)"
	 , las.TimeStamp() - T0
	 , GetCurrentTime() - Ts0
         , robot.odometer().dist()
	 , scanId
	 );

  if(!loc.hasMoved()){
      REPORT("[no move]\n");
      TLOG_LAS_STOP;
      return;
  }

  double rotVel = robot.getRotVelocity();

  if(rotVel > 9.0*DEGTORAD){
    REPORT("[skipped]\n");

    TLOG_LAS_STOP;
    return;
  }

  n = extractCorners(obs,las);

  REPORT("[%02d]",n);

  if(n <= 0){
    REPORT("\n");
    TLOG_LAS_STOP;
    return;
  }

  int obs_ind[n];
  obs0.startScan();
  for(i = 0; i < n; ++i){
    obs_ind[i] = obs0.add(obs[i], scanId); //ScanId -- 0 based
  }

  loc.evaluate(las.TimeStamp(), obs0);
  loc.normalise();

  if(scanId - lastScanResample > 5){
    lastScanResample = scanId;
    loc.resample();
    REPORT(" -- resample.");
  }

  REPORT("\n");
  if(n_odo_t >= sz_odo_t){
    sz_odo_t += 5000;
    odo_t = (double*) realloc(odo_t, sz_odo_t*sizeof(double));
  }

  odo_t[n_odo_t] = lastOdoTimeStamp;
  n_odo_t += 1;

  TLOG_LAS_STOP;
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

  loc.advance(&o);
  lastOdoTimeStamp = odo.TimeStamp();

  TLOG_ODO_STOP;
}

void initLocaliser(LocaliserFilter *loc, RobotPoseCov &pose0){
  RobotPoseSampler sampler(pose0);
  int i;

  for(i = 0; i < loc->numParticles(); ++i){
    loc->setPose(i, sampler.sample());
  }
}

void globalLocalise(LocaliserFilter *loc){
  REPORT("Global Init not supported\n");
  exit(1);
}

class LocaliserLogPoses: public LocaliserMonitor{
private:
  FILE* f;


public:
  LocaliserLogPoses():f(NULL){;}
  LocaliserLogPoses(FILE* file):f(file){;}

  //Interfaces
  //Called by particle filter once before and after processing observation

  /*
  void startOdo(const LocaliserFilter *pf,
                const OdometryReadingInterface *reading){;}
  */
  
  void moved(const LocaliserParticle* p){
    const RobotPose &pose = p->getPose(); 
    fprintf(f," %+.9e %+.9e %+.9e %+.9e", pose.x, pose.y, pose.rot,
              p->getWeight());
  }

  void endOdo(const LocaliserFilter *pf){
    fprintf(f,"\n");
  }

};

extern const double ZEROS_3x3[3][3];

void robotPoseCov_destructor(void *ptr){
  RobotPoseCov* p = (RobotPoseCov*) ptr;
  delete p;
}

class LocaliserLogMean: public LocaliserMonitor{
private:
  double (*poses)[4];
  int sz_poses;

public:
  GenericStack odoStack;

  RobotPoseCov mean;
  int np;
  double sumW;
  double minBound[3];
  double maxBound[3];

  LocaliserLogMean():poses(NULL),sz_poses(0){
    odoStack.setDataDestructor(robotPoseCov_destructor);
  }

  ~LocaliserLogMean(){ if(poses != NULL) delete[] poses; }

  //Observation update
//   void startObservation(const LocaliserFilter *f,
// 			const ObservationStore &obs_store){
//   }
  
//   //Matched obs => map element
//   void observe(const LocaliserParticle* p, int obsInd, int mapInd){;}

//   void observation(const LocaliserParticle* p, const RobotPose *r){
//   }

//   //No match for an observation
//   void noMatch(const LocaliserParticle* p, int obsInd){
//   }


//   void endObservation(const LocaliserFilter *f){
//   }


  //Odometry update
  void startOdo(const LocaliserFilter *pf,
                const OdometryReadingInterface *reading){
    np = 0;
    sumW = 0.0;
    mean.set(0,0,0,ZEROS_3x3);
    minBound[0] = minBound[1] = minBound[2] = +Inf;
    maxBound[0] = maxBound[1] = maxBound[2] = -Inf;

    if(sz_poses != pf->numParticles()){
      if(poses != NULL) delete[] poses;
   
      sz_poses = pf->numParticles();
      poses = new double[sz_poses][4];
    }

  }
  
  void moved(const LocaliserParticle* p){
    const RobotPose &pose = p->getPose(); 
    mean += pose*p->getWeight();
    sumW += p->getWeight();

    poses[np][0] = pose.x;
    poses[np][1] = pose.y;
    poses[np][2] = pose.rot;
    poses[np][3] = p->getWeight();

    np   += 1;

    if(minBound[0] > pose.x)   minBound[0] = pose.x;
    if(minBound[1] > pose.y)   minBound[1] = pose.y;
    if(minBound[2] > pose.rot) minBound[2] = pose.rot;

    if(maxBound[0] < pose.x)   maxBound[0] = pose.x;
    if(maxBound[1] < pose.y)   maxBound[1] = pose.y;
    if(maxBound[2] < pose.rot) maxBound[2] = pose.rot;
  }

  void endOdo(const LocaliserFilter *pf){
#if 0
    REPORT("EndOdo: %d %.5f (avg %.5f)\n",np, sumW, sumW/np);
    REPORT("  pose = [%+7.3f %+7.3f %+7.3f]\n"
           , mean.x, mean.y, mean.rot*RADTODEG);
    REPORT("  spread = [%7.3f %7.3f %7.3f]\n"
	   ,maxBound[0] - minBound[0]
	   ,maxBound[1] - minBound[1]
	   ,RADTODEG*(maxBound[2] - minBound[2]));
#endif

    if(sumW > 0){
      mean /= sumW;
      computeCovariance();
    }

    odoStack.add(new RobotPoseCov(mean));
  }

  void odoNoMove(const LocaliserFilter *pf){
    odoStack.add(new RobotPoseCov(mean));
  }


  //misc.

  void dumpOdoStack(FILE *f){
    RobotPoseCov *out[odoStack.numElements()];
    odoStack.getAll((GenericStackType*)out);
    int i;

    for(i = 0; i < odoStack.numElements(); ++i){
      const RobotPoseCov *p = out[i];
      fprintf(f,"%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e\n"
             , p->x, p->y, p->rot
	     , p->cov[0][0], p->cov[0][1]  ,p->cov[0][2]
	     , p->cov[1][0], p->cov[1][1]  ,p->cov[1][2]
	     , p->cov[2][0], p->cov[2][1]  ,p->cov[2][2] );
   }

  }

  void computeCovariance(){
    int i;
    double scaler = 1/sumW;

    for(i = 0; i < np; ++i){
      double x,y,o,w;
      x = poses[i][0] - mean.x;
      y = poses[i][1] - mean.y;
      o = angleDiffRad(poses[i][2] , mean.rot);
      w = poses[i][3]*scaler;

      mean.cov[0][0] += x*x*w;
      mean.cov[0][1] += x*y*w;
      mean.cov[0][2] += x*o*w;
      //mean.cov[1][0] += x*y*w;
      mean.cov[1][1] += y*y*w;
      mean.cov[1][2] += y*o*w;
      //mean.cov[2][0] += x*o*w;
      //mean.cov[2][1] += y*o*w;
      mean.cov[2][2] += o*o*w;
    }

    mean.cov[1][0] = mean.cov[0][1];
    mean.cov[2][0] = mean.cov[0][2];
    mean.cov[2][1] = mean.cov[1][2];
  }
};

bool loadMap(const char* map_file){
  FILE* f = fopen(map_file,"r");

  if(f == NULL){
    fprintf(stderr,"Failed to open file: %s\n", map_file);
    return false;
  }

  MatlabVariable m;

  if(m.read(f) != MatlabVariable::noError){
    fprintf(stderr,"Matlab module couldn't read it: %s\n",map_file);
    fclose(f);
    return false;
  }
  fclose(f);


  SimpleMap *map = mapper.loadLocalMap(m);

  if(map == NULL) return false;

  guts.map = map;

  return true;
}

LocaliserLogMean mon;

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

  out_prefix = argv[4];

  if(argc > 5) np   = atoi(argv[5]);
  if(argc > 7) seed = atoi(argv[7]);

  if(argc > 6){
    varPose = argv[6];
    double x,y,a;

    if(sscanf(varPose,"%lf:%lf:%lf",&x,&y,&a) != 3){
      REPORT("Invalid pose argument: %s | expect x:y:rot (deg)\n",varPose);
      return 1;
    }
    double Sxx = POW2(0.2/3);
    double Syy = Sxx;
    double Saa = POW2(2*DEGTORAD/3);

    double COV0[3][3] = {{Sxx, 0, 0},{0, Syy, 0},{0, 0, Saa}};
    pose0.set(x,y,a*DEGTORAD,COV0);
  }


  //Set kill signal 
  signal(SIGINT, onKillSignal);

  //Init logs
  sprintf(buf,"%s_mat.m",out_prefix);
  mfile = fopen(buf,"w");
  TLOG_INIT(mfile);

  sprintf(buf,"%s_poses.txt",out_prefix);
  FILE *f_poses = fopen(buf,"w");
  if(f_poses == NULL){
    REPORT("Failed to open: %s\n", buf);
    return 1;
  }

  initLaserCornerExtractor();

  //Randomise
  loc.setSeed(seed);

  //Load the map
  if(!loadMap(varMap)){
    fprintf(stderr,"Failed to load: %s\n", varMap);
    return 1;
  }

  guts.monitor = &mon;

  loc.addMonitor(&mon);
  loc.init(np, &robot, &guts);

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
  FileLaserSource flas(varLas,8.18);

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);


  //Store Results
  mon.dumpOdoStack(f_poses);

  int i;
  fprintf(mfile,"odo_t = [ ...\n");
  for(i = 0; i < n_odo_t; ++i){
    fprintf(mfile, "%.3f\n", odo_t[i]);
  }
  fprintf(mfile,"];\n");

  obs0.matlabDump(mfile,"obs0");

  fclose(mfile);
  fclose(f_poses);

  return 0;
}
