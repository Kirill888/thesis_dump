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
#include "landmark2d.h"
#include "localiser.h"
#include "globalmap.h"
#include "MultiLocaliser.h"


extern int extractShinyFeatures(double *range_out, 
                                double *bearing_out,
                                const LaserSource &las);

#define M_log(format, args...) fprintf(mfile, format, ##args)
#define REPORT(format, args...) printf(format, ##args)

extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

//Robot: odometry and sensor
MotionModel                 robot;
RangeAndBearingSensorRange  sensorRange;

//For SLAM (global localistion)
Landmark2dFactory           factory;
MapBuilder                  mapBuilder;

//Localiser
Landmark2dLocGuts           guts;
MultiLocaliser loc;

//Storage for observations
ObservationStore obs0(5000,1000);
int scanCount = 0;

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

#define doAnimation()

static double *lnd_r = new double[181];
static double *lnd_a = new double[181];

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

  if(!loc.hasMoved()){
      REPORT("[no move]\n");
      TLOG_LAS_STOP;
      return;
  }

  double rotVel = robot.getRotVelocity();

  if(rotVel > 9.0*DEGTORAD){
    REPORT("[skipped]\n");

    doAnimation();
    TLOG_LAS_STOP;
    return;
  }

  n = extractShinyFeatures(lnd_r,lnd_a,las);

  REPORT("[%02d]\n",n);

  if(n <= 0){
    //    Slam.storeOdo(las.TimeStamp(), scanId);
    doAnimation();
    TLOG_LAS_STOP;
    return;
  }

  int obs_ind[n];

  for(i = 0; i < n; ++i){
    double SigmaR2 = POW2(0.1/3.0) + POW2(0.05*lnd_r[i]/3.0);
    double SigmaA2 = POW2(5.0*DEGTORAD/3.0) + POW2(0.8*rotVel/3.0);

    DEBUG_print("\tF[%d] = (%6.3f,%6.3f)\n"
		,i,lnd_r[i], lnd_a[i]*RADTODEG);

    obs_ind[i] = obs0.add(new Landmark2dObs(lnd_r[i],SigmaR2,lnd_a[i],SigmaA2)
                          ,scanId);
  }


  loc.observation(las.TimeStamp(), obs0, obs_ind, n);

  TLOG_LAS_STOP;

  doAnimation();

  print_memstat(stdout);
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



  loc.odometry(&o);

  TLOG_ODO_STOP;
}

void initLocaliser(LocaliserFilter *loc, RobotPoseCov &pose0){
  RobotPoseSampler sampler(pose0);
  int i;

  for(i = 0; i < loc->numParticles(); ++i){
    loc->setPose(i, sampler.sample());
  }
}

void initLocaliser(LocaliserFilter *loc, LocalMap &map){
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

  int numNoMatch[100];
  int nObs;
  int obs_i0;
  const ObservationStore* allObs;


  LocaliserLogMean():poses(NULL),sz_poses(0){
    odoStack.setDataDestructor(location_destructor);
  }

  ~LocaliserLogMean(){ if(poses != NULL) delete[] poses; }

  //Observation update
  void startObservation(const LocaliserFilter *f,
			const ObservationStore &obs_store,
			const int *obs_ind, int nobs){

    memset(numNoMatch,0,nobs*sizeof(int));

    odoStack.add(new Location(0,mean));

    nObs = nobs;
    obs_i0 = obs_ind[0];
    allObs = &obs_store;
  }
  
  //Matched obs => map element
  //  void observe(const LocaliserParticle* p, int obsInd, int mapInd){;}

  /*
  void observation(const LocaliserParticle* p, const RobotPose *r){
    REPORT("P.OBS: %.3f %.3f %.3f\n", r->x, r->y, r->rot*RADTODEG);
  }
  */

  //No match for an observation
  void noMatch(const LocaliserParticle* p, int obsInd){
     numNoMatch[obsInd - obs_i0] += 1;
  }


  void endObservation(const LocaliserFilter *f){
    int i;
    REPORT("OBS:[%05d]",obs_i0);
    for(i = 0; i < nObs; ++i){
      REPORT(" %4d",numNoMatch[i]);
    }
    REPORT("\n");

    for(i = 0; i < nObs; ++i){
      if(numNoMatch[i] >= np){
	const Landmark2dObs  *o = (const Landmark2dObs*) 
                                           allObs->get(i + obs_i0);
	RobotPose laser;
	robot2sensor(&laser, &mean);
// 	RobotPose laser(LASER_SENSOR_POSE);
// 	laser.translateMe(mean);

	Gaussian2d g(o->toCartesian(&laser));

	REPORT(" %.5e %.5e %.5e %.5e %.5e %.5e %%missing\n"
               ,g.x, g.y
               ,g.cov[0][0], g.cov[0][1]
               ,g.cov[1][0], g.cov[1][1]);
      }
    }

  }


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
    REPORT("EndOdo: %d %.5f (avg %.5f)\n",np, sumW, sumW/np);
    REPORT("  pose = [%+7.3f %+7.3f %+7.3f]\n"
           , mean.x, mean.y, mean.rot*RADTODEG);
    REPORT("  spread = [%7.3f %7.3f %7.3f]\n"
	   ,maxBound[0] - minBound[0]
	   ,maxBound[1] - minBound[1]
	   ,RADTODEG*(maxBound[2] - minBound[2]));

    if(sumW > 0){
      mean /= sumW;
      computeCovariance();
    }
  }


  //misc.

  void dumpOdoStack(FILE *f){
    Location *out[odoStack.numElements()];
    odoStack.getAll((GenericStackType*)out);
    int i;

    for(i = 0; i < odoStack.numElements(); ++i){
      const RobotPoseCov &p = out[i]->pose;
      fprintf(f,"%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e\n"
             , p.x, p.y, p.rot
	     , p.cov[0][0], p.cov[0][1]  ,p.cov[0][2]
	     , p.cov[1][0], p.cov[1][1]  ,p.cov[1][2]
	     , p.cov[2][0], p.cov[2][1]  ,p.cov[2][2] );
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


int main(int argc, char **argv){

  if(argc < 5){
    fprintf(stderr,"Usage: %s map odo las out_prefix [n_loc, n_slam, pose]\n"
	    ,argv[0]);
    return 1;
  }

  char buf[1024];

  char *varMap  = NULL;
  char *varLas  = NULL;
  char *varOdo  = NULL;
  char *varPose = NULL;
  RobotPoseCov pose0;
  int startMap = 0;

  int np = 10000;
  int np_slam = 300;

  //Parse command line
  varMap     = argv[1];
  varOdo     = argv[2];
  varLas     = argv[3];

  out_prefix = argv[4];

  if(argc > 5) np = atoi(argv[5]);
  if(argc > 6) np = atoi(argv[6]);

  if(argc > 7){
    varPose = argv[7];
    double x,y,a;

    if(sscanf(varPose,"%d_%lf:%lf:%lf",&startMap,&x,&y,&a) != 4){
      REPORT("Invalid pose argument: %s | expect map_x:y:rot\n",varPose);
      return 1;
    }
    double Sxx = POW2(0.5/3);
    double Syy = Sxx;
    double Saa = POW2(5*DEGTORAD/3);

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


#if PFILT_DEBUG_LOG
  sprintf(buf,"%s_pfilt",out_prefix);
  loc.setDebugLog(buf);
#endif

  //Set Shiny parameters
  //  LMAP_MapAreaFactory = new Landmark2dMapAreaFactory;
  //  LMAP_LoadMap = Landmark2d_LoadMap;

  //Randomise
  loc.setSeed(time(NULL));

  //Load the map
  GlobalMap gm;

  REPORT("Loading %s",varMap);
  if(gm.load(varMap)){
    REPORT("... ok [%d local maps]\n",gm.numMaps());
  }else{
    REPORT("... failed!\n");
    return 2;
  }

  //Set up
  mapBuilder.setFactory(&factory);
  mapBuilder.setSensorRange(&sensorRange);
  //  LMAP_MapAreaFactory = new Landmark2dMapAreaFactory;

  loc.setup(&gm, &guts, &robot, gm.numMaps(), np, &mapBuilder,np_slam);

  if(varPose == NULL){
    loc.globalLocalise();
  }else{
    RobotPoseSampler sampler(pose0);
    loc.init(startMap, &sampler);
  }

  //Run Simulation
  FileOdoSource fodo(varOdo);
  FileLaserSource flas(varLas);

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);


  //Store Results
  loc.BestHype()->dumpToFile(f_poses);

  obs0.matlabDump(mfile,"obs0");

  fclose(mfile);
  fclose(f_poses);

  return 0;
}
