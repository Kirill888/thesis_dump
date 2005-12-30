#ifndef __SIMPLE_SLAM__
#define __SIMPLE_SLAM__

#include "slam.h"
#include "util.h"
#include "mapRegion.h"
#include "timeLog.h"
#include "localmap.h"

class SimpleSLAM: public SLAMParticleOwner, public SLAMMonitor{

 private:
  SLAM slam;
  int n_obs;
  int np;
  LocalMap lmap;
  RegionAll mapRegion;
  SLAMMonitorMultiplex multiMonitor;

  int n_odo;
  int sz_odo;
  int *odo2scan;

  void checkPoint();
  void computeMean(RobotPoseCov &);

 public:

  int RESAMPLE_STEP;
  int NP_MIN;
  int NP_MAX;

  SimpleSLAM():n_obs(0)
    ,n_odo(0), sz_odo(0), odo2scan(NULL)
    ,RESAMPLE_STEP(2),  NP_MIN(100), NP_MAX(1000){
    multiMonitor.add(this);
    monitor = &multiMonitor;
    slam.addMonitor(monitor);
  }

  ~SimpleSLAM(){ 
    DESTROY(mapper); 
    if(odo2scan != NULL) free(odo2scan);
   }

  void setup(const MotionModelInterface *robot,
             const MapBuilderInterface *mapper){
    slam.setOdoModel(robot);
    this->mapper = mapper->clone();
    this->mapper->setMapArea(&mapRegion);
  }

  void setMonitor(SLAMMonitor *mon){
    multiMonitor.add(mon);
  }

  void start(){
    int i;
    n_obs = 0;
    np = NP_MIN;
    slam.resize(np);
    for(i = 0; i < np; ++i){
      slam.setp(i, slam.makeNewParticle(this));
    }
  }

  void observation(double TimeStamp, const ObservationStore &obs,
                   const int * obs_ind, int nobs){
    if(nobs <= 0) return;

    n_obs += 1;


    TLOG_EVAL_START;
    storeOdo(TimeStamp, obs.getLastScanId());
    slam.evaluate(TimeStamp, obs); 
    slam.normalise();
    TLOG_EVAL_STOP;

    if( n_obs % 3 == 0){
      checkPoint();
    }

    if( (n_obs % RESAMPLE_STEP) == 0){
      TLOG_RESAMPLE_START;
      slam.resample(np);
      TLOG_RESAMPLE_STOP;
      printf("RESAMPLE\n");
    }

  }

  void odometry(const OdometryReadingInterface *odo){
    slam.advance(odo);
  }

  void finalise(){
    lmap.set(&slam);
  }

  const LocalMap* getMap()const{return &lmap;}
  const OdometryStore* getPath()const{
    int i = slam.findBestParticle();
    if(i < 0) return NULL;

    return &slam[i]->getOdo();
  }


  //Inlines
  bool hasMoved()const{return slam.hasMoved();}

  void storeOdo(double T, int ScanId){ 
    if(n_odo >= sz_odo){
      sz_odo += 5000;
      odo2scan = (int*) realloc(odo2scan, sz_odo*sizeof(int));
    }

    odo2scan[n_odo] = ScanId;
    n_odo += 1;

    slam.storeOdo(T); 
  }
  void setSeed(int seed){slam.setSeed(seed);}
  bool dumpOdo2Scan(FILE*f, const char* name);

  const MapBuilderInterface* getMapBuilder()const{ return mapper;}

  void setDebugLog(char *base){
#if PFILT_DEBUG_LOG
    slam.setDebugLog(base);
#endif
  }

};

inline bool SimpleSLAM::dumpOdo2Scan(FILE*f, const char* name){
  int i;

  if( fprintf(f,"%s = [", name) < 0) return false;

  for(i =0; i < n_odo; ++i){
    if((i % 20) == 0) fprintf(f," ...\n");
    if(fprintf(f,"%d ", odo2scan[i]) < 0)  return false;
  }

  if(fprintf(f,"\n]; %% %s\n",name)<0)       return false;

  return true;
}

inline void SimpleSLAM::checkPoint(){
#if 0
  RobotPoseCov mean;
  computeMean(mean);

  double sigma = sqrt(mean.norm());
  double sigNp = norm2np(3*sigma);

  //  np = NP_MIN + (int)((NP_MAX - NP_MIN)*sigNp);

  printf("Check Point: %.3f (%.2f => %d)\n",3*sigma, sigNp,np);
#endif
}

extern const double ZEROS_3x3[3][3];

inline void SimpleSLAM::computeMean(RobotPoseCov &mean){
  
  mean.set(0,0,0, ZEROS_3x3);

  int i;

  for(i = 0; i < slam.numParticles(); ++i){
    mean += slam[i]->getPose()*slam[i]->getWeight();
  }

  //Compute covariance matrix
  for(i = 0; i < np; ++i){
    register double x,y,o;
    register double w_ = slam[i]->getWeight();

    x = slam[i]->getPose().x   - mean.x;
    y = slam[i]->getPose().y   - mean.y;
    o = angleDiffRad(slam[i]->getPose().rot,mean.rot);

    mean.cov[0][0] += x*x*w_;
    mean.cov[0][1] += x*y*w_;
    mean.cov[0][2] += x*o*w_;
    //mean.cov[1][0] += x*y*w_;
    mean.cov[1][1] += y*y*w_;
    mean.cov[1][2] += y*o*w_;
    //mean.cov[2][0] += x*o*w_;
    //mean.cov[2][1] += y*o*w_;
    mean.cov[2][2] += o*o*w_;

  }

  mean.cov[1][0] = mean.cov[0][1];
  mean.cov[2][0] = mean.cov[0][2];
  mean.cov[2][1] = mean.cov[1][2];
}

#endif
