#ifndef __LOCALISER_HYPE_H__
#define __LOCALISER_HYPE_H__

#include "localiser.h"
#include "DataStore.h"
#include "matrix3.h"
#include "globalmap.h"
#include "util.h"

#define USE_WEIGHTED_MEAN 0

class LocaliserStateMonitor: public LocaliserMonitor{
private:
  double (*poses)[4];
  int sz_poses;

  void set(const LocaliserStateMonitor *other){
    poses = NULL;
    sz_poses = 0;

    mean = other->mean;
    np = other->np;
    sumW = other->sumW;
    map = other->map;

    nObs = 0;
    obs_i0 = 0;
    allObs = NULL;
  }

public:
  RobotPoseCov mean;
  int np;
  double sumW;
  double minBound[3];
  double maxBound[3];

  int obsMatch[100][3]; //Core,Peripheral,NoMatch
  int nObs;
  int obs_i0;
  const ObservationStore* allObs;

  const SimpleMap *map;


  LocaliserStateMonitor():poses(NULL),sz_poses(0)
     ,mean(NaN,NaN,NaN,ZEROS_3x3),np(0),sumW(0),map(NULL){;}

  LocaliserStateMonitor(const LocaliserStateMonitor &other){
    set(&other);
  }

  const LocaliserStateMonitor& operator=(const LocaliserStateMonitor & rhs){
    if(&rhs == this) return *this;
    DESTROY_ARRAY(poses);
    set(&rhs);
    return *this;
  }

  ~LocaliserStateMonitor(){ if(poses != NULL) delete[] poses; }

  //Observation update
  void startObservation(const LocaliserFilter *f,
			const ObservationStore &obs_store,
			const int *obs_ind, int nobs){

    memset(obsMatch,0,nobs*sizeof(int[3]));

    nObs = nobs;
    obs_i0 = obs_ind[0];
    allObs = &obs_store;
  }
  
  //Matched obs => map element
  void observe(const LocaliserParticle* p, int obsInd, int mapInd){
    int i = obsInd - obs_i0;

    if(map->get(mapInd)->isCore()){
      obsMatch[i][0] += 1;
    }else{
      obsMatch[i][1] += 1;
    }
  }

  /*
  void observation(const LocaliserParticle* p, const RobotPose *r){
    REPORT("P.OBS: %.3f %.3f %.3f\n", r->x, r->y, r->rot*RADTODEG);
  }
  */

  //No match for an observation
  void noMatch(const LocaliserParticle* p, int obsInd){
     obsMatch[obsInd - obs_i0][2] += 1;
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
#if USE_WEIGHTED_MEAN
    mean += pose*p->getWeight();
#else
    mean += pose;
#endif

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

#if USE_WEIGHTED_MEAN
    if(sumW > 0){
      mean /= sumW;
      computeCovariance();
    }else{
      mean.set(NaN,NaN,NaN,ZEROS_3x3);
    }
#else
    if(np > 0){
      mean /= ((double) np);
      computeCovariance();
    }else{
      mean.set(NaN,NaN,NaN,ZEROS_3x3);
    }
#endif

  }


  //misc.

  const ObservationInterface* getObs(int i)const{
    return allObs->get(obs_i0 + i);
  }


  void computeCovariance(){
    int i;
#if USE_WEIGHTED_MEAN
    double scaler = 1/sumW;
#else
    double scaler = 1/((double) np);
#endif

    for(i = 0; i < np; ++i){
      double x,y,o,w;
      x = poses[i][0] - mean.x;
      y = poses[i][1] - mean.y;
      o = angleDiffRad(poses[i][2] , mean.rot);

#if USE_WEIGHTED_MEAN
      w = poses[i][3]*scaler;
#else
      w = scaler;
#endif

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


#define MAX_NEIGHBOURS 100

class LocaliserHype{
 private:
  //Inner Classes

  class Neighbour{
  public:
    int mapId;
    MapAreaInterface* area;
    double score;

    Neighbour():mapId(-1),area(NULL),score(0.0){;}

    Neighbour(int i, MapAreaInterface* a): mapId(i), area(a), score(0.0){;}

    ~Neighbour(){DESTROY(area);}
  };

  //Private Variables

  LocaliserFilter *loc;
  GlobalMap map;
  LocaliserParticleGuts *guts;
  int currentMap;

  LocaliserStateMonitor monitor;
  GenericStack path;


  int n_neighbours;
  Neighbour *neighbours[MAX_NEIGHBOURS];

  double stayScore;

  //Private Functions
  void initNeighbours();
  bool isValidNeighbour(int mapId);

  void resetScore();
  void normaliseScore();
  double computeScore(const MapAreaInterface*);

  void set(const LocaliserHype* other);

 public:
  ///////////////////////////////////////
  // Creation/Deletion
  ///////////////////////////////////////

  LocaliserHype():loc(NULL),guts(NULL),n_neighbours(0),stayScore(0.0){;}

  LocaliserHype(int id, 
		LocaliserFilter *lf,
                const GlobalMap &m,
                const LocaliserParticleGuts *g):
          loc(lf), map(m), guts(g->clone())
          ,n_neighbours(0),stayScore(0.0){

    guts->id = id;
    guts->monitor = &monitor;
    loc->addMonitor(&monitor);
  }

  LocaliserHype(const LocaliserHype &other):
   loc(NULL),guts(NULL),n_neighbours(0),stayScore(0.0)
   { set(&other); }
  LocaliserHype(const LocaliserHype *other):
    loc(NULL),guts(NULL),n_neighbours(0),stayScore(0.0)
    { set(other); }

  ~LocaliserHype(){ 
    int i;
    DESTROY(guts);
    for(i = 0 ;  i < n_neighbours; ++i) delete neighbours[i];
   }

  LocaliserHype* clone(int newId)const;

  ///////////////////////////////
  //Meat Section
  //////////////////////////////

  //Add numP in the localmap map, sampling using sampler
  void init(int map, const RobotPoseSampler *sampler, int numP);

  //Transfer to a map (doesn't have to be a neighbour)
  void transfer(int map);

  //Remove all particles
  void destroy();

  //Compute State: should be called after observation is finished
  void computeState();

  int checkPoint(int *maps_out, double *w_out);



  /////////////////////////////
  //Set/Get methods
  /////////////////////////////

  void setId(int id){ guts->id = id;}
  int getId()const{ return guts->id;}

  int getCurrentMap()const{return currentMap;}

  int numParticles()const{ return monitor.np;}
  double weight()const{    return monitor.sumW;}

  /////////////////////////////
  // Output methods
  /////////////////////////////

  //Store the path to the file
  void dumpToFile(FILE *f);

};


//////////////////////////////////////////////
//Inlines
////////////////////////////////////////

inline void LocaliserHype::set(const LocaliserHype* other){
  loc        = other->loc;
  map        = other->map;
  currentMap = other->currentMap;
  monitor    = other->monitor;
  path       = other->path;

  if(other->guts == NULL){
    guts = NULL;
  }else{
    guts = other->guts->clone();
    guts->monitor = &monitor;
  }

  if(loc != NULL){
    loc->addMonitor(&monitor);
  }

  initNeighbours();
  resetScore();
}

inline void LocaliserHype::resetScore(){
  stayScore = 0.0;

  for(int i = 0; i < n_neighbours; ++i){
    neighbours[i]->score = 0.0;
  }
}

inline void LocaliserHype::normaliseScore(){
  double sumW = stayScore;

  for(int i = 0; i < n_neighbours; ++i){
    sumW += neighbours[i]->score;
  }

  if(sumW <= 0) return;

  double scaler = 1/sumW;
  stayScore    *= scaler;

  for(int i = 0; i < n_neighbours; ++i){
    neighbours[i]->score *= scaler;
  }
}


#endif
