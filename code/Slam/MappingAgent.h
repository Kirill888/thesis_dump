#ifndef __MAPPING_AGENT_H__
#define __MAPPING_AGENT_H__

#include "globalmap.h"
#include "slam.h"
#include "util.h"
#include "mapRegion.h"
#include "mappingProcess.h"

class MultiSLAM;


#define MAX_NEIGHBOURS 100
#define MAX_OBS  100

//====================================================================
// ParticleHistory
//====================================================================
class ParticleHistory{
 private:
  class SharedData{
  public:
    mutable int copy_count;

    int n;

    int* data;

    SharedData():copy_count(1),n(0), data(NULL){;}

    ~SharedData();

    void set(int i, int v);
    int get(int i)const{ return data[i]; }
    SharedData* makeCopy()const;
    void merge(const SharedData*o);
  };

  SharedData* maps;
  SharedData* trans;

 public:
  ParticleHistory():maps(NULL),trans(NULL){;}
  ParticleHistory(const ParticleHistory &other);

  int getMap(int i)const;
  void setMap(int i, int v);

  int getTransition(int i)const;
  void setTransition(int i, int v);

  void merge(const ParticleHistory* other);

  ParticleHistory * clone()const;
  ~ParticleHistory();

  void debugDump(FILE* f)const;
};

//====================================================================
//LoopCloseResult
//====================================================================
class LoopCloseResult{
 public:
  SimpleMap* map1;
  SimpleMap* map2;
  RobotPoseCov r2in1;

  double w;
  int scanId; //When it was computed
  double probPass;

  LoopCloseResult():map1(NULL), map2(NULL),w(-1),scanId(-1),probPass(0){;}

  LoopCloseResult(const LoopCloseResult &other){
    map1 = CLONE(other.map1);
    map2 = CLONE(other.map2);
    r2in1 = other.r2in1;
    w = other.w;
    scanId = other.scanId;
    probPass = other.probPass;
  }

  ~LoopCloseResult(){
    DESTROY(map1);
    DESTROY(map2);
  }

  void reset(){
    DESTROY(map1);
    DESTROY(map2);
    w = 0.0;
  }
};

//====================================================================
//MA_Monitor
//====================================================================
class MA_Monitor: public SLAMMonitor{

 public:
  int nObs;

  int np;
  double sumW;
  RobotPose mean;
  double ax, ay;

  const SLAMParticle *bestP;

  int nCore,nPeripheral, nMature, nNew, nNoMatch;

  const ObservationStore *obsStore;
  const ObservationInterface *const* scan;

  int lastScanId;
  double lastTimeStamp;

  MA_Monitor():nObs(0), np(0),sumW(0),bestP(NULL),obsStore(NULL),scan(NULL){;}


  ///////////////////////////////
  //Access Functions
  //////////////////////////////
  const ObservationInterface* getObs(int i)const{
    return scan[i];
  }

  //////////////////////////////////////////////////////
  // MonitorInterface
  //////////////////////////////////////////////////////

//Called by particle filter once before and after processing observation
  void startObservation(const SLAM *f, double TimeStamp,
                        const ObservationStore &obs_store){
    //    printf("StartOBS: %d\n", nobs);

    obsStore = &obs_store;
    nObs     = obs_store.numObsLastScan();
    scan     = obs_store.getLastScan();
    lastScanId = obs_store.getLastScanId();

    np    = 0;
    sumW  = 0.0;
    mean.set(0,0,0); 
    ax = ay = 0;

    nCore = nPeripheral = nMature = nNew = nNoMatch = 0;
    bestP = NULL;
    lastTimeStamp = TimeStamp;
  }
  
  void endObservation(const SLAM *f){
    if(np > 0){ 
      double np_inv = 1.0/double(np);
      mean.x *= np_inv;
      mean.y *= np_inv;
      mean.rot = atan2(ay*np_inv, ax*np_inv);
    }
  }

  void onNormalise(double scaler){
    //    printf("OnNormalise: %e\n",scaler);
    sumW *= scaler;
  }

  void observation(const SLAMParticle* p, const RobotPose* r){
    sumW += p->getWeight();
    mean.x += r->x;
    mean.y += r->y;
    ax += cos(r->rot);
    ay += sin(r->rot);

    np   += 1;
    if(bestP){
      if(bestP->getWeight() < p->getWeight()){
	bestP = p;
      }
    }else{
      bestP = p;
    }
  }

  //For every observation that matched
  void observe(const SLAMParticle*, int obsInd
	       ,const MapEntryInterface* m){
    //    printf("OBSERVE: %d => %d\n",obsInd, m->getGlobalId());

    if(m->isNew()){
      nNew        += 1;
    }

    if(m->isCore()){
      nCore       += 1;
    }else{
      nPeripheral += 1;
    }
  }

  //Fot every that didn't match
  void noMatch(const SLAMParticle*, int obsInd){
    //    printf("NoMatch: %d\n", obsInd);
    nNoMatch += 1;
  }

  void onResample(const SLAM*f){
    np   = 0;
    sumW = 0.0;
    bestP = NULL;
  }

  void onClone(const SLAMParticle* parent, const SLAMParticle* clone){
    np += 1;
    sumW += clone->getWeight();

    if(bestP == NULL) bestP = clone;
    else if(bestP->getWeight() < clone->getWeight()) bestP = clone;

  }

};

//====================================================================
//MappingAgent
//====================================================================
class MappingAgent: public SLAMParticleOwner{
 private:
  class Neighbour{
  public:
    int mapId;
    const MapAreaInterface* area;
    double score;

    Neighbour():mapId(-1),area(NULL),score(0.0){;}

    Neighbour(int i, const MapAreaInterface* a): 
              mapId(i), area(a), score(0.0){;}

    Neighbour(const Neighbour* o):mapId(o->mapId),area(o->area)
              ,score(o->score){;}
    //    ~Neighbour(){;}
  };

  class NewMap{
  public:
    MapAreaInterface * region;
    RobotPose x0;

    NewMap():region(NULL){;}
    NewMap(MapAreaInterface* r, const RobotPose &x):region(r),x0(x){;}
  };

  void nullAll(){
   currentMap = -1;
   slam = NULL;
   multiSLAM = NULL;
   superMode = false;
   matchers = NULL;
   sz_matchers = 0;
   nTotalChecks = 0;
   n_neighbours = 0;
   regMap = NULL;
   hist = NULL;
   blocks = NULL;
   n_blocks = 0;
   sz_hist = 0;
  }

 public:
  static int    NUM_PARTICLES_PER_PATH;

  enum MappingAction{Continue=0, StartNewMap, CloseLoop
                    ,TransitToMap};

  struct MappingDecision{
    MappingAction action;
    int map;
    double weight;

    MappingDecision(){
      action = Continue;
      weight = 0.0;
      map = -1;
    }

    MappingDecision(enum MappingAction a, int m, double w){
      action = a;
      map = m;
      weight = w;
    }

  };

  MappingAgent():SLAMParticleOwner(-1,&mon,NULL),path(location_destructor){ 
    nullAll();
  }

  MappingAgent(const MappingAgent &other):
               SLAMParticleOwner(-1,&mon,NULL)
               ,path(location_destructor){
    nullAll();
    set(other);
  }

  const MappingAgent & operator=(const MappingAgent &other){
    if(this != &other){
      destroy();
      set(other);
    }
    return *this;
  }

  void init(int ID, int numP
            , SLAM * slam, const MultiSLAM *mslam);

  //Generates an array of weighted mapping decisions
  //array is sorted in decending order, up to nmax
  //hypothesis is generated, if there are more than nmax possibilities
  //only the fittest nmax will be passed.
  //RETURN VALUE: number of decisions generated.
  int checkPoint(bool *canStay, 
                 struct MappingAgent::MappingDecision *decs_out,
		 int nmax);

  //Take an action specified by the decs.
  void change(struct MappingAgent::MappingDecision decs);

  int getId()const{return id;}
  void setId(int ID){ id = ID;}

  const GlobalMap & getMap()const{ return gmap; }
  bool dumpPath(FILE* f, const char* name);

  //Create clone of itself with new ID, 
  //this duplicates particles in the SLAM filter.
  MappingAgent * clone(int newID);

  //Save results of current mapping.
  void finalise();

  void set(const MappingAgent &other);

  //Resets map to empty, deletes all related particles from the SLAM filter
  void destroy();

  //If in super mode, assumes all particles in the SLAM
  //filter belong to it.
  bool isSuperMode()const{ return superMode;}
  void setSuperMode(bool set){ superMode = set;}

  //Estimate of current location in the current reference frame.
  //  RobotPose getCurrentPose();

  //Compute number of particles and total weight.
  void computeState();

  void dumpCurrentState(FILE * f, const char* varName = "p")const;

  int numParticles()const{ return mon.np;}
  double getWeight()const{ return mon.sumW;}

  int getCurrentMap()const{return currentMap;}

  SimpleMap* getCurrentLocalMap();

  ~MappingAgent(){
     destroy();
  }

  void animationDump(FILE *f,const char* prefix);
 private:

  bool mappingNew;
  bool superMode;

  GlobalMap gmap;
  SLAM *slam;
  const MultiSLAM *multiSLAM;

  class PHist{
  public:
    int n;
    ParticleHistory **h;

    PHist():n(0),h(NULL){;}

    PHist(int sz){
      n = sz;
      h = new ParticleHistory*[sz];
      int i;
      for(i = 0; i < n; ++i) h[i] = new ParticleHistory();
    }

    PHist(const PHist &o);

    void set(ParticleHistory *h, int n);

    ParticleHistory *& operator[](int i){
      return h[i];
    }

    ~PHist(){destroy();}

    PHist* clone();

    void destroy();
  };

  PHist **blocks;
  int n_blocks;
  PHist **hist;
  int sz_hist;


  int nTotalChecks;
  int currentMap;
  int prevMap;

  int n_neighbours;
  Neighbour *neighbours[MAX_NEIGHBOURS];

  MapAreaInterface *regMap;

  LoopCloseResult **matchers;
  int sz_matchers;

  NewMap nextMap;

  RobotPoseCov meanPose;
  MA_Monitor mon;

  int nChecks;

  GenericStack path;

  void updateParticleHistory(SLAMParticle** p, int np);
  void destroyHist();

  void sampleBlock(int map, int* ind, int np);
  void updateCurrentMap();
  void storePath();

  void updateTransition(int mapId);

  MapTransition* makeTransition(const RobotPose &odo0);

  void startNewMap();
  void closeLoop(int mapId);

  void transitToMap(int mapId);

  void resetMatchers();
  SimpleMap* getRecentMap(int scanId);
  void checkLoopClose(bool forceUpdate);

  void initNextMapRegion();

  void initNeighbours();
  void initMappingRegion();

  void LoopClosePostProcess(int map);

  void computeRobotPose();
  bool isRobotInside();

  double computeScore(const MapAreaInterface*a, const RobotPose*p);
  double probIsInside(const RobotPoseCov* robot,
		      const MapTransition* pth,
                      const MapAreaInterface* region);


  void resetScore(){
    int i;
    for(i = 0; i < n_neighbours; ++i) neighbours[i]->score = 0;
  }

  void normaliseScore(){
    int i;
    double sumScore = 0.0;

    for(i = 0; i < n_neighbours; ++i) sumScore += neighbours[i]->score;

    if(sumScore > 1.e-06){
      double scaler = 1.0/sumScore;
      for(i = 0; i < n_neighbours; ++i) neighbours[i]->score *= scaler;
    }

  }


  int getFinalPoses(RobotPose *, int *);
  void evaluatePath(const MapPath *path, const Array2d &pth
		    ,const double *log_w);

  //Adds another decision to the array, keeping
  //the order of decreasing weight.
  //overwrites the one with lowest weight if array is full already.
  static void addDecision(struct MappingAgent::MappingDecision decs,
		   struct MappingAgent::MappingDecision *decs_out,
		   int &ndecs,
		   int nmax);


};


//====================================================================
// Misc. Support functions
//====================================================================
extern bool match_maps(LoopCloseResult *res, MapMatcherInterface *matcher,
		       const SimpleMap *m1, const SimpleMap *m2
		       ,const RobotPoseCov *r2in1
		       ,double MD_MAX);


void estimateTransition(RobotPoseCov *m2in1,
                        const SimpleMap *m1, const SimpleMap *m2,
                        const RobotPose* x0);

extern void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, double *weight, int nsamples,int);


#endif
