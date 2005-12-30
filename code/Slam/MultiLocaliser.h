#ifndef __MULTILOCLALISER_H__
#define __MULTILOCLALISER_H__

#include "LocaliserHype.h"
#include "MappingAgent.h"

class MultiLocaliser{

 private:
  /////////////////////////////////////////////
  //Supplied by the user
  /////////////////////////////////////////////
  const GlobalMap *gmap;
  const LocaliserParticleGuts *defaultLocGuts;

  int NUMP_PER_HYPE;
  int NUMP_SLAM; //For global localisation we use slam


  /////////////////////////////////////////////
  // Types
  /////////////////////////////////////////////
  enum State {Tracking, GlobalLocalise, Lost};


  /////////////////////////////////////////////
  // Private variables
  /////////////////////////////////////////////
  SLAM slam;
  LocaliserFilter filter;

  enum State state;

  int maxHypes;
  int nHypes;
  LocaliserHype **hypes;

  int nScansSinceLastCheck;
  int *isMapOccupied;

  int hypeId;


  /////////////////////////////////////////////
  //Private methods/functions
  /////////////////////////////////////////////

  int getNewHypeId(){ return ++hypeId;}

  void checkPoint();
  void sortHypes();
  void pruneHypes();

  void addHype(const LocaliserHype* parent, int mapId);

  void slamCheckPoint();
  void addHype(int mapId, const LoopCloseResult *);


 public:
  /////////////////////////////////////////////
  // Initialisation and setup
  /////////////////////////////////////////////

  MultiLocaliser():gmap(NULL),defaultLocGuts(NULL)
    ,state(Lost), maxHypes(0),nHypes(0),hypes(NULL)
    ,nScansSinceLastCheck(0), isMapOccupied(NULL), hypeId(0)
  {;}

  void setup( const GlobalMap *m
	     ,const LocaliserParticleGuts *guts
	     ,const MotionModelInterface *odoModel
	     ,int maxHypes
	     ,int numP
	     ,const MapBuilderInterface* mapper
             ,int numP_slam = 300);

  /////////////////////////////////////////////
  // Particle Filter stuff
  /////////////////////////////////////////////
  void observation(double TimeStamp, const ObservationStore &obs0,
                   const int * ind, int n_obs);

  void odometry(const OdometryReadingInterface *odo);

  void init(int map, const RobotPoseSampler *sampler);
  void globalLocalise();

  void finalise(){ pruneHypes(); sortHypes();}

  /////////////////////////////////////////////
  //Access methods
  /////////////////////////////////////////////
  bool hasMoved()const{return filter.hasMoved();}

  LocaliserHype *hype(int i){ return hypes[i];}
  LocaliserHype *BestHype(){ return hypes[0];}


  /////////////////////////////////////////////
  //Misc.
  /////////////////////////////////////////////
  void setSeed(int seed){filter.setSeed(seed);}
  void setDebugLog(char *base){
#if PFILT_DEBUG_LOG
    loc.setDebugLog(base);
#endif
  }


};



#endif

