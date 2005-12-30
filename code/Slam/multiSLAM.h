#ifndef __MULTI_SLAM_H__
#define __MULTI_SLAM_H__

#include "slam.h"
#include "MappingAgent.h"

//const int MAX_AGENTS = 5;


class MultiSLAM{
 private:

  class Hype{
  public:
    MappingAgent* hype;
    Hype* parent;       // NULL if it's dead
    int *childHypes;    // size ---> n_maps + 1
    int nChildren;
    bool dead;
    bool fresh; //For animation purposes

    Hype():hype(NULL),parent(NULL),childHypes(NULL),nChildren(0)
          ,dead(false),fresh(true){;}

    Hype(MappingAgent* h, Hype* p);


    ~Hype(){
      kill();
      DESTROY_ARRAY(childHypes);
     }

    void kill();
    void unregister();
    void change(const MappingAgent::MappingDecision dec);
    void resetChildren();


  };

 public:
  class ProposedHype{
  public:
    Hype* source;
    double weight;
    MappingAgent::MappingDecision decision;
    bool orphan;

    ProposedHype():source(NULL),weight(0){;}

    ProposedHype(Hype *s, double w, MappingAgent::MappingDecision dec
                 ,bool Orphan):
      source(s),weight(w),decision(dec),orphan(Orphan){;}

  };

 private:

  SLAM slam;
  const SensorRangeInterface* sensorRange;
  const MapBuilderInterface* mapper;

  Hype *bestHype;
  Hype **hypes;
  int n_hypes;

  int n_obs;
  double lastOdo;
  int lastNobs;
  int minorCheckOdo;

  int _nextAgentId;

  int n_odo;
  int sz_odo;
  int *odo2scan;

  void cleanupDeadAgents();
  void cleanupDeadHypes();
  void unregisterHype(Hype*);
  void minorCheckPoint();
  void checkPoint(bool *);

  void init(const MotionModelInterface *robot,
            const MapBuilderInterface *mapper);

  void addHype(ProposedHype* hype);
  int nextAgentId(){ _nextAgentId = (_nextAgentId+1)% 10000; 
      return _nextAgentId;}

 public:

  double CHECK_DIST;
  int CHECK_NOBS;
  int MAX_HYPE_PER_AGENT;
  int NUMP_PER_AGENT;
  int RESAMPLE_STEP;
  int MAX_AGENTS;

  MultiSLAM():_nextAgentId(0),n_odo(0),sz_odo(0),odo2scan(NULL)
              { init(NULL,NULL); }
  MultiSLAM(const MotionModelInterface *robot,
            const MapBuilderInterface *mapper);

  ~MultiSLAM();

  void start();
  void start(const MotionModelInterface *robot,
             const MapBuilderInterface *mapper);

  void observation(double TimeStamp, const ObservationStore &obs0,
                   const int * ind, int n_obs);
  void odometry(const OdometryReadingInterface *odo);
  void storeOdo(double T, int ScanId); 
  bool dumpOdo2Scan(FILE*f, const char* name);

  void finalise();

  void animationDump(FILE*f)const;

  //Inlines
  bool hasMoved()const{return slam.hasMoved();}

  MappingAgent * agent(int i){return hypes[i]->hype;}
  MappingAgent * BestAgent() {return bestHype->hype;}
  int numAgents()const{return n_hypes;}

  void setSeed(int seed){slam.setSeed(seed);}

  const int* getOdo2Scan(int* n_odo)const{ 
     *n_odo = this->n_odo; return odo2scan;
  }

  const SensorRangeInterface* getSensorRange()const{return sensorRange;}
  const MapBuilderInterface* getMapBuilder()const{ return mapper;}

  void setDebugLog(char *base){
#if PFILT_DEBUG_LOG
    slam.setDebugLog(base);
#endif
  }

};


#endif
