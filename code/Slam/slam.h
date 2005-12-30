#ifndef __SLAM_H__
#define __SLAM_H__

#include "util.h"
#include "map.h"
#include "DataStore.h"
#include "pfilter.h"

class SLAMParticle;
class SLAM;

class SLAMMonitor{
 public:

//Called by particle filter once before and after processing observation
  virtual void startObservation(const SLAM *f, double TimeStamp,
                                const ObservationStore &obs_store){;}
  virtual void endObservation(const SLAM *f){;}
  virtual void onNormalise(double scaler){;}

//called by individual particles
  //Starting Observation update
  virtual void observation(const SLAMParticle*        , const RobotPose*){;}
  //For every observation that matched
  virtual void observe(const SLAMParticle*, int obsInd
                       ,const MapEntryInterface*){;}
  //Fot every that didn't match
  virtual void noMatch(const SLAMParticle*, int obsInd){;}


  virtual void onResample(const SLAM*f){;}

  //Called When spawning a child
  virtual void onClone(const SLAMParticle* parent, 
                       const SLAMParticle* clone){;}

  virtual ~SLAMMonitor(){;}
};

class SLAMMonitorMultiplex: public SLAMMonitor{
 private:
  struct List{
    SLAMMonitor *data;

    struct List *next;

    List():data(NULL),next(NULL){;}
    List(SLAMMonitor* d, struct List* n):data(d),next(n){;}
  };

  struct List head;

  void destroy(){
    struct List *e;
    e = head.next; 
    while(e != NULL){
      struct List* tmp = e;
      e = e->next;
      delete tmp;
    }
  }

 public:
  //Multiplex functions
  void add(SLAMMonitor *monitor){
    struct List *l = new struct List(monitor,head.next);
    head.next = l;
  }

  void remove(SLAMMonitor *monitor){
    struct List *e;
    struct List *prev = &head;

    for(e = head.next; e != NULL; e = e->next){
      if(e->data == monitor){
	prev->next = e->next;
	delete e;
	return;
      }
      prev = e;
    }
  }

//Utility macros

#define MUX_RUN_COMMAND(action)\
  for(struct List *e = head.next; e != NULL; e = e->next){e->data->action;}

  //Monitor Interface
   void startObservation(const SLAM *f, double TimeStamp,
                         const ObservationStore &obs_store){
     MUX_RUN_COMMAND(startObservation(f,TimeStamp, obs_store));
   }

   void endObservation(const SLAM *f){
     MUX_RUN_COMMAND(endObservation(f));
   }

   void observation(const SLAMParticle *p, const RobotPose*r){
     MUX_RUN_COMMAND(observation(p,r));
   }
   void observe(const SLAMParticle* p, int obsInd, 
                const MapEntryInterface *m){
     MUX_RUN_COMMAND(observe(p,obsInd,m));
   }
   void noMatch(const SLAMParticle* p, int obsInd){
     MUX_RUN_COMMAND(noMatch(p,obsInd));
   }

   void onNormalise(double scaler){
     MUX_RUN_COMMAND(onNormalise(scaler));
   }

   void onResample(const SLAM*f){
     MUX_RUN_COMMAND(onResample(f));
   }

   void onClone(const SLAMParticle* parent, const SLAMParticle* clone){
     MUX_RUN_COMMAND(onClone(parent,clone));
   }
};




class SLAMParticleOwner{
 public:
  int id;
  double scaler;
  SLAMMonitor *monitor;
  MapBuilderInterface* mapper;

  SLAMParticleOwner():id(-1),scaler(1),monitor(NULL),mapper(NULL){;}
  SLAMParticleOwner(int ID, SLAMMonitor* mon, MapBuilderInterface* mapBuilder):
      id(ID),scaler(1),monitor(mon),mapper(mapBuilder){;}
};

class SLAMParticle : public ParticleInterface{
 private:
  SLAMParticleOwner* owner;
  MapInterface     * map;
  OdometryStore odoHist;

  int histId;

 public:

  SLAMParticle():owner(NULL),map(NULL), histId(-1)
  {;} 

  SLAMParticle(SLAMParticleOwner* o):owner(o),map(NULL), histId(-1)
  {
    if(owner != NULL){
      map = owner->mapper->makeEmptyMap();
    }
  }

  SLAMParticle(const SLAMParticle &other){
     set_private(&other);
  }

  SLAMParticle(const SLAMParticle *other){
     set_private(other);
  }

  const SLAMParticle & operator=(const SLAMParticle &other){
    if(this != &other){
      destroy();
      set_private(&other);
    }
    return *this;
  }

  ~SLAMParticle(){
    //if(mapper != NULL) delete mapper;
    destroy();
  }

  void evaluate(double dt,
		const ObservationStore &obs_store);

  ParticleInterface* clone()const{
    SLAMParticle * clon = new SLAMParticle(this);
    if(owner->monitor) owner->monitor->onClone(this, clon);

    return clon;
  }

  const MapInterface* getMap()const{ return map;}

  const OdometryStore& getOdo()const{return odoHist;}

  int getId()const{ return owner->id;}
  void setOwner(SLAMParticleOwner* o){ owner = o;}
  const SLAMParticleOwner* getOwner()const{return owner;}

  void set(const SLAMParticle *other){
    destroy();
    set_private(other);
  }

  void setMap(const SimpleMap *m){
    if(owner != NULL && owner->mapper != NULL){
      DESTROY(map);
      map = owner->mapper->makeMap(m);
    }else{
      fprintf(stderr,"Call setMap with unitialised mapper!\n");
      exit(0);
    }
  }

  void setMap(const MapInterface *m){
    DESTROY(map);
    map = CLONE(m);
  }

  void setEmptyMap(){
    DESTROY(map);
    map = owner->mapper->makeEmptyMap();
  }

  void setOdo(const OdometryStore &odo){
    odoHist = odo;
  }

  void reset(){
    if(owner!=NULL && owner->mapper!=NULL){
      DESTROY(map);
      map = owner->mapper->makeEmptyMap();
    }

    if(u!=NULL) u->reset();

    odoHist.reset();
    odo.set(0.0,0.0,0.0);
  }

  void setHist(int i){ histId = i;}
  int  getHist()const{return histId;}

  void resetPath(){ odoHist.reset(); }
  void storeOdo(double dt);

 private:

  void set_private(const SLAMParticle *other);
  void destroy();
};

class SLAM: public ParticleFilter{
 private:
  SLAMMonitorMultiplex monitor;

 public:
  SLAM(){;}

  SLAM(const SLAM &other):ParticleFilter(other){;}

  const SLAM & operator=(const SLAM &other){
    if(this == &other) return *this;
    ParticleFilter::operator=(other);
    return *this;
  }

  //  ~SLAM(){;}


  void resample(){
    monitor.onResample(this);
    ParticleFilter::resample();
  }

  void resample(int newSz){
    monitor.onResample(this);
    ParticleFilter::resample(newSz);
  }
    

  const SLAMParticle * operator[](int i)const{
      return (const SLAMParticle*) particles[i];
  }

  SLAMParticle ** getParticles()const{
      return (SLAMParticle **)particles;
  }

  const SLAMParticle * getBestParticle()const;

  void storeOdo(double TimeStamp);

  SLAMParticle * makeNewParticle(SLAMParticleOwner* o)const{
    return new SLAMParticle(o);
  }

  void evaluate(double TimeStamp, 
		const ObservationStore &obs_store){
    monitor.startObservation(this,TimeStamp, obs_store);
    ParticleFilter::evaluate(TimeStamp,obs_store);
    monitor.endObservation(this);
  }

  void normalise(){
    double sum = sumWeight();

    if(sum == 0 || isnan(sum)){
      ABORT("Error: Bad Sum in Normalise %e\n", sum);
    }

    double scaler = 1/sum;

    ParticleFilter::normalise(scaler);
    monitor.onNormalise(scaler);
  }

  void addMonitor(SLAMMonitor* m){ monitor.add(m);}
  void removeMonitor(SLAMMonitor* m){monitor.remove(m);}
};


#endif
