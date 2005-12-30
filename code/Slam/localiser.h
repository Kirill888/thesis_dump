#ifndef __LOCALISER_H__
#define __LOCALISER_H__

#include "map.h"
#include "pfilter.h"

class LocaliserParticle;
class LocaliserFilter;

//////////////////////////////////////////////////////////////////////
//LocaliserMonitor
//////////////////////////////////////////////////////////////////////
class LocaliserMonitor{

 public:
//Called by particle filter once before and after processing observation
  virtual void startObservation(const LocaliserFilter *f,
                                const ObservationStore &obs_store){;}
  virtual void endObservation(const LocaliserFilter *f){;}

//Called by particle filter once before and after processing odometry
  virtual void startOdo(const LocaliserFilter *f,
                        const OdometryReadingInterface *reading){;}
  virtual void endOdo(const LocaliserFilter *f){;}

  virtual void odoNoMove(const LocaliserFilter *f){;}


//Called by individual particles

  //Starting observation update
  virtual void observation(const LocaliserParticle *p, const RobotPose*r){;}

  //Matched obs => map element
  virtual void observe(const LocaliserParticle* p, int obsInd, int mapInd){;}

  //No match for an observation
  virtual void noMatch(const LocaliserParticle* p, int obsInd){;}

  virtual void moved(const LocaliserParticle* p){;}

//Other stuff
  virtual ~LocaliserMonitor(){;}
};



//////////////////////////////////////////////////////////////////////
//LocaliserParticleGuts
//////////////////////////////////////////////////////////////////////

class LocaliserParticleGuts{
 public:
  int id;
  const SimpleMap   *map;
  LocaliserMonitor* monitor;

  double MD2_MAX;

  LocaliserParticleGuts():id(-1),map(NULL),monitor(NULL),MD2_MAX(POW2(3))
  {;}

  LocaliserParticleGuts(int ID, const SimpleMap*m, LocaliserMonitor *mon
                        ,double max = POW2(3)):
        id(ID),map(m),monitor(mon),MD2_MAX(max){;}


  virtual LocaliserParticleGuts * clone()const{ 
       return new LocaliserParticleGuts(id,map,monitor,MD2_MAX);}

  virtual int associate(int *corr_out, double *w_out, 
                        const RobotPose *p,
                        const ObservationStore &obs_store,
  		        const int obs_ind);

  //Probability of the observation coming from some other source,
  //not present in the map = Prob(Landmark is not in the map) +
  //                         Prob(Spurious reading)
  //
  virtual double probNoMatchLog(const ObservationInterface *obs,
			        const RobotPose* robot){
    return log(0.001);
  }

  virtual double probNoMatch(const ObservationInterface *obs,
			     const RobotPose* robot){
    return 0.001;
  }

  virtual ~LocaliserParticleGuts(){;}
};

//////////////////////////////////////////////////////////////////////
//LocaliserParticle
//////////////////////////////////////////////////////////////////////
class LocaliserParticle: public ParticleInterface{
 private:

  void destroy();
  void set(const LocaliserParticle *other);

 public:
  LocaliserParticleGuts *guts;

  //Constructors
  LocaliserParticle():guts(NULL){;}
  LocaliserParticle(LocaliserParticleGuts* g):guts(g){;}

  LocaliserParticle(const LocaliserParticle &other){
    set(&other);
  }
  LocaliserParticle(const LocaliserParticle *other){
    set(other);
  }

  LocaliserParticle & operator=(const LocaliserParticle &rhs){
    if(this != &rhs ){
      destroy();
      set(&rhs);
    }
    return *this;
  }

  ~LocaliserParticle(){ destroy();}

  //ParticleInterface Functions
  ParticleInterface* clone()const{ return new LocaliserParticle(this); }

  void evaluate(double dt,
		const ObservationStore &obs_store);
  void advance(OdoControlInputInterface *u);

  //Access functions
  int getId()const{ return guts->id;}

};



//////////////////////////////////////////////////////////////////////
//LocaliserMonitorMultiplex
//////////////////////////////////////////////////////////////////////

class LocaliserMonitorMultiplex: public LocaliserMonitor{
 private:
  struct List{
    LocaliserMonitor *data;

    struct List *next;

    List():data(NULL),next(NULL){;}
    List(LocaliserMonitor* d, struct List* n):data(d),next(n){;}
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
  void add(LocaliserMonitor *monitor){
    struct List *l = new struct List(monitor,head.next);
    head.next = l;
  }

  void remove(LocaliserMonitor *monitor){
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
   void startObservation(const LocaliserFilter *f,
                         const ObservationStore &obs_store){
     MUX_RUN_COMMAND(startObservation(f,obs_store));
   }

   void endObservation(const LocaliserFilter *f){
     MUX_RUN_COMMAND(endObservation(f));
   }
   void startOdo(const LocaliserFilter *f,
                 const OdometryReadingInterface *reading){
     MUX_RUN_COMMAND(startOdo(f,reading));
   }
   void endOdo(const LocaliserFilter *f){
     MUX_RUN_COMMAND(endOdo(f));
   }
   void odoNoMove(const LocaliserFilter *f){
     MUX_RUN_COMMAND(odoNoMove(f));
   }
   void observation(const LocaliserParticle *p, const RobotPose*r){
     MUX_RUN_COMMAND(observation(p,r));
   }
   void observe(const LocaliserParticle* p, int obsInd, int mapInd){
     MUX_RUN_COMMAND(observe(p,obsInd,mapInd));
   }
   void noMatch(const LocaliserParticle* p, int obsInd){
     MUX_RUN_COMMAND(noMatch(p,obsInd));
   }
   void moved(const LocaliserParticle* p){
     MUX_RUN_COMMAND(moved(p));
   }
};


//////////////////////////////////////////////////////////////////////
//LocaliserFilter
//////////////////////////////////////////////////////////////////////
class LocaliserFilter: public ParticleFilter{
 private:
  LocaliserMonitorMultiplex monitor;

 public:
  LocaliserFilter(){;}

  void init(int np, const MotionModelInterface *m,
            LocaliserParticleGuts *guts);

  void add(const LocaliserParticle **particles, int np);

  void advance(const OdometryReadingInterface *reading){
    int i;

    odo->newOdometry(reading);


    if(odo->hasMoved()){
      monitor.startOdo(this,reading);
      for(i = 0; i < nParticles; ++i){
	particles[i]->advance(odo->sampleControlInput());
      }
      moved = true;
      monitor.endOdo(this);
    }else{
      monitor.odoNoMove(this);

      for(i = 0; i < nParticles; ++i){
	particles[i]->stopped();
      }
    }

  }

  void evaluate(double TimeStamp, 
		const ObservationStore &obs_store){
    monitor.startObservation(this,obs_store);
    ParticleFilter::evaluate(TimeStamp,obs_store);
    monitor.endObservation(this);
  }

  void setParticle(int i, 
                   const RobotPose &x, LocaliserParticleGuts *guts){
    particles[i]->setPose(x);
    ((LocaliserParticle*)particles[i])->guts = guts;
  }

  void setPose(int i, const RobotPose &x){
    particles[i]->setPose(x);
  }

  void addMonitor(LocaliserMonitor*m){     monitor.add(m);}
  void removeMonitor(LocaliserMonitor *m){ monitor.remove(m);}


};






//////////////////////////////////////////////////////////////////////
//Inlines
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//LocaliserParticle
//////////////////////////////////////////////////////////////////////

inline void LocaliserParticle::set(const LocaliserParticle *other){
  guts = other->guts;

  ParticleInterface::set(other);
}


inline void LocaliserParticle::destroy(){
  ParticleInterface::destroy();
}

//////////////////////////////////////////////////////////////////////
//LocaliserFilter
//////////////////////////////////////////////////////////////////////


#endif

