#ifndef __P_FILTER_H__
#define __P_FILTER_H__

#include "odoModel.h"
#include "DataStore.h"
#include "random.h"
#include "bisearch.h"

#if PFILT_DEBUG_LOG
#include <stdio.h>
#endif

class ParticleInterface{
 protected:
  double weight;

  OdoControlInputInterface* u;
  RobotPose odo;

 protected:
  void destroy(){
    if(u != NULL){ delete u; u = NULL;}
  }

 public:

  ParticleInterface(): weight(1),u(NULL){;}


  //Interface functions
  virtual void advance(OdoControlInputInterface *u){
    if(this->u != NULL) delete this->u;
    this->u = u;

    u->advance(&odo);
  }

  virtual void stopped(){
    if(u != NULL) delete u;

    u = NULL;
  }

  virtual void evaluate(double dt,
			const ObservationStore &obs_store){
    printf("....WARNING...calling dummy virtual function!\n");
  }

  virtual ParticleInterface* clone()const{return NULL;}

  virtual ~ParticleInterface(){destroy();}

  //Access Methods
  double getWeight()const{return weight;}
  void setWeight(double w){ weight = w;}

  void normaliseWeight(double scaler){ weight *= scaler;}

  void setPose(const RobotPose &p){ odo = p;}
  const RobotPose& getPose()const{ return odo;}

  RobotPose* pose(){return &odo;}

  void set(const ParticleInterface* other){
    weight = other->weight;
    odo = other->odo;
    if(other->u != NULL) u = other->u->clone();
    else u = NULL;
  }
};

class ParticleFilter{

 protected:
  int nParticles;
  ParticleInterface **particles;
  double *cumSumW;

  MotionModelInterface *odo;

  void destroy_particles(ParticleInterface **p, int n);
  void destroy_particles();

  int nOdo;
  int nObs;

  bool moved;

#if PFILT_DEBUG_LOG
  FILE *f_debug1;
  FILE *f_debug2;
  void debug_print_weights();
  void debug_print_poses();

#define PFILT_DEBUG_LOG_INIT() { f_debug1 = f_debug2 = NULL;}
#define PFILT_DEBUG_LOG_DESTROY() if(f_debug1 != NULL){fclose(f_debug1);} \
				  if(f_debug2 != NULL){fclose(f_debug2);}

#else

#define PFILT_DEBUG_LOG_INIT() 
#define PFILT_DEBUG_LOG_DESTROY() 

#endif

 public:

  ParticleFilter(){
    nParticles = 0;
    particles = NULL;
    cumSumW   = NULL;
    odo       = NULL;
    nOdo      = 0;
    nObs      = 0;
    moved     = false;

    PFILT_DEBUG_LOG_INIT();

  }

  ParticleFilter(const MotionModelInterface *odoModel,
		 ParticleInterface const **p, int n){
    nParticles = 0;
    particles = NULL;
    odo       = NULL;
    cumSumW   = NULL;
    nOdo      = 0;
    nObs      = 0;
    moved     = false;

    PFILT_DEBUG_LOG_INIT();
    init(odoModel, p, n);
  }

  ParticleFilter(const ParticleFilter & other);
  const ParticleFilter & operator=(const ParticleFilter &other);


  ~ParticleFilter(){
    if(odo != NULL) delete odo;
    destroy_particles();

    PFILT_DEBUG_LOG_DESTROY();
  }

  void setOdoModel(const MotionModelInterface *m){
    if(odo == NULL) delete odo;
    if(m != NULL) odo = m->clone();
    else          odo = NULL;
  }

  void init(const MotionModelInterface *m, 
	    ParticleInterface const **p, 
	    int n);

  void advance(const OdometryReadingInterface *);

  void evaluate(double TimeStamp, 
		const ObservationStore &obs_store);

  void resample();
  void resample(int newSz);
  void normalise();
  void normalise(double scaler);
  double sumWeight();

  void setSeed(int seed){srand(seed);}

  int numParticles()const{return nParticles;}
  ParticleInterface *get(int i){return particles[i];}
  void setp(int i, ParticleInterface *p){ particles[i] = p;}
  ParticleInterface*& operator[](int i){ return particles[i];}

  void resize(int np);

  void resetWeight();


  bool hasMoved()const{ return moved;}

  void cleanupDead();

  int findBestParticle()const;

#if PFILT_DEBUG_LOG
  bool setDebugLog(const char *fbase);
#endif

  const Odometer& odometer()const{return odo->odometer();}
  int numObs()const{ return nObs;}
};


#endif
