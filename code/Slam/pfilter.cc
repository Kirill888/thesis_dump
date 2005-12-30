#include "pfilter.h"
#include "debug.h"

#ifndef DEPEND
#include <string.h>
#endif

#if PFILT_DEBUG_LOG

#define DEBUG_LOG1(format, args...) if(f_debug1 != NULL){\
                                     fprintf(f_debug1, format, ##args ); }

#define DEBUG_LOG2(format, args...) if(f_debug2 != NULL){\
                                     fprintf(f_debug2, format, ##args ); }


bool ParticleFilter::setDebugLog(const char *fbase){
  char buf[1024];

  snprintf(buf,sizeof(buf),"%s.pw",fbase);
  f_debug1 = fopen(buf,"w");

  snprintf(buf,sizeof(buf),"%s.odo",fbase);
  f_debug2 = fopen(buf,"w");

  if(f_debug1 == NULL || f_debug2 == NULL) return false;

  return true;
}

void ParticleFilter::debug_print_poses(){
  if(f_debug2 != NULL && nParticles > 0){
    int i;

    for(i = 0; i < nParticles; ++i){
      fprintf(f_debug2," %E %E %E"
	      ,particles[i]->getPose().x
	      ,particles[i]->getPose().y
	      ,particles[i]->getPose().rot
	      );
    }

    fprintf(f_debug2,"\n");
  }
}


void ParticleFilter::debug_print_weights(){
  if(f_debug1 != NULL && nParticles > 0){
    int i;

    for(i = 0; i < nParticles; ++i){
      fprintf(f_debug1," %E", particles[i]->getWeight());
    }

    fprintf(f_debug1,"\n");
  }
}

#else

#define debug_print_weights()
#define debug_print_poses()
#define DEBUG_LOG1(format, args...)
#define DEBUG_LOG2(format, args...)

#endif

void ParticleFilter::destroy_particles(ParticleInterface **p, int n){
  if(p != NULL){
    int i;

    for(i = 0; i < n; ++i){
      delete p[i];
    }
    delete [] p;
  }

}

void ParticleFilter::destroy_particles(){
  if(particles != NULL){
    destroy_particles(particles,nParticles);
    delete [] cumSumW;

    particles = NULL;
    cumSumW = NULL;
  }
}

void ParticleFilter::resize(int np){
  if(np <= 0){
    destroy_particles();
    nParticles = 0;
    return;
  }

  int i;
  ParticleInterface **pnew = new ParticleInterface*[np];

  //  printf("Resize: %d => %d\n",nParticles,np);

  if(particles != NULL){
    //    printf("Copying old accross\n");
    memcpy(pnew, particles, nParticles*sizeof(ParticleInterface*));
    //    printf("Done.\n");

    if( np < nParticles){ //If shrinking delete particles np_new->np_old
      //printf("Shrinking.\n");

      for(i = np; i < nParticles; ++i) delete particles[i];
    }else{
      //printf("Expanding.\n");

      //Set the rest to NULL, this is just a sanity check, so
      //that we get instant SEGFAULT in case we forget to set
      //particles after resize (which we should).

      memset(pnew + nParticles,0,sizeof(ParticleInterface*)*(np - nParticles));
    }

    delete[] particles;
  }else{
    //    printf("Expanding from [].\n");

    //set all to NULL;
    memset(pnew, 0, sizeof(ParticleInterface*)*np);
  }

  particles = pnew;
  nParticles = np;

  if(cumSumW != NULL) delete[] cumSumW;

  cumSumW = new double[nParticles];
}

//void ParticleFilter::set(int i, ParticleInterface *p){
//  particles[i] = p;
//}


void ParticleFilter::init(const MotionModelInterface *m, 
                         ParticleInterface const **p, int n){

  setOdoModel(m);

  destroy_particles();


  if( n > 0){
    nParticles = n;
    particles = new ParticleInterface*[nParticles];
    cumSumW   = new double[nParticles];

    int i;


    for(i = 0; i < n; ++i){
      particles[i] = p[i]->clone();
    }
    

  }else{
    particles = NULL;
    cumSumW = NULL;
    nParticles = 0;
  }
}

ParticleFilter::ParticleFilter(const ParticleFilter & other){
  if(other.odo != NULL) odo = other.odo->clone();
  else                  odo = NULL;

  nParticles = other.nParticles;
  moved      = other.moved;

  if(nParticles > 0){
    int i;
    particles = new ParticleInterface*[nParticles];
    cumSumW   = new double[nParticles];

    for(i = 0; i < nParticles; ++i){
      particles[i] = other.particles[i]->clone();
    }

  }
}

const ParticleFilter & ParticleFilter::operator=(const ParticleFilter &other){
  if(this == &other) return *this;

  if(odo != NULL) delete odo;
  destroy_particles();

  if(other.odo != NULL) odo = other.odo->clone();
  else                  odo = NULL;

  nParticles = other.nParticles;
  moved      = other.moved;

  if(nParticles > 0){
    int i;
    particles = new ParticleInterface*[nParticles];
    cumSumW   = new double[nParticles];

    for(i = 0; i < nParticles; ++i){
      particles[i] = other.particles[i]->clone();
    }

  }

  return *this;

}

void ParticleFilter::advance(const OdometryReadingInterface *reading){
  int i;

  //  if(odo == NULL) return;

  odo->newOdometry(reading);

  if(odo->hasMoved()){
    for(i = 0; i < nParticles; ++i){
      particles[i]->advance(odo->sampleControlInput());
    }
    moved = true;
  }else{
    moved = false;
    for(i = 0; i < nParticles; ++i){
      particles[i]->stopped();
    }

  }
}

void ParticleFilter::evaluate(double TimeStamp, 
                              const ObservationStore &obs_store){
  register int i;
  register double sum = 0.0;
  register double dt  = odo->getDT(TimeStamp);

  //printf("EVALUATE: %d\n", nParticles);

  for(i = 0; i < nParticles; ++i){
    particles[i]->evaluate(dt, obs_store);

    //printf("...w[%d] = %.3e\n",i, particles[i]->getWeight());
    sum += particles[i]->getWeight();
    cumSumW[i] = sum;
  }

  nObs = obs_store.numObsLastScan();
}

void ParticleFilter::resample(){
  ParticleInterface **tmp = new ParticleInterface*[nParticles];
  int i;
  double sum = cumSumW[nParticles - 1];
  double w0 = 1.0/nParticles;

  UniformRandomNumber dice(0.0, sum);

//   printf("Resample: %e -> %e [%d]\n"
//          ,cumSumW[0], cumSumW[nParticles -1],nParticles);
   
  debug_print_poses();
  debug_print_weights();

  double pos  = dice.nextDoubleRandom();
  double step = sum/nParticles;
  int ind     = 0;

  for(i = 0; i < nParticles; ++i){
    pos += step;
    if(pos > sum){
      pos = pos - sum;
      ind = 0;
    }

    while(pos > cumSumW[ind]){
      ind += 1;
    }

    tmp[i] = particles[ind]->clone();
    tmp[i]->setWeight(w0);
    DEBUG_LOG1(" %d", ind);
  }

  DEBUG_LOG1("\n");


  destroy_particles(particles,nParticles);
  particles = tmp;
}

//Resample with different size.
void ParticleFilter::resample(int newSz){

  if(newSz == nParticles){
    resample();
    return;
  }

  ParticleInterface **tmp = new ParticleInterface*[newSz];
  int i;
  double sum = cumSumW[nParticles - 1];
  double w0 = 1.0/nParticles;

  UniformRandomNumber dice(0.0, sum);

  DEBUG_print("Resample: %e -> %e\n",cumSumW[0], sum);
  DEBUG_print("   ind:");
   
  debug_print_poses();
  debug_print_weights();

  double pos  = dice.nextDoubleRandom();
  double step = sum/nParticles;
  int ind     = 0;

  for(i = 0; i < newSz; ++i){
    pos += step;
    if(pos > sum){
      pos = pos - sum;
      ind = 0;
    }

    while(pos > cumSumW[ind]){
      ind += 1;
    }

    tmp[i] = particles[ind]->clone();
    tmp[i]->setWeight(w0);
    DEBUG_LOG1(" %d", ind);
  }

  DEBUG_LOG1("\n");


  destroy_particles(particles,nParticles);
  particles = tmp;

  nParticles = newSz;
  delete[] cumSumW;
  cumSumW = new double[nParticles];

  DEBUG_print("...done\n");
}

void ParticleFilter::resetWeight(){
  int i;
  double w0 = 1.0/nParticles;
  double sum = 0.0;

  for(i = 0; i < nParticles; ++i){
    particles[i]->setWeight(w0);

    sum += w0;
    cumSumW[i] = sum;
  }
}

void ParticleFilter::normalise(){
  //  printf("PF::normalise, sum = %f, %f\n",sum,1/sum);
  double sum = sumWeight();

  register double scaler = 1.0/sum;

  //If all weights are zero, just ignore it.
  if(isinf(scaler)){ 
    printf("PF::normalise -- SUMW = 0 -- %e %e!!!\n",sum,scaler);
    exit(1);
    return;
  }

  normalise(scaler);
}

void ParticleFilter::normalise(double scaler){
  int i;
  double sum = 0.0;

  for(i = 0; i < nParticles; ++i){
    particles[i]->normaliseWeight(scaler);
    sum += particles[i]->getWeight();
    cumSumW[i] = sum;

    if(isnan(particles[i]->getWeight())){
      printf("PF::normalise_2[%d] isNAN!!!\n",i);
      exit(1);
    }
  }
}

double ParticleFilter::sumWeight(){
  register double sum = 0.0;
  int i;
  for(i = 0; i < nParticles; ++i){
    sum += particles[i]->getWeight();
  }
  return sum;
}

int ParticleFilter::findBestParticle()const{
  register int i;
  register double w_max = -1;
  int ind = -1;

  for(i = 0; i < nParticles; ++i){
    if(particles[i]->getWeight() > w_max){
      ind = i; w_max = particles[i]->getWeight();
    }
  }

  return ind;
}

void ParticleFilter::cleanupDead(){
  ParticleInterface **tmp = new ParticleInterface*[nParticles];
  int n = 0;
  int i;

  for(i = 0; i < nParticles; ++i){
    if(particles[i] != NULL){
      tmp[n] = particles[i];
      n += 1;
    }
  }

  delete[] particles;
  particles = tmp;
  nParticles = n;

  if(nParticles > 0){

  }else{
    delete[] particles;
    delete[] cumSumW;
    particles = NULL;
    cumSumW = NULL;
  }

}
