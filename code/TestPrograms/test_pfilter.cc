#ifndef DEPEND
#include <stdio.h>
#include <time.h>
#endif

#include "odoModel.h"
#include "pfilter.h"

class DummyParticle: public ParticleInterface{
   private:


   public:
     int id;

     DummyParticle(){
       id = 0;
       weight = 1.0;
     }

     DummyParticle(int ID, double W){
       id = ID;
       weight = W;
     }

     DummyParticle(const DummyParticle &other){
        id = other.id;
	weight = other.weight;
     }

     void advance(const OdoControlInput &u){
        printf("p_%d -> advance\n",id);
	ParticleInterface::advance(u);
     }

     void evaluate(double TimeStamp,
                   const ObservationStore &obs_store,
                   int *obs_ind, int nobs){

        printf("p_%d -> evaluate\n",id);
     }

     ParticleInterface* makeCopy()const{
        ParticleInterface *copy =  new DummyParticle(id,weight);
        printf("p_%d -> makeCopy(%p)\n",id,copy);

	return copy;
     }

     void set(int ID, double W){
        id = ID;
	weight = W;
     }

     virtual ~DummyParticle(){;}
};


int main(){

  const int NP = 100;

  MotionModel odoModel;
  ObservationStore obs;

  int i;

  DummyParticle p[NP];
  const ParticleInterface *pp[NP];

  for(i = 0; i < NP; ++i){
     p[i].set(i,0.1*i+1);
     pp[i] = &p[i];
  }

  ParticleFilter pf;

  pf.setSeed(time(NULL));
  pf.set(odoModel,pp,NP);

  const int NRUNS = 100;

  for(i = 0; i < NRUNS; ++i){
    pf.advance(i, RobotPose(0,0,0));

    pf.evaluate(i,obs,NULL, 0);
  }

  return 0;
}
