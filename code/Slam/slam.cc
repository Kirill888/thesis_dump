#include "slam.h"
#include "mappingProcess.h"

extern MappingProcessInterface *mappingProcess;

//=============================================================================
// SLAM Particle
//=============================================================================

void SLAMParticle::evaluate(double dt,
	                    const ObservationStore &obs_store){


  const RobotPose* robot = odoHist.getLastNode()->getPose_p();

  double w = owner->mapper->update(map, obs_store, 
				   robot,
				   this, owner->monitor)*owner->scaler;
  if(isnan(w)){
    ABORT("SLAMParticle::isNaN");
  }

  weight *= w;

  if(owner->monitor) owner->monitor->observation(this, robot);
}

void SLAMParticle::storeOdo(double dt){
  RobotPose robot(odo);

  if(u != NULL){ 
    u->advance(&robot,dt);
  }


#if STORE_SENSOR_POSE
  RobotPose sensor;

  mappingProcess->robot2sensor(&sensor, &robot);

  odoHist.add(sensor);
#else
  odoHist.add(robot);
#endif

}

void SLAMParticle::set_private(const SLAMParticle *other){
  map = CLONE(other->map);

  odoHist = other->odoHist;
  owner   = other->owner;

  histId  = other->histId;

  ParticleInterface::set(other);
}

void SLAMParticle::destroy(){
  DESTROY(map);
  ParticleInterface::destroy();
}

//=============================================================================
// SLAM
//=============================================================================

const SLAMParticle * SLAM::getBestParticle()const{
  int i;
  int i_best = -1;
  double w_max = -1.0;

  for( i = 0; i < nParticles; ++i ) {
    if (w_max < particles[i]->getWeight() ) {
      w_max = particles[i]->getWeight();
      i_best = i;
    }
  }

  return (SLAMParticle*) particles[i_best];
}

void SLAM::storeOdo(double TimeStamp){
  int i;
  register double dt  = odo->getDT(TimeStamp);

  for( i = 0; i < nParticles; ++i ) {
     ((SLAMParticle*)particles[i])->storeOdo(dt);
  }
}
