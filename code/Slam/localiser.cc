#ifndef DEPEND
#include <math.h>
#include <string.h>
#endif

#include "localiser.h"

//this function is in map.cc
extern int findNN(const ObservationInterface *obs0,
           ObservationInterface **map, int nmap,
           double LOG_MAX);

extern void robot2sensor(RobotPose*,const RobotPose*);

//////////////////////////////////////////////////////////////////////
//LocaliserParticleGuts
//////////////////////////////////////////////////////////////////////

int LocaliserParticleGuts::associate(int *corr_out, 
				     double *w_out,
                                     const RobotPose *robot,
				     const ObservationStore &obs_store,
				     int obs_ind){

  //NOTE: very inefficient implementation
  //      inherited classes should overload this method
  int nMatch = 0;
  int nMap = map->numElements();
  int i;

  for(i = 0; i < nMap; ++i){
    double md2 = map->get(i)->mahalanobis2(obs_store[obs_ind],robot);
    if(md2 < MD2_MAX){
      corr_out[nMatch] = i;
      w_out[nMatch]    = map->get(i)->probMatch(obs_store[obs_ind],robot);
      nMatch += 1;
    }
  }

  return nMatch;
}



//////////////////////////////////////////////////////////////////////
//LocaliserParticle
//////////////////////////////////////////////////////////////////////

void LocaliserParticle::evaluate(double dt,
				 const ObservationStore &obs_store){
  //1. Perform data association
  //2. Compute overlap of the map and observation ( Sum(log(w_i)) )
  //3. Compute weight w = exp(Sum)

  int corr[guts->map->numElements()];
  double w_match[guts->map->numElements()];

  RobotPose robot(odo);

  if(u != NULL){
    u->advance(&robot,dt);
  }

  RobotPose sensor;
  robot2sensor(&sensor, &robot);

  double sumW = 0;
  int i;
  int nobs = obs_store.numObsLastScan();

  guts->monitor->observation(this,&robot);

  const ObservationInterface *const* scan = obs_store.getLastScan();

  for(i = 0; i <  nobs;++i){
    int i_obs = obs_store.scan2Ind(i);

    int nCorr = guts->associate(corr,w_match, &sensor,obs_store
                               ,i_obs);
    int j;
    double w = 0;

    w += guts->probNoMatch(scan[i], &sensor);

    for(j = 0; j < nCorr; ++j){
      w += w_match[j];

      guts->monitor->observe(this,i_obs,corr[j]);
    }

    if(nCorr <= 0) guts->monitor->noMatch(this,i_obs);

    sumW += log(w);
  }

  weight *= exp(sumW);
}

void LocaliserParticle::advance(OdoControlInputInterface *u){
  ParticleInterface::advance(u);
  guts->monitor->moved(this);
}

//////////////////////////////////////////////////////////////////////
// Localiser
//////////////////////////////////////////////////////////////////////

void LocaliserFilter::init(int np, const MotionModelInterface *m,
		     LocaliserParticleGuts *guts){

  ParticleFilter::init(m,NULL,0);
  if(np <= 0) return;

  resize(np);

  int i;
  for(i = 0; i < nParticles; ++i){
    particles[i] = new LocaliserParticle(guts);
  }

}

void LocaliserFilter::add(const LocaliserParticle **p, int np){
  int iStart = nParticles;

  resize(nParticles + np);

  memcpy(particles + iStart, p, np);
}

