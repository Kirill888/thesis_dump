#include "LocaliserHype.h"


#define REPORT(format, args...) printf(format, ##args)

#define FOR_EACH(i,p,np, doit) for(i = 0; i < np; ++i){ \
     p = (LocaliserParticle*) loc->get(i);\
     if(p->guts == guts){ doit; }}


void LocaliserHype::init(int mapInd
                        , const RobotPoseSampler *sampler, int numP){

  map.setReferenceFrame(mapInd);
  currentMap = mapInd;

  guts->map = map[mapInd].getMap();
  monitor.map = guts->map;

  int np = loc->numParticles();
  int i;

  loc->resize(np + numP);

  for(i = 0; i < numP; ++i){
    LocaliserParticle *p = new LocaliserParticle(guts);
    p->setPose(sampler->sample());

    loc->setp(np + i, p);
  }

  initNeighbours();
  resetScore();
}

void LocaliserHype::transfer(int mapInd){
  const MapPath* pth = map.getPath(mapInd);
  LocaliserParticle *p;
  int i;

  //Transfer all particles
  FOR_EACH(i,p,loc->numParticles(),
  { 
    pth->transit(p->pose());
  });

  //Set new reference frame
  map.setReferenceFrame(mapInd);
  currentMap = mapInd;
  guts->map = map[mapInd].getMap();
  monitor.map = guts->map;

  initNeighbours();
  resetScore();
}

void LocaliserHype::destroy(){
  LocaliserParticle *p;
  int i;

  FOR_EACH(i,p,loc->numParticles(),
  { 
    delete p;
    loc->setp(i,NULL);
  });

  loc->cleanupDead();
  loc->removeMonitor(&monitor);
}

bool LocaliserHype::isValidNeighbour(int mapId){
  return map.isNeighbour(mapId);
}

void LocaliserHype::initNeighbours(){
  int i;

  //First clean old ones.
  for(i = 0; i < n_neighbours; ++i) delete neighbours[i];

  n_neighbours = 0;

  for(i = 0; i < map.numMaps(); ++i){
    if(isValidNeighbour(i)){
      MapAreaInterface *area = map[i].getRegion()->clone();

      area->translateMe(map.getMapRef(i));

      neighbours[n_neighbours] = new Neighbour(i,area);
      n_neighbours += 1;
    }
  }

}

void LocaliserHype::computeState(){
  int i;

  double np = monitor.np;
  int nCore = 0;
  int nPeripheral = 0;
  int nMissing = 0;

  REPORT("ComputeState:\n");

  if(!isnan(monitor.mean.x)){
    path.add(new LocationCov(currentMap, monitor.mean));
  }

  for( i = 0; i < monitor.nObs; ++i){
    double obsCore       = monitor.obsMatch[i][0]/np;
    double obsPeripheral = monitor.obsMatch[i][1]/np;
    double obsMissing    = monitor.obsMatch[i][2]/np;

    if(obsCore > 0.7){
      REPORT(" obs[%d] -- core\n", i);

      nCore += 1;

    }else if( obsPeripheral > 0.7){
      REPORT(" obs[%d] -- peripheral\n", i);

      nPeripheral += 1;

    }else if(obsMissing > 0.9){
      REPORT(" obs[%d] -- missing\n", i);

      nMissing += 1;

    }else{
      REPORT(" obs[%d] -- unclassified\n", i);
    }
  }

  stayScore += nCore;
  stayScore += 0.1*nPeripheral;

  for(i = 0; i < n_neighbours; ++i){
    double w = computeScore(neighbours[i]->area);
    neighbours[i]->score += w;
  }


}

double LocaliserHype::computeScore(const MapAreaInterface* a){
  int i;
  double w = 0;

  RobotPose p(monitor.mean);

  for(i = 0; i < monitor.nObs; ++i){
    const ObservationInterface * o = monitor.getObs(i);

    w += a->probIsInside(o,&p);
  }

  return w;
}

void addSorted(int *d_out, double *w_out, int n, int d, double w){
  int i;
  for(i = 0; i < n; ++i){
    if(w_out[i] < w){
      memmove(d_out + i + 1, d_out + i, (n-i)*sizeof(int));
      memmove(w_out + i + 1, w_out + i, (n-i)*sizeof(double));

      d_out[i] = d; w_out[i] = w;
      return;
    }
  }

  d_out[n] = d; w_out[n] = w;
}

int LocaliserHype::checkPoint(int *maps_out, double *w_out){
  int n = 0;
  int i;

  normaliseScore();

  if(stayScore > 0){
    maps_out[n] = -1;
    w_out[n] = stayScore;
    n += 1;
  }

  for(i = 0; i < n_neighbours; ++i){
    if(neighbours[i]->score > 0.1){
      addSorted(maps_out, w_out, n
		,neighbours[i]->mapId
		,neighbours[i]->score
		);
      n += 1;
    }
  }

  resetScore();
  return n;
}


LocaliserHype* LocaliserHype::clone(int newId)const{
  LocaliserHype *copy = new LocaliserHype(this);

  copy->guts->id = newId;

  //Copy particles
  if(loc != NULL){
    int np = loc->numParticles();
    LocaliserParticle* p;

    int i;
    int n = 0;

    loc->resize(np + monitor.np); 

    FOR_EACH(i,p,np,
    {
      LocaliserParticle *p_new = (LocaliserParticle*) p->clone();
      p_new->guts = copy->guts;

      loc->setp(np + n, p_new);
      n += 1;
    });


  }

  return copy;
}

void LocaliserHype::dumpToFile(FILE *f){
    LocationCov *out[path.numElements()];
    path.getAll((GenericStackType*)out);
    int i;

    for(i = 0; i < path.numElements(); ++i){
      const RobotPoseCov &p = out[i]->pose;
      fprintf(f,"%3d %+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e "
                "%+.9e %+.9e %+.9e\n"
             , out[i]->mapId, p.x, p.y, p.rot
	     , p.cov[0][0], p.cov[0][1]  ,p.cov[0][2]
	     , p.cov[1][0], p.cov[1][1]  ,p.cov[1][2]
	     , p.cov[2][0], p.cov[2][1]  ,p.cov[2][2] );
   }

}
