#ifndef DEPEND
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#endif

#include "localmap.h"
#include "globalmap.h"
#include "matlab.h"
#include "util.h"

const int MIN_OBS_COUNT = 5;

#define ERR(msg) fprintf(stderr,"LMAP: %s\n",msg);
#define REPORT(msg) fprintf(stdout,"LMAP: %s\n", msg);

#define LMAP_LoadMap(m) mappingProcess->loadLocalMap(m)


void LocalMap::setToNull(){
  bestMap   = NULL;  coreMap   = NULL;  mapRegion = NULL; 

  maps = NULL;  nmaps = 0; sz_maps = 0;
  weights = cumsum = NULL;
}

void LocalMap::resize(int sz_new){
  MapInterface ** m2 = new MapInterface*[sz_new];
  double *w2 = new double[sz_new];
  double *cs2 = new double[sz_new];

  if(maps != NULL){
    int ncopy = min(sz_new, nmaps);
    memcpy(m2 , maps   , ncopy*sizeof(MapInterface*));
    memcpy(w2 , weights, ncopy*sizeof(double));
    memcpy(cs2, cumsum , ncopy*sizeof(double));

    delete[] maps;
    delete[] weights;
    delete[] cumsum;
  }

  maps = m2;
  weights = w2;
  cumsum = cs2;
  sz_maps = sz_new;
}

void LocalMap::copyMaps(MapInterface** m, int n){
  int i;

  //Delete old maps
  for(i = 0; i < nmaps; ++i) delete maps[i];

  if(n > sz_maps){
    DESTROY(maps);

    if(n > 0){
      maps = new MapInterface*[n];
      sz_maps  = n;
    }else{
      maps = NULL;
      sz_maps = 0;
    }
  }

  for(i = 0; i < n; ++i){
    maps[i] = m[i]->clone();
  }

  nmaps = n;
}

//NOTE: should call destroy before calling any of the set funcitons,
//      if localmap was not empty.
//
void LocalMap::set(const SLAM *slamFilter){
  set(slamFilter->getParticles(), slamFilter->numParticles());
}

void LocalMap::set(const SLAMParticle *const*slamFilter, int n){
  update(slamFilter,n);
}

void LocalMap::update(const SLAMParticle *const* slamFilter, int n){

  int i;
  double w_max = slamFilter[0]->getWeight();
  int imax = 0;

  //Delete existing maps
  for(i = 0; i < nmaps; ++i) delete maps[i];

  if(n > sz_maps){
    resize(n);
  }

  //Set all weights to equal
  double w0 = 1.0/double(n);
  double wsum = 0;

  //Copy Maps and Find best Particle
  ////////////////////////////
  for(i = 0; i < n; ++i){
    maps[i] = slamFilter[i]->getMap()->clone();
    weights[i] = w0;
    wsum += w0;
    cumsum[i] = wsum;

    if(w_max < slamFilter[i]->getWeight()){
      w_max = slamFilter[i]->getWeight();
      imax = i;
    }
  }

  nmaps = n;

  //Second: Update the bestMap
  /////////////////////////////

  DESTROY(bestMap);
  RangeMature range;
  bestMap = maps[imax]->getSubMap(range);
}


void LocalMap::computeCoreMap(){
  DESTROY(coreMap);

  int i;
  MapEntryInterface **map = bestMap->getMap();
  int ind_core[bestMap->numMap()];
  int n_core = 0;

  for(i = 0; i < bestMap->numMap(); ++i){
    if(map[i]->isCore()){
      ind_core[n_core] = i;
      n_core += 1;
    }
  }

  coreMap = bestMap->subMap(ind_core,n_core);
}

MapInterface* LocalMap::sampleMap(int *out)const{
  if(nmaps <= 0) return NULL;

  UniformRandomNumber dice(0,nmaps);
  int ind = dice.nextIntRandom();

  while(ind >= nmaps) ind = dice.nextIntRandom();

  if(out != NULL) *out = ind;

  return maps[ind];
}


bool LocalMap::matlabDump(FILE *f, char *var_prefix)const{
  bool res;

  //Store Maps
  ////////////
  res = matlabDump_map(f, var_prefix, bestMap);

  return res;
}

bool LocalMap::dumpAll(FILE*f, const char* var)const{
  RangeMinObs range(1);
  SimpleMap *m;
  int i,j;

  fprintf(f,"%s = [...\n",var);
  for(i = 0; i < nmaps; ++i){
    m = maps[i]->getSubMap(range); 

    for(j = 0; j < m->numElements(); ++j){
      m->get(j)->matlabDump(f);
      fprintf(f, " %d\n", i);
    }

    delete m;
  }
  fprintf(f,"];\n");

  return true;
}



bool LocalMap::matlabDump_map(FILE *f, const char* var_prefix
			      , const  SimpleMap *map)const{
  bool res;
  int i;

  res = fprintf(f,"%smap = [ ...\n",var_prefix) > 0;

  for(i = 0 ; i < map->numMap() && res; ++i){ //For every map element
    res = map->get(i)->matlabDump(f);
    res &= fprintf(f,"\n");
  }

  res &= fprintf(f,"]; %%end %smap\n\n", var_prefix) > 0;

  return res;
}


void LocalMap::set(const LocalMap& other){
  //  printf("LocalMap::set( %p <= %p\n", this, &other);

  bestMap   = CLONE(other.bestMap);
  coreMap   = CLONE(other.coreMap);
  mapRegion = CLONE(other.mapRegion);

  copyMaps(other.maps, other.nmaps);

  if(sz_maps > 0){
    weights = new double[sz_maps];
    cumsum  = new double[sz_maps];
    memcpy(weights, other.weights, nmaps*sizeof(double));
    memcpy(cumsum , other.cumsum , nmaps*sizeof(double));
  }
}

void LocalMap::destroy(){
  DESTROY(bestMap);
  DESTROY(coreMap);
  DESTROY(mapRegion);
  DESTROY_ARRAYP(maps,nmaps);
}



bool LocalMap::load(const char *fileName){

  FILE *f = fopen(fileName,"r");

  if(f == NULL){
    ERR("Failed to open matlab file.\n");
    return false;
  }

  bool res = load(f);
  fclose(f);

  return res;
}


bool LocalMap::load(FILE *f){
  MatlabVariable map_;

  // REPORT("Loading Matrices.\n");

  if(map_.read(f)   != MatlabVariable::noError){
    ERR("Failed to read matlab file.");
    return false;
  }

  //REPORT("Checking Sizes.\n");

  destroy(); //Clean up first

  //load maps 
  bestMap = LMAP_LoadMap(map_);
  if(bestMap == NULL) return false;

  computeCoreMap();

  return true;
}
