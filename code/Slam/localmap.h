#ifndef __LOCAL_MAP_H__
#define __LOCAL_MAP_H__

#ifndef DEPEND
#include <string.h>
#include <stdio.h>
#endif

#include "map.h"
#include "slam.h"
#include "matlab.h"
#include "mapMatch.h" 
#include "bisearch.h"
#include "mappingProcess.h"

extern MappingProcessInterface *mappingProcess;

class LocalMap{

 public:

  LocalMap(){
    setToNull();
  }

  LocalMap(const LocalMap &other){
    setToNull();
    set(other);
  }

  const LocalMap & operator=(const LocalMap &other){
    if(this != &other){
      destroy();
      set(other);
    }

    return *this;
  }

  ~LocalMap(){ destroy();}

  void set(const SLAM *slamFilter);
  void set(const SLAMParticle *const*p, int n);
  
  void update(const SLAMParticle *const*slamFilter, int n);

  const SimpleMap* getMap()const{ return bestMap;}
  operator const SimpleMap&()const{ return *bestMap;}

  const SimpleMap* getCoreMap()const{ return coreMap;}

  MapInterface* sampleMap(int *ind = NULL)const;

  void sample(int *ind, int nsample)const{
    sample_cumsum(ind, cumsum, nmaps, nsample);
  }

  const MapInterface* getMap(int i)const{ return maps[i]; }
  int numMaps()const{ return nmaps; }

  void setLandmarkState(int i, int st){
    bestMap->get(i)->setState(st);
  }

  void setLandmarkId(int i, int id){
    bestMap->get(i)->setGlobalId(id);
  }

/*   void getSuitableCoreLandmarks(int *core)const{ */
/*     mappingProcess->getSuitableCoreLandmarks(core, bestMap); */
/*   } */

  void computeCoreMap();

  MapAreaInterface *getRegion()const{ return mapRegion;}
  void setMapRegion(MapAreaInterface* reg){ mapRegion = reg;}

  //Storage/Retrieval functions

  bool load(const char *fileName);
  bool load(FILE *f);

  bool store(FILE *f,char *var_prefix = "")const{
     return matlabDump(f,var_prefix);
  }
  bool matlabDump(FILE *f,char *var_prefix)const;

  bool dumpAll(FILE*f, const char* var)const;

 private:

  //Map and region
  SimpleMap *bestMap;
  SimpleMap *coreMap;
  MapAreaInterface *mapRegion;

  MapInterface **maps;
  double*        weights;
  double*        cumsum;
  int            nmaps;
  int            sz_maps;

  //Private methods
  void destroy();
  void setToNull();

  void set(const LocalMap &other);
  void resize(int sz_new);

  void copyMaps(MapInterface** m, int n);

  //Write/Read functions
  bool matlabDump_passes(FILE* f, const char* var_prefix
		      , const OdometryStore *const *p, int np)const;

  bool matlabDump_map(FILE *f, const char* var_prefix
		      , const  SimpleMap *map)const;
};


#endif

