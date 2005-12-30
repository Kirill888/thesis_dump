#ifndef DEPEND
#include <string.h>
#include <stdlib.h>
#endif

#include "MappingAgent.h"
#include "MultiLocaliser.h"

#define REPORT(format, args...) printf(format, ##args); fflush(stdout)

const int NSCANS_BETWEEN_CHECK_POINTS = 5;


void MultiLocaliser::setup(const GlobalMap *m
			  ,const LocaliserParticleGuts *guts
			  ,const MotionModelInterface *odoModel
			  ,int maxHypes
			  ,int numP
 	                  ,const MapBuilderInterface* mapper
                          ,int npSLAM){

  gmap = m;
  defaultLocGuts = guts;
  filter.setOdoModel(odoModel);

  this->maxHypes = maxHypes;
  hypes    = new LocaliserHype*[maxHypes];
  nHypes  = 0;

  NUMP_PER_HYPE = numP;

  DESTROY_ARRAY(isMapOccupied);
  isMapOccupied = new int[gmap->numMaps()];
  memset(isMapOccupied,0,sizeof(int)*gmap->numMaps());

  //Init slam filter
  //  slam.setMapBuilder(mapper);
  slam.setOdoModel(odoModel); 
  NUMP_SLAM = npSLAM;
}


void MultiLocaliser::init(int map, const RobotPoseSampler *sampler){
  hypes[0] = new LocaliserHype(map, &filter, *gmap, defaultLocGuts);
  nHypes = 1;

  hypes[0]->init(map, sampler, NUMP_PER_HYPE);

  isMapOccupied[map] = 1;

  state = Tracking;
}

void MultiLocaliser::globalLocalise(){
  REPORT("Global Localistion is not supported.\n");
  exit(3);
}


static int compare_hypes(const LocaliserHype* l1, const LocaliserHype* l2){
  if(l1->weight() < l2->weight())       return -1;
  else if(l1->weight() == l2->weight()) return  0;
  else                                  return  1;
}

inline void MultiLocaliser::sortHypes(){
  if(nHypes <= 1) return;

  qsort(hypes,nHypes,sizeof(LocaliserHype*),
	(int (*)(const void*, const void*))compare_hypes);
}

void MultiLocaliser::pruneHypes(){
  int i;
  int n = 0;
 
  //First compute state
  for(i = 0; i < nHypes; ++i){ hypes[i]->computeState(); }

  //No need to prune if there is only one left
  if(nHypes <= 1) return;

  REPORT("Prune:\n");

  //Then prune
  for(i = 0; i < nHypes; ++i){
    REPORT("  hype[%d] -- %d (%d %.3f)", i
           , hypes[i]->getCurrentMap()
           , hypes[i]->numParticles(), hypes[i]->weight());

    //Keep it if there are particles left
    if(hypes[i]->numParticles() > 0 // && hypes[i]->weight() > 0.1
       ){

      hypes[n] = hypes[i];

      n += 1;
      REPORT(" [ok]\n");
    }else{
      REPORT(" [removed]\n");
      isMapOccupied[hypes[i]->getCurrentMap()] = 0;
      hypes[i]->destroy();
      delete hypes[i];
    }
  }

  //New size and new bestHype
  nHypes = n;
}

inline void MultiLocaliser::addHype(const LocaliserHype* parent, int mapId){
  //  REPORT("Add new: %d", mapId);
  //  REPORT("  clone->");
  hypes[nHypes] = parent->clone(mapId);

  //  REPORT("  transfer->");
  hypes[nHypes]->transfer(mapId);
  //  REPORT("... done\n");

  isMapOccupied[mapId] = 1;

  nHypes += 1;
}


void MultiLocaliser::checkPoint(){
  int    maps[nHypes][gmap->numMaps()];
  double w[nHypes][gmap->numMaps()];
  int n[nHypes];
  int i;
  bool resampleOk = true;

  sortHypes();

  for(i = 0; i < nHypes; ++i){
    n[i] = hypes[i]->checkPoint(maps[i],w[i]);
  }

  REPORT("Check Point: %d\n", nHypes);
  int nh_old = nHypes;

  for(i = 0; i < nh_old; ++i){
    int j;
    LocaliserHype * hype = hypes[i];

    REPORT("Hype[%d]: w=%.3f np = %d n_opt=%d "
           ,i,hype->weight(),hype->numParticles(), n[i]);

    for(j = 0; j < n[i]; ++j){
      int map = maps[i][j];

      if(map > gmap->numMaps()){
	REPORT("ERR: map %d does not exist.\n", map);
	exit(4);
      }

      if(map > -1){ //Ignore staying request, that is granted by default
	if(isMapOccupied[map]){
	  REPORT("[occupied %d] ", map);
	}else{
	  REPORT("[transfer => %d]  ", map);

	  addHype(hype, map);
	  resampleOk = false;
	}
      }

      if(nHypes >= maxHypes) break;
	
    }

    if(n[i] == 0){
      REPORT("[lost]");
    }

    REPORT("\n");
    if(nHypes >= maxHypes) break;

  }

  if(resampleOk){
    filter.resample(); filter.normalise();
  }
  
}

const int NP_TRANSITION = 300;

void MultiLocaliser::addHype(int mapId, const LoopCloseResult * res){
  REPORT("  Adding Hype: %d\n", mapId);
  RobotPose trans[NP_TRANSITION];
  double w[NP_TRANSITION];
  RobotPoseCov m2in1;

  //  estimateTransition(&m2in1, res->map1, res->map2, &res->r2in1);
  sampleTransition( res->map1, res->map2, &res->r2in1, trans, 
                    w, NP_TRANSITION,0);
  MapTransition tr(-1, mapId, trans, w, NP_TRANSITION);

  //  int i;



}


void MultiLocaliser::slamCheckPoint(){
#if 0
  int nlandmarks = slam.getBestParticle()->getMap()->numMature();

  REPORT("SLAMCheckPoint[%d]: ",nlandmarks);

  if(nlandmarks >= 5){
    LocalMap lmap1;
    lmap1.set(&slam);
    MapMatcher matcher;
    LoopCloseResult res;

    int i;
    REPORT("Matching:\n");
    for(i = 0; i < gmap->numMaps(); ++i){
      const LocalMap &lmap2 = gmap->get(i);
      REPORT("   => %d (%d x %d)  ", i+1
             , lmap1.getMap()->numElements()
	     , lmap2.getMap()->numElements());
      if(match_maps(&res, &matcher, lmap1.getMap(), lmap2.getMap()
                        , NULL, 0.0)){
	REPORT("[matched]\n");

	addHype(i, &res);
      }else{
	REPORT("[failed]\n");
      }
    }
  }

  REPORT("\n");
  slam.resample();
#endif
}

void MultiLocaliser::observation(double TimeStamp, 
                                 const ObservationStore &obs0,
				 const int * ind, int n_obs){

  if(slam.numParticles() > 0){
    REPORT("Doing SLAM(%d)\n", slam.numParticles());
    slam.evaluate(TimeStamp, obs0);
    slam.normalise();
    slamCheckPoint();
  }

  if(nHypes > 0){
    filter.evaluate(TimeStamp, obs0);
    filter.normalise();

    nScansSinceLastCheck += 1;

    pruneHypes();

    if(nScansSinceLastCheck >= NSCANS_BETWEEN_CHECK_POINTS){
      checkPoint();
      nScansSinceLastCheck = 0;
    }else{
      filter.resample(nHypes*NUMP_PER_HYPE);
      filter.normalise();
    }
  }
}

void MultiLocaliser::odometry(const OdometryReadingInterface *odo){
  filter.advance(odo);
  slam.advance(odo);
}
