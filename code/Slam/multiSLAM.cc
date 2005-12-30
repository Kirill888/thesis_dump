#ifndef DEPEND
#include <stdlib.h>
#endif

#include "multiSLAM.h"
#include "timeLog.h"

#define REPORT(format, args...) printf(format, ##args)


MultiSLAM::Hype::Hype(MappingAgent* h, Hype* p):hype(h),parent(p)
                     ,dead(false),fresh(true){
  nChildren = 0;

  int n_map = hype->getMap().numMaps();
  childHypes = new int[n_map + 10];
  memset(childHypes, 0, (n_map+10)*sizeof(int));

  //Register with parent
  if(parent != NULL){
    parent->nChildren += 1;
    parent->childHypes[hype->getCurrentMap()] += 1;
  }
}

void MultiSLAM::Hype::kill(){
  dead = true;
  unregister();

  if(hype != NULL){
    delete hype;
    hype = NULL;
  }
}

void MultiSLAM::Hype::unregister(){
  //Unregister with the parent
  if(parent != NULL){
    parent->nChildren -= 1;
    parent->childHypes[hype->getCurrentMap()] -= 1;
    parent = NULL;
  }
}

void MultiSLAM::Hype::change(const MappingAgent::MappingDecision dec){
  int nmap0 = hype->getMap().numMaps() + 10;

  if(parent != NULL){
    parent->childHypes[hype->getCurrentMap()] -= 1;
  }

  hype->change(dec);

  int nmap1 = hype->getMap().numMaps() + 10;

  if(nmap0 < nmap1){
    ARRAY_RESIZE(childHypes, int, nmap0, nmap1);
    memset(childHypes+nmap0, 0, (nmap1-nmap0)*sizeof(int));
  }

  if(parent != NULL){
    parent->childHypes[hype->getCurrentMap()] += 1;
  }

  fresh = true;
}

void MultiSLAM::Hype::resetChildren(){
  nChildren = 0;
  memset(childHypes, 0, sizeof(int)*(hype->getMap().numMaps()+10));
}


//-----------------------------------------------------------------
//  MultiSLAM
//-----------------------------------------------------------------

void MultiSLAM::init(const MotionModelInterface *m,
                     const MapBuilderInterface *mapper){

  CHECK_DIST = 0.1;
  CHECK_NOBS = 3;

  MAX_HYPE_PER_AGENT = 5;
  NUMP_PER_AGENT = 100;
  RESAMPLE_STEP = 1;
  MAX_AGENTS = 5;

  if(m != NULL) slam.setOdoModel(m);

  if(mapper != NULL) sensorRange = mapper->getSensorRange();

  this->mapper = mapper;

  hypes = NULL;
}

MultiSLAM::MultiSLAM(const MotionModelInterface *m,
                     const MapBuilderInterface *mapper):_nextAgentId(0)
            ,n_odo(0),sz_odo(0),odo2scan(NULL){

  init(m,mapper);
}

MultiSLAM::~MultiSLAM(){
  int i;

  if(hypes != NULL){
    for(i = 0; i < n_hypes; ++i){
      if(hypes[i] != NULL){
	//Should not attempt to unregister
	hypes[i]->parent = NULL;
	delete hypes[i];
      }
    }
    delete[] hypes;
  }

  if(odo2scan != NULL) free(odo2scan);

}


void MultiSLAM::storeOdo(double T, int ScanId){
  slam.storeOdo(T); 

  if(n_odo >= sz_odo){
    sz_odo += 5000;
    odo2scan = (int*) realloc(odo2scan, sz_odo*sizeof(int));
  }

  odo2scan[n_odo] = ScanId;
  n_odo += 1;
}

bool MultiSLAM::dumpOdo2Scan(FILE*f, const char* name){
  int i;

  if( fprintf(f,"%s = [", name) < 0) return false;

  for(i =0; i < n_odo; ++i){
    if((i % 20) == 0) fprintf(f," ...\n");
    if(fprintf(f,"%d ", odo2scan[i]) < 0)  return false;
  }

  if(fprintf(f,"\n]; %% %s\n",name)<0)       return false;

  return true;
}


void MultiSLAM::start(){
  hypes    = new Hype*[MAX_AGENTS];
  MappingAgent *agent = new MappingAgent();
  agent->init(nextAgentId(), NUMP_PER_AGENT, &slam, this);

  hypes[0] = new Hype(agent, NULL);

  bestHype = hypes[0];
  n_hypes = 1;
  memset(hypes + 1, 0, (MAX_AGENTS - 1)*sizeof(Hype*));

  lastOdo = 0.0;
  n_obs = 0;
  minorCheckOdo = 0;
}

void MultiSLAM::start(const MotionModelInterface *m,
                      const MapBuilderInterface *mapper){

  sensorRange = mapper->getSensorRange();
  slam.setOdoModel(m);
  this->mapper = mapper;

  start();
}

void MultiSLAM::odometry(const OdometryReadingInterface *odo){
  //  REPORT("ODO: %d\n",minorCheckOdo);

  slam.advance(odo);


  if(slam.hasMoved() && n_obs > 0){
     minorCheckOdo += 1;
     if(minorCheckOdo > 10) minorCheckPoint();
  }
}

void MultiSLAM::observation(double TimeStamp, const ObservationStore &obs,
			    const int *obs_ind, int nobs){

  //  REPORT("OBS: %d [%04d]\n", nobs, n_obs);

  if(nobs <= 0) return;

  TLOG_EVAL_START;
  storeOdo(TimeStamp, obs.getLastScanId());
  slam.evaluate(TimeStamp, obs); 
  TLOG_EVAL_STOP;

  n_obs += 1;

  minorCheckPoint();

  bool okResample = true;

  if(slam.odometer().dist() - lastOdo > CHECK_DIST ||
     n_obs - lastNobs > CHECK_NOBS){
    
    TLOG_CHECK_START;
    checkPoint(&okResample);
    TLOG_CHECK_STOP;

    lastOdo = slam.odometer().dist();
    lastNobs = n_obs;

  }

  if(okResample && (n_obs % RESAMPLE_STEP) == 0){
    TLOG_RESAMPLE_START;
    slam.resample(n_hypes*NUMP_PER_AGENT);
    TLOG_RESAMPLE_STOP;
  }

}

void MultiSLAM::animationDump(FILE*f)const{
  int i;

  int i_max = 0;
  double w_max = slam[0]->getWeight();

  fprintf(f,"odo = [...\n");

  for(i = 0; i < slam.numParticles(); ++i){
    const SLAMParticle* p = slam[i];
    const RobotPose &r = p->getPose();


    fprintf(f,"%e, %e, %e, %e, %d\n"
	    ,r.x, r.y, r.rot, p->getWeight(), p->getId());

    if(p->getWeight() > w_max){
      w_max = p->getWeight();
      i_max = i;
    }
  }
  fprintf(f,"];\n");
  char buf[256];
  char prefix[256];

  //Go through the hypes
  for(i = 0; i < n_hypes; ++i){
    MappingAgent* agent = hypes[i]->hype;

    SimpleMap *m = agent->getCurrentLocalMap();
    fprintf(f,"%% Agent %d map %d\n",agent->getId(),agent->getCurrentMap()+1);
    sprintf(prefix,"hype(%d).",agent->getId());

    if(m != NULL){
      sprintf(buf,"%smap",prefix);
      m->matlabDump(f,buf);
      delete m;
    }else{
      fprintf(f,"%smap = [];\n", prefix);
    }

    fprintf(f,"%smapi = %d; %smapn = %d;\n", prefix
             , agent->getCurrentMap()+1
             , prefix, agent->getMap().numMaps());

    if(hypes[i]->fresh){
      fprintf(f,"%%Update hype: %d,[id %d]\n",i+1, agent->getId());
      hypes[i]->fresh = false;
      agent->animationDump(f, prefix);
    }else{
      fprintf(f,"%sregion = []; %smap_ref = [];\n",prefix,prefix);
    }
  }

//   const MapInterface *map = slam[i_max]->getMap();

//   RangeInterface range; //All landmarks
//   SimpleMap* m = map->getSubMap(range);
//   m->matlabDump(f,"map");
//   delete m;
}

void  MultiSLAM::minorCheckPoint(){
  int i;

  REPORT("MINOR CHECK POINT: %d agents\n", n_hypes);

  slam.normalise();
  minorCheckOdo = 0;


  int n_dead = 0;

  //First remove bad hypes
  for(i = 0; i < n_hypes; ++i){
    MappingAgent* agent = hypes[i]->hype;

    if(hypes[i]->dead){
      ABORT("Error: dead hype\n");
    }

    if(agent == NULL){ //Safety check
      ABORT("Error: NULL agent\n");
    }

    //    REPORT("...Agent %p\n",agent);

    agent->computeState();

    REPORT("...Agent[%d]| n= %d, w = %.10e m = %d"
	   , i
	   , agent->numParticles()
	   , agent->getWeight()
	   , agent->getCurrentMap()+1);


    if(agent->numParticles() <= 0.05*NUMP_PER_AGENT){
      REPORT("[removed]\n");

      hypes[i]->kill();
      n_dead += 1;
    }else{
      REPORT("[ok]\n");
    }
  }

  REPORT("Dead Hypes: %d\n", n_dead);

  if(n_dead > 0)  cleanupDeadHypes();

}

void MultiSLAM::finalise(){
  int i;
  double wmax = -Inf;
  bestHype = NULL;
  slam.normalise();

  for(i = 0; i < n_hypes; ++i){
    hypes[i]->hype->computeState();
    if(hypes[i]->hype->getWeight() > wmax){
      wmax = hypes[i]->hype->getWeight();
      bestHype = hypes[i];
    }

    hypes[i]->hype->finalise();
  }

}

void MultiSLAM::unregisterHype(Hype* h){
  int i;

  for(i = 0; i < n_hypes; ++i){
    if(hypes[i]->parent == h) hypes[i]->parent = NULL;
  }

  h->unregister();
}

void MultiSLAM::cleanupDeadHypes(){
  //Clean up dead hypes
  Hype *aliveHypes[n_hypes];
  Hype* deadHypes[n_hypes];
  REPORT("Cleaning up dead hypes.\n");

  int n_alive = 0;
  int n_dead = 0;
  int i,j;

  for(i = 0; i < n_hypes; ++i){
    if(hypes[i]->dead){
      deadHypes[n_dead] = hypes[i];
      n_dead += 1;
    }else{
      aliveHypes[n_alive] = hypes[i];
      n_alive += 1;
    }
  }

  for(i = 0; i < n_dead; ++i){
    for(j = 0; j < n_alive; ++j){
      if(aliveHypes[j]->parent == deadHypes[i])
	aliveHypes[j]->parent = NULL;
    }
    delete deadHypes[i];
  }

  if(n_dead > 0){
     memcpy(hypes, aliveHypes, n_alive*sizeof(Hype*));
     n_hypes = n_alive;
  }
}

static int qsort_compare(const void * v1, const void * v2){
  const MultiSLAM::ProposedHype *h1 = *((const MultiSLAM::ProposedHype**) v1);
  const MultiSLAM::ProposedHype *h2 = *((const MultiSLAM::ProposedHype**) v2);

  printf("QSORT: %p %e <-> %p %e\n",h1,h1->weight, h2, h2->weight);

  if(h1->weight < h2->weight)      return -1;
  else if(h1->weight > h2->weight) return  1;
  else                             return  0;
}

void MultiSLAM::addHype(ProposedHype* proposed){
  const char *action2name[] = {"Continue","==NEW MAP=="
                             , "Loop Close", "Transition"};


  MappingAgent::MappingDecision *dd = &proposed->decision;
  int mapDest = dd->map;
  Hype *parent = proposed->source;

  REPORT("  Adding: %03d -- %s -> %02d [%e]\n"
	 , proposed->source->hype->getId()
         , action2name[dd->action]
	 , mapDest + 1
	 , proposed->weight);

  if(parent->nChildren >= MAX_HYPE_PER_AGENT ||
     parent->childHypes[mapDest] > 0){
    REPORT(" -- refused.\n");
    return;
  }

  MappingAgent *agent = parent->hype->clone(nextAgentId());
  agent->change(proposed->decision);

  if(proposed->orphan){    parent = NULL;   }

  hypes[n_hypes] = new Hype(agent, parent);
  n_hypes += 1;
  REPORT(" -- accepted. [%p]\n",agent);
}

void MultiSLAM::checkPoint(bool *okResample){
  ProposedHype *decs[n_hypes*MAX_HYPE_PER_AGENT];

  int n_proposed = 0;
  int i;
  MappingAgent::MappingDecision dd[MAX_HYPE_PER_AGENT];
  int nd;

  REPORT("MAJOR CHECK POINT: %d hypes\n", n_hypes);

  MappingAgent::MappingDecision dd0[n_hypes];
  bool stays[n_hypes];

  for(i = 0; i < n_hypes; ++i){
    int j;

    nd = hypes[i]->hype->checkPoint(stays + i, dd,MAX_HYPE_PER_AGENT);

    if(!stays[i]){
      dd0[i] = dd[0];      //Store first action to take
      printf("First Choice[hype->%d]: %d\n", i+1,dd0[i].action);
      j = 1;
    }else{ j = 0; }

    //Add the rest actions
    for(; j < nd; ++j){
      decs[n_proposed] = new ProposedHype(hypes[i],
                          dd[j].weight*hypes[i]->hype->getWeight(),dd[j]
                          ,!stays[i]);

      printf("Proposed: %d %p %e\n",n_proposed,
                                    decs[n_proposed],decs[n_proposed]->weight);

      n_proposed += 1;
    }
  }

  //Only sort if more than one element
  if(n_proposed > 1){
    qsort(decs, n_proposed, sizeof(ProposedHype*), qsort_compare);
  }

  int n_hypes0 = n_hypes;
  bool resetWeight = false;

  for(i = 0; i < n_proposed && n_hypes < MAX_AGENTS; ++i){
    //Add hype
     addHype(decs[i]);
     resetWeight = true;
  }

  for(i = 0; i < n_hypes0; ++i){
    if(!stays[i]){
      unregisterHype(hypes[i]);
      hypes[i]->resetChildren();

      hypes[i]->change(dd0[i]);

      resetWeight = true;
    }
  }

  if(resetWeight){
    printf("Reseting Weight: %d\n",slam.numParticles());
    slam.resetWeight();
  }

  *okResample = !resetWeight;

  //Clean up
  for(i = 0; i < n_proposed; ++i) delete decs[i];
}
