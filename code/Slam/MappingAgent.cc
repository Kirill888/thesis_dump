#include "MappingAgent.h"
#include "matrix3.h"
#include "timeLog.h"
#include "multiSLAM.h"
#include "util.h"
#include "gridmap.h"
//#include "localiser.h"

////////////////////////////////////////////////////////////
// Support  functions
////////////////////////////////////////////////////////////


void matlabDump(FILE *f, 
                int i1, int i2,
		const RobotPoseCov r1in2,
		const RobotPose *fp, const double *w, int np,
		const Array2d       & path,
		const LoopCloseResult *res);

////////////////////////////////////////////////////////////////////////
// Global Profile
///////////////////////////////////////////////////////////////////////

ParticleHistory::SharedData *ParticleHistory::SharedData::makeCopy()const{
  SharedData *d = new SharedData;

  if(n > 0){
    d->data = (int*) malloc(n*sizeof(int));
    memcpy(d->data,data, sizeof(int)*n);
    d->n = n;
  }

  return d;
}

ParticleHistory::SharedData::~SharedData(){ free(data); }

void ParticleHistory::SharedData::set(int i, int v){
  if(n <= i){
    int n0 = n;
    int j;
    n = i + 1;
    data = (int*) realloc(data, sizeof(int)*n);

    for(j = n0; j < i; ++j) data[j] = -1;

  }
  data[i] = v;
}

void ParticleHistory::SharedData::merge(const ParticleHistory::SharedData*o){
  int i;

  if(n < o->n){ //Make sure the size is right
    set(o->n-1, o->data[o->n-1]);
  }

  for(i = 0; i < o->n; ++i){
    if(data[i] < 0 && o->data[i] >= 0) data[i] = o->data[i];
  }
}



int ParticleHistory::getMap(int i)const{
  return ((maps != NULL && i < maps->n) ? maps->get(i): -1);
}

int ParticleHistory::getTransition(int i)const{
  return ((trans !=NULL && i < trans->n) ? trans->get(i): -1);
}

void ParticleHistory::setMap(int i, int v){
  if(maps == NULL){
    maps = new SharedData;
  }

  if(maps->copy_count > 1){
    maps->copy_count -= 1;

    maps = maps->makeCopy();
  }

  maps->set(i,v);
}

void ParticleHistory::setTransition(int i, int v){
  if(trans == NULL){
    trans = new SharedData;
  }

  if(trans->copy_count > 1){
    trans->copy_count -= 1;

    trans = trans->makeCopy();
  }

  trans->set(i,v);
}

void ParticleHistory::merge(const ParticleHistory* other){

  if(other->maps != NULL){
    if(maps == NULL){
      maps = new SharedData;
    }

    if(maps->copy_count > 1){
      maps->copy_count -= 1;

      maps = maps->makeCopy();
    }

    maps->merge(other->maps);
  }

  if(other->trans != NULL){
    if(trans == NULL){
      trans = new SharedData;
    }

    if(trans->copy_count > 1){
      trans->copy_count -= 1;

      trans = trans->makeCopy();
    }

    trans->merge(other->trans);
  }
}

ParticleHistory::ParticleHistory(const ParticleHistory &other):maps(other.maps)
                                                        ,trans(other.trans){
  if(maps  != NULL) maps->copy_count += 1;
  if(trans != NULL) trans->copy_count += 1;
}

ParticleHistory * ParticleHistory::clone()const{
  return new ParticleHistory(*this);
}

ParticleHistory::~ParticleHistory(){
  if(maps != NULL){
    maps->copy_count -= 1;

    if(maps->copy_count == 0) delete maps;
  }

  if(trans != NULL){
    trans->copy_count -= 1;

    if(trans->copy_count == 0) delete trans;
  }
}

void ParticleHistory::debugDump(FILE* f)const{
  int i;

  if(maps != NULL){

    fprintf(f,"Maps:  ");
    for(i = 0; i < maps->n; ++i){
      fprintf(f," %3d", maps->get(i));
    }
    fprintf(f,"\n");
  }else{
    fprintf(f,"Maps: null\n");
  }

  if(trans != NULL){
    fprintf(f,"Trans: ");
    for(i = 0; i < trans->n; ++i){
      fprintf(f," %3d", trans->get(i));
    }
    fprintf(f,"\n");
  }else{
    fprintf(f,"Trans: null\n");
  }
  

}

////////////////////////////////////////////////////////////////////////
// Mapping Agent
///////////////////////////////////////////////////////////////////////


int MappingAgent::NUM_PARTICLES_PER_PATH = 10000;

////////////////////////////////////////////////////////////////////////
// Initialisation and destruction
///////////////////////////////////////////////////////////////////////

void MappingAgent::init(int ID, int numP, SLAM * Slam, const MultiSLAM *mslam){
  id = ID;

  slam      = Slam;
  multiSLAM = mslam;
  mapper    = multiSLAM->getMapBuilder()->clone();
  // mapper->setMapArea(&regMap);

  //  regMap.set(&grid);

  sz_matchers = 500;
  matchers = new LoopCloseResult*[sz_matchers];
  memset(matchers,0, sz_matchers*sizeof(LoopCloseResult*));

//   sz_hist = 100;
//   hist = new PHist*[sz_hist];
//   memset(hist, 0, sz_hist*sizeof(PHist*));

  sz_hist = 0;
  hist = NULL;
  n_blocks = 0;
  blocks = NULL;

  //Add particles to the filter.
  int i;
  int np_old = slam->numParticles();
  double w0 = 1.0/numP;

  slam->resize(np_old + numP);

  for( i = 0; i < numP; ++i){
    SLAMParticle *p = slam->makeNewParticle(this);
    p->setWeight(w0);
    slam->setp(i + np_old, p);
  }

  mappingNew = true;
  prevMap = -1;
  currentMap = -1;
  nTotalChecks = 0;

  initNextMapRegion();
  startNewMap();
    
  slam->addMonitor(monitor);

  resetScore();

  initNeighbours();
  initMappingRegion();
}


void MappingAgent::set(const MappingAgent &other){
  mappingNew = other.mappingNew;
  superMode = other.superMode;

  gmap      = other.gmap;
  slam      = other.slam;
  multiSLAM = other.multiSLAM;
  mapper    = CLONE(other.mapper);
  mon       = other.mon;

  id         = other.id;
  prevMap    = other.prevMap;
  currentMap = other.currentMap;
  regMap     = gmap[currentMap].getRegion();

  nTotalChecks = other.nTotalChecks;

  sz_matchers = other.sz_matchers;
  int i;

  if(sz_matchers > 0){

    matchers = new LoopCloseResult*[sz_matchers];

    for(i = 0; i < sz_matchers; ++i){
      if(other.matchers[i] != NULL){
	matchers[i] = new LoopCloseResult(*other.matchers[i]);
      }else{
	matchers[i] = NULL;
      }
    }
   
  }else{
    matchers = NULL;
  }

  if(other.sz_hist > 0){
    destroyHist();

    sz_hist = other.sz_hist;
    n_blocks = other.n_blocks;

    hist = new PHist*[sz_hist];
    blocks = new PHist*[sz_hist];

    memset(hist  , 0, sz_hist*sizeof(PHist*));
    memset(blocks, 0, sz_hist*sizeof(PHist*));

    //SPAM:
    for(i = 0; i < n_blocks; ++i){
      blocks[i] = other.blocks[i]->clone();
    }

    for(i = 0; i < gmap.numMaps(); ++i){
      int j;
      if(other.hist[i] != NULL){
	for(j = 0; j < n_blocks; ++j){
	  if(other.hist[i] == other.blocks[j]){
	    hist[i] = blocks[j];
	    break;
	  }
	}
      }else{
	hist[i] = NULL;
      }
    }

  }else{
    destroyHist();
  }

  path = other.path;

  nextMap = other.nextMap;

  initNeighbours();

  //Register the monitor
  if(slam != NULL) slam->addMonitor(monitor);

  resetScore();
}

void MappingAgent::destroyHist(){
  int i;

  if(blocks == NULL || hist == NULL) return;

   for(i = 0; i < n_blocks; ++i){
//      printf("Deleting block %d = %p\n", i, blocks[i]);
//      fflush(stdout);
     delete blocks[i];
   }

//    fflush(stdout);
//    printf("Deleting blocks and hist (%p,%p)\n", blocks, hist);

   delete[] blocks;
   delete[] hist;

   hist = NULL;
   blocks = NULL;
   n_blocks = 0;
   sz_hist = 0;
}

//Resets map to empty, deletes all related particles from the SLAM filter
void MappingAgent::destroy(){
  DESTROY_ARRAYP(matchers,sz_matchers);

  gmap.reset();

  int i;

  if(slam != NULL){
    //Unregister the monitor
    slam->removeMonitor(monitor);

    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle *particle = (SLAMParticle*)slam->get(i);
      if(particle->getId() == id){
	slam->setp(i,NULL);
	delete particle;
      }
    }
    slam->cleanupDead();
  }

  for(i = 0; i < n_neighbours; ++i) delete neighbours[i];

  n_neighbours = 0;
  DESTROY(mapper);

  destroyHist();
}

//Create clone of itself with the new ID, 
//this duplicates particles in the SLAM filter.
// assumes that computeState has been called recently
MappingAgent * MappingAgent::clone(int newID){
  MappingAgent *clon = new MappingAgent(*this);
  clon->setId(newID);
  clon->nTotalChecks = 0;

  if(slam != NULL){
    int i;
    int np_old = slam->numParticles();
    slam->resize(np_old + mon.np);
    int n = 0;

    clon->mon.bestP = NULL;

    for(i = 0; i < np_old; ++i){
      SLAMParticle *particle = (SLAMParticle*)slam->get(i);

      if(particle->getId() == id){
	SLAMParticle *p_new = new SLAMParticle(particle);
	p_new->setOwner(clon);

	slam->setp(np_old + n, p_new);
	n += 1;

	if(particle == mon.bestP){ //Update bestP of the clon
	  clon->mon.bestP = p_new;
	}
      }
    }

    if(mon.np != n){
      ABORT("Error: expect %d particles, but found %d\n", mon.np, n);
    }
  }
  return clon;
}


void MappingAgent::finalise(){
  updateCurrentMap();
  gmap.updateCoreLandmarks(currentMap,false);

  if(mappingNew){ //Finalise mapping region.
    mappingProcess->finaliseMapArea(regMap,gmap[currentMap].getMap());
  }

  storePath();
}


////////////////////////////////////////////////////////////////////////
// State Computation
///////////////////////////////////////////////////////////////////////

//Generates an array of weighted mapping decisions.
//Array is sorted in decsending order, up to nmax
//hypothesis is generated, if there are more than nmax possibilities
//only the fittest nmax will be passed.
//RETURN VALUE: number of decisions generated.
int MappingAgent::checkPoint(bool *canStay,
                             struct MappingAgent::MappingDecision *decs_out,
			     int nmax){
  int n = 0;
  int i;
  double newScore = 1;

  bool startingNew = false;

  normaliseScore();

  double stayScore;
  RobotPose sensor;

  mappingProcess->robot2sensor(&sensor, &mon.mean);

  stayScore = mappingProcess->computeRegionScore(regMap, &sensor);
  //  printf("StayScore: %.3f\n",stayScore);

  if(stayScore < 0.7){
    startingNew = true;

    printf("Checking neighbours: %d\n",n_neighbours);

    //1. Check possible transitions.
    for(i = 0; i < n_neighbours; ++i){

      double score = mappingProcess->computeRegionScore(neighbours[i]->area,
                                                        &sensor);
      printf("   N[%d] = %f\n",i,score);

      if(score > 1.1*stayScore){
	startingNew = false;
	addDecision(MappingDecision(TransitToMap, neighbours[i]->mapId,
				    neighbours[i]->score),
		    decs_out, n, nmax);
      }
    }

    //2. If no transitions possible
    if(startingNew){
      if(stayScore < 0.1){ //If stay score is too low -- start new map
	*canStay = false;
	initNextMapRegion();

	addDecision(MappingDecision(StartNewMap, gmap.numMaps(), newScore), 
		    decs_out, n, nmax);
      }else{
	startingNew = false;
	*canStay = true;
      }
    }else{
      *canStay = false;
    }

  }else{
    *canStay = true;
  }

  // Check Loop Close (only if mapping first time)
  if(mappingNew){
    checkLoopClose(startingNew);

    int i;

    for(i = 0; i < gmap.numMaps(); ++i){
      if(matchers[i] != NULL && matchers[i]->w > 0){
	addDecision(MappingDecision(CloseLoop, i, matchers[i]->w)
		    ,decs_out, n, nmax);
      }
    }
  }

  resetScore();

  return n;
}

void MappingAgent::initNextMapRegion(){
  mappingProcess->getInitialMapArea( &(nextMap.region), &(nextMap.x0));

  if(currentMap >= 0){
    //Now need to subtract neighbouring regions.
    int i;
    int n = n_neighbours + 1;
    RobotPose odo[n];
    const MapAreaInterface *regions[n];

    //Compute pose of the new region in the current one
    double x1,x2,y1,y2,a1,a2;
    x1 = meanPose.x;   y1 = meanPose.y;   a1 = meanPose.rot;
    x2 = nextMap.x0.x; y2 = nextMap.x0.y; a2 = nextMap.x0.rot;

    RobotPose odo0;
    odo0.rot = angleDiffRad(a1, a2);
    double ca,sa;
    ca = cos(odo0.rot); sa = sin(odo0.rot);

    odo0.x = x1 - x2*ca + y2*sa;
    odo0.y = y1 - x2*sa - y2*ca;

    printf("New Map Pose: %.3f %.3f %.3f (deg)\n",odo0.x,odo0.y
	   ,odo0.rot*RADTODEG);

    //Compute pose of Neighbours and current region in the new region
    for(i = 0; i < n; ++i){
      int ind; 
      if( i < n_neighbours){
	ind = neighbours[i]->mapId;
      }else{
	ind = currentMap;
      }

      const RobotPoseCov &r = gmap.getMapRef(ind);

      printf("Convert Pose: %.2f %.2f %.2f =>",
	     r.x, r.y, r.rot*RADTODEG);

      regions[i] = gmap.get(ind).getRegion();
      double dx,dy;

      dx = r.x - odo0.x;
      dy = r.y - odo0.y;

      odo[i].rot = angleDiffRad(r.rot, odo0.rot);
      odo[i].x =  dx*ca + dy*sa;
      odo[i].y = -dx*sa + dy*ca;

      printf("%.2f %.2f %.2f\n", odo[i].x, odo[i].y, odo[i].rot*RADTODEG);
    }

    //Subtract regions
    mappingProcess->subtractFromRegion(nextMap.region, regions, odo, n);
  }

  mappingProcess->finaliseMapArea(nextMap.region, nextMap.x0);


}

//Compute number of particles and total weight.
void MappingAgent::computeState(){

  if(mon.np <= 0) return;

  computeRobotPose();

  int i;

  //Compute score for neighbours, 
  for(i = 0; i < n_neighbours; ++i){
    double w = computeScore(neighbours[i]->area, &meanPose);
    neighbours[i]->score += w;
  }


#if 0
  printf("ComputeState: %d, %f\t[%d]\n",mon.np,mon.sumW,currentMap+1);
  print_rcov("   pose",meanPose);
  printf("...MapNew    = %s\n", mappingNew?"true":"false");
#endif

}

bool MappingAgent::isRobotInside(){
  double x = mon.mean.x;
  double y = mon.mean.y;

  double prob = regMap->probIsInside(x,y);

  return prob > 0.7;
}

void MappingAgent::computeRobotPose(){
  int i;

  const RobotPose & mean = mon.mean;
  meanPose.set(mean, ZEROS_3x3);
  int numP = mon.np;
  register double w = 1/(double)numP;

  if(superMode){
    //Compute covariance
    for(i = 1; i < slam->numParticles(); ++i){
      register double x,y,o;
      x = slam->get(i)->getPose().x - mean.x; 
      y = slam->get(i)->getPose().y - mean.y;
      o = angleDiffRad(slam->get(i)->getPose().rot, mean.rot);

      meanPose.cov[0][0] += x*x;
      meanPose.cov[0][1] += x*y;
      meanPose.cov[0][2] += x*o;
      meanPose.cov[1][1] += y*y;
      meanPose.cov[1][2] += y*o;
      meanPose.cov[2][2] += o*o;
    }

  }else if(numP > 3){//Compute covariance if there are particles left
    for(i = 0; i < slam->numParticles(); ++i){
      const SLAMParticle *particle = (const SLAMParticle*)slam->get(i);
      if(particle->getId() == id){
	register double x,y,o;
	x = particle->getPose().x - mean.x; 
	y = particle->getPose().y - mean.y;
	o = angleDiffRad(particle->getPose().rot, mean.rot);

	meanPose.cov[0][0] += x*x;
	meanPose.cov[0][1] += x*y;
	meanPose.cov[0][2] += x*o;
	meanPose.cov[1][1] += y*y;
	meanPose.cov[1][2] += y*o;
	meanPose.cov[2][2] += o*o;
      }
    }
  }

  meanPose.cov[0][0] *= w;
  meanPose.cov[0][1] *= w;
  meanPose.cov[0][2] *= w;
  meanPose.cov[1][1] *= w;
  meanPose.cov[1][2] *= w;
  meanPose.cov[2][2] *= w;

  meanPose.cov[1][0] = meanPose.cov[0][1];
  meanPose.cov[2][0] = meanPose.cov[0][2];
  meanPose.cov[2][1] = meanPose.cov[1][2];

}

double MappingAgent::computeScore(const MapAreaInterface*a, const RobotPose*p){
  double w = 0;

  const int np = 4;
  double points[np][2] ={{0.30,  0.00},
			 {0.40,  0.00},
		         {0.20,  0.25},
		         {0.20, -0.25}};

  int i;
  double ca = cos(p->rot);
  double sa = sin(p->rot);

  for(i = 0; i < np; ++i){
    double x = ca*points[i][0] - sa*points[i][1] + p->x;
    double y = sa*points[i][0] + ca*points[i][1] + p->y;
    w += a->probIsInside(x, y);
  }


  return w/np;
}

 //============================================================//
 // Loop Close Checking
 //===========================================================//


const double MAP_OVERLAP_THRESHOLD = -7;
const double ODO_OVERLAP_THRESHOLD = -7;

const double POSE_MD2_THRESHOLD    = POW2(5.0);

void MappingAgent::resetMatchers(){
  int i;

  for(i = 0; i < sz_matchers; ++i){
    if(matchers[i] != NULL){
       matchers[i]->reset();
       matchers[i]->scanId = -1;
       matchers[i]->probPass = 0.0;
    }
  }
}


SimpleMap* MappingAgent::getRecentMap(int scanId){
  if(mon.bestP == NULL) return NULL;

  RangeLastScanId range(scanId);
  SimpleMap* m = mon.bestP->getMap()->getSubMap(range);
  return m;
}

SimpleMap* MappingAgent::getCurrentLocalMap(){
  if(mon.bestP == NULL) return NULL;

  RangeMature range;
  SimpleMap* m = mon.bestP->getMap()->getSubMap(range);
  return m;
}

//DODGE!!!!!!!!!!!!!!!!
///////////////////////
double MappingAgent::probIsInside(const RobotPoseCov* robot,
				  const MapTransition* path,
				  const MapAreaInterface* region){
  int i;
  double w = 0;

  RobotPose r;

  int N_CHECKS = 100;

  for(i = 0; i < N_CHECKS; ++i){
    r.set(robot->x, robot->y, robot->rot);
    int j = path->sample();

    path->transit(&r,currentMap,j);

    w += region->probIsInside(&r);
  }


  return w/N_CHECKS;
}

void MappingAgent::checkLoopClose(bool forceUpdate){
  printf("Check Loop Close: %d [%d]\n",mon.lastScanId, (int)forceUpdate);
  int i;
  SimpleMap *m = getCurrentLocalMap();

  if(m == NULL) return;
  if(m->numElements() < 5){ delete m; return ;}


  printf("  numElements = %d;\n", m->numElements());

  for(i = 0; i < gmap.numMaps(); ++i){
    if(i != currentMap && !gmap.isAdjacent(i,currentMap)){
      bool shouldMatch = forceUpdate;
      LoopCloseResult *res = matchers[i];
      const MapTransition* path       = gmap.getCompoundTransition(i);
      const MapAreaInterface * region = gmap[i].getRegion();
 
      if(res == NULL){
	res = new LoopCloseResult;
	matchers[i] = res;
      }

      if(res->scanId < 0){ shouldMatch = true; }  //First time
      else if(res->scanId < mon.lastScanId - 20){ //Every 20 scans
	shouldMatch = true;
      }

      if(shouldMatch){
	res->reset();

	res->scanId = mon.lastScanId;
	double pPass = probIsInside(&meanPose, path, region);

	printf("ProbInside(%d) = %g [%d]\n", i+1, pPass, mon.lastScanId);

	res->probPass = max(res->probPass, pPass);

	shouldMatch = pPass > 0.1;
      }

      if(!shouldMatch && 
         forceUpdate && res->probPass > 0){ //Attempt matching if the robot
	shouldMatch = true;           //have passed through the region before
	res->probPass = 0.0;
      }

      if(shouldMatch){
        RobotPoseCov r2in1 = gmap.getMapRef(i);
        print_rcov("r2in1",r2in1);
	r2in1.scaleCov(1.5);

        printf("Map Matching: %d=>%d\n",currentMap+1,i+1);

	TLOG_MAPMATCH_START;
	if( !match_maps(res, mappingProcess->matcher, m, gmap[i].getMap(), 
			&r2in1, POSE_MD2_THRESHOLD)){
	  printf("-----------FAILED-----------\n");
	  res->w = 0;
	}else{
	  res->w = 0.8;
	  printf("-----------PASSED-----------\n");
	}
	TLOG_MAPMATCH_STOP(currentMap+1, i +1);

      }else{
       	res->w = 0.0;
      }

    }
  }

  delete m;
}

////////////////////////////////////////////////////////////////////////
// State Change
///////////////////////////////////////////////////////////////////////


//Take an action specified by the decs.
void MappingAgent::change(struct MappingAgent::MappingDecision decs){
  //No action is required.
  if(decs.action == Continue) return;

  //Reset nTotalChecks here
  nTotalChecks = 0;

  finalise();
  switch(decs.action){
  case TransitToMap:
    //updateTransition(decs.map);
    transitToMap(decs.map);
    scaler = 1.0;
    break;

  case StartNewMap:
    startNewMap();
    scaler = 1.0;
    break;

  case CloseLoop:
    closeLoop(decs.map);
    scaler = 1.0;
    break;

  default:
    printf("Unsupported action: %d\n", decs.action);
  }

  //Some common updates.
  //  gmap.setReferenceFrame(currentMap);
  initNeighbours();
  initMappingRegion();

  resetScore();
  resetMatchers();
}

MapTransition* MappingAgent::makeTransition(const RobotPose &odo0){
  int np = slam->numParticles();
  RobotPose * pose = new RobotPose[np];
  int *histId = new int[np];

  np = getFinalPoses(pose, histId);
  if(np <= 0){
    ABORT("makeTransition: NO PARTICLES WERE FOUND\n");
  }

  int i;
  for(i = 0; i < np; ++i){
    double a,x,y;
    a = angleDiffRad(pose[i].rot, odo0.rot);
    double ca,sa;
    ca = cos(a); sa = sin(a);
    x = pose[i].x - odo0.x*ca + odo0.y*sa;
    y = pose[i].y - odo0.x*sa - odo0.y*ca;

    pose[i].set(x,y,a);
  }

  MapTransition *t = new MapTransition(currentMap, gmap.numMaps()-1
				       ,pose, np);

  //Update the history
  int transId = gmap.numTrans();
  PHist *h = hist[currentMap];

  for(i = 0; i < h->n; ++i){
    (*h)[i]->setTransition(transId, i);
  }

  delete[] pose;
  delete[] histId;

  return t;
}

void MappingAgent::startNewMap(){
  int i;

  printf("Start new map id=%d (%p)\n", id,this);

  /////////////////////////////////////////////////
  // Compute Map Area and robot's pose within it
  /////////////////////////////////////////////////
  RobotPose odo0(nextMap.x0);
  regMap = nextMap.region;

  mapper->setMapArea(regMap);

  //ADD empty map.
  LocalMap *emptyMap = new LocalMap();
  emptyMap->setMapRegion(regMap);
  gmap.add(emptyMap);

  //////////////////
  // Here we construct transition based on current
  // particle locations in current map frame.

  if(currentMap >= 0){
    gmap.addTransition(makeTransition(odo0));
  }

  printf("Start new map: %d=>%d  (id %d)\n", currentMap+1, gmap.numMaps(),id);

  ////////////////////
  // Set current map frame
  ///////////////////////
  prevMap = currentMap;
  currentMap = gmap.numMaps() - 1;
  gmap.setReferenceFrame(currentMap);


  //Go through all particles and reset them

  if(superMode){
    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle* p = (SLAMParticle*) slam->get(i);

      p->setPose(odo0);
      p->setEmptyMap();
      p->resetPath();
    }
  }else{
    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle* p = (SLAMParticle*) slam->get(i);
      if(p->getId() == id){
	p->setPose(odo0);
	p->setEmptyMap();
	p->resetPath();
      }
    }
  }

  mappingNew = true;
  scaler = 1.0;
}

inline double computeWeight(const RobotPose* r2in1,
                            const SimpleMap *m1, const SimpleMap *m2){
  SimpleMap m2_(*m2);
  m2_.translateMe(*r2in1);
  double w_log = m1->probMatchLog(m2_);
  return exp(w_log);
}

void MappingAgent::closeLoop(int mapId){
  /*Steps involved:
   * 1. Generate Path from currentMap to mapId
   * 2. Evaluate final poses of the path.
   * 3. Sample NP from these poses.
   * 
   * THEN
   *   a. Add link to the map (using samples from map alignment)
   *   b. Call transitToMap(mapId)
   */

  TLOG_LOOPCLOSE_START;
  int i1 = currentMap;
  int i2 = mapId;

//   const MapPath *path = gmap.getPath(i2);
  LoopCloseResult *res = matchers[mapId];

  //Add transition
  ////////////////////////////////////////
  printf("Sampling Transition\n");
  //Adding new transition
  int NUM_T = multiSLAM->NUMP_PER_AGENT;
  double weight[NUM_T];
  RobotPose trans[NUM_T];

  mappingProcess->sampleTransition(res->map1, res->map2, &res->r2in1, 
                                   trans, weight, NUM_T);

  gmap.addTransition(new MapTransition(i1,i2,trans,NUM_T));

  //Update the current map before transit.
  LoopClosePostProcess(mapId);


  printf("------LOOP_CLOSED----%d->%d----[id %d]\n",i1+1,i2+1, id);
  res->map1->matlabDump(stdout,"m1");
  res->map2->matlabDump(stdout,"m2");

  //Now just perform the transition.
  /////////////////////////////////
  transitToMap(mapId);


  //Debug output
#if 0 //DUMP_LOOP_CLOSE_DATA
 {
  int i;
  char buf[256];
  sprintf(buf,"fp%03d_%03d.m",i1+1,i2+1);
  FILE * f = fopen(buf,"w");

  if(f == NULL){
    printf("Failed to open %s\nDumping to stdout.\n", buf);
    f = stdout;
  }

  printf("....Saving to %s\n",buf);
  matlabDump(f,i1,i2,r1in2
	     , fp, w, np
	     , pth
	     , res);

  fprintf(f,"transition = [ ...\n");

  for(i = 0; i < NUM_T; ++i){
    fprintf(f,"%e %e %e %e\n"
	    ,trans[i].x
	    ,trans[i].y
	    ,trans[i].rot
	    ,weight[i]);
  }
  
  fprintf(f,"];\n");

  fclose(f);
 }
#endif

  TLOG_LOOPCLOSE_STOP(i1+1,i2+1);
}

void MappingAgent::LoopClosePostProcess(int mapId){
  //Recompute neighbours
  gmap.setReferenceFrame(currentMap);
  gmap.updateCoreLandmarks(currentMap,true);

  //Subtract neighbours from current map area
  int i;
  int n = 0;

  RobotPose odo[gmap.numMaps()];
  const MapAreaInterface* a[gmap.numMaps()];

  for(i = 0; i < gmap.numMaps(); ++i){
    if(gmap.isNeighbour(i)){
      odo[n] = gmap.getMapRef(i);
      a[n] = gmap.get(i).getRegion();
      n += 1;
    }
  }

  mappingProcess->subtractFromRegion(regMap, a, odo, n);
  mappingProcess->finaliseMapArea(regMap);

  //Fuse transitions: (DODGE: untested)
#if 0
  const MapTransition *t0 = gmap.getTransition(mapId, currentMap);

  for(i = 0; i < gmap.numMaps(); ++i){
    if(i!= mapId && i != currentMap && 
                    gmap.isAdjacent(currentMap, i) && 
                   !gmap.isAdjacent(i,mapId)){
      //Add transition betwen i and mapId.
      const MapTransition *t = gmap.getTransition(i,currentMap);

      printf("Merging Transitions:%d->%d->%d\n"
             , mapId+1, currentMap+1,i+1);

      //Fuse t and t0
      MapTransition* trans = mergeTransitions(t0,t, mapId, i);

      gmap.addTransition(trans);
    }
  }
#endif

}

void MappingAgent::updateTransition(int mapId){
  int i1 = currentMap;
  int i2 = mapId;

  printf("Update Transtion: %d->%d\n", i1+1,i2+1);

  const SimpleMap *m1 = gmap[i1].getMap();
  const SimpleMap *m2 = gmap[i2].getMap();
  RobotPoseCov r2in1 = gmap.getMapRef(i2);
  r2in1.scaleCov(4);

  LoopCloseResult res;

  MapTransition *t = gmap.getTransition(i1,i2);
  printf("Map Matching: %d=>%d\n", i1+1, i2+1);
  TLOG_MAPMATCH_START;

  if(match_maps(&res, mappingProcess->matcher, m1, m2, &r2in1,
		POSE_MD2_THRESHOLD)){
    printf("...[Matched]\n");

    int NUM_T = multiSLAM->NUMP_PER_AGENT;

    double weight[NUM_T];
    RobotPose trans[NUM_T];

    mappingProcess->sampleTransition(res.map1, res.map2, &res.r2in1, 
				     trans, weight, NUM_T);

    if(t != NULL){
      //Update transition with the sample
      t->update(i1,i2, trans,weight,NUM_T);
    }else{
      //Create new transition
      gmap.addTransition(new MapTransition(i1,i2,trans,weight, NUM_T));
    }

  }else{
    printf("...[Failed]\n");
  }
  TLOG_MAPMATCH_STOP(currentMap+1, mapId +1);

}

void MappingAgent::sampleBlock(int map2, int* ind, int np){
  PHist *h1,*h2;
  h1 = hist[currentMap];
  h2 = hist[map2];
  int i;

  if(h1 == h2){
    ABORT("Sample block -- shared block!!\n");
  }

  if(h1->n != np){
    ABORT("Sample block -- invalid number of particles %d != %d\n",
	  np, h1->n);
  }

  //Sample profiles
  if( np == h2->n ){
    for(i = 0; i < np; ++i){
      ind[i] = i;
    }

    shuffle_ind(ind, np, np*2);
  }else{
    UniformRandomNumber dice(0, h2->n - 1);

    for(i = 0; i < np; ++i){
      ind[i] = dice.nextIntRandom();
    }
  }

  //Merge profiles
  for(i = 0; i < np; ++i){
    h1->h[i]->merge(h2->h[ind[i]]);
  }

  //Delete second profile
  delete h2;
  for(i = 0; i < n_blocks; ++i){
    if(blocks[i] == h2){
      blocks[i] = blocks[n_blocks-1];
      break;
    }
  }
  n_blocks -= 1;

  //Merge two blocks: Set h2 = h1
  for(i = 0; i < gmap.numMaps(); ++i){
    if(hist[i] == h2) hist[i] = h1;
  }

}

void MappingAgent::transitToMap(int mapId){
  int i;

  const MapPath *path = gmap.getPath(mapId)->clone();

  if(path == NULL){
    ABORT("ERROR: attempting transition %d=>%d\n"
	  "no such link in the map.\n", currentMap, mapId);
  }

  printf("Making Transition %d => %d  [id %d]\n", currentMap+1, mapId+1,id);

#if DUMP_TRANSIT
  int scanId  = mon.lastScanId;
  char buf[256];
  sprintf(buf,"transition%03d_%03d_%05ddump.m",currentMap+1,mapId +1,scanId);
  printf("Dumping to: %s\n",buf);
  FILE* f = fopen(buf,"w");
  fprintf(f,"%%Transition from %d->%d\n",currentMap+1, mapId +1);
#if 0
  gmap.get(currentMap).getMap()->matlabDump(f,"m1");
  gmap.get(mapId).getMap()->matlabDump(f,"m2");
#else
  gmap.get(currentMap).dumpAll(f,"m1");
  gmap.get(mapId).dumpAll(f,"m2");
#endif

  path->matlabDump(f,"trans");
  fprintf(f,"obs = [...\n");
  for(i = 0; i < mon.nObs; ++i){
    mon.obsStore->get(i)->matlabDump(f);
    fprintf(f,"\n");
  }
  fprintf(f,"];\n");
  fprintf(f, "odo = [ ...\n");

#define DUMP_ODO(r) fprintf(f,"   %.5e %.5e %.5e", r.x,r.y,r.rot)
#define DUMP_DOUBLE(w) fprintf(f," %e", w)
#define DUMP_INT(i) fprintf(f," %d",i)
#define DUMP_EOL fprintf(f,"\n")
#define DUMP_END_ARRAY fprintf(f,"];\n")
#define DUMP_FINISH fclose(f)

#else

#define DUMP_ODO(r)
#define DUMP_EOL
#define DUMP_END_ARRAY
#define DUMP_DOUBLE(w)
#define DUMP_INT(i)
#define DUMP_FINISH

#endif

  int np;
  int nhops                = path->getNHops();
  RobotPose *poses         = new RobotPose[mon.np];
  SLAMParticle** particles = new SLAMParticle*[mon.np];
  const LocalMap &lmap     = gmap[mapId];

  if(superMode){
    np = slam->numParticles();

    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle *p = (SLAMParticle*)slam->get(i);
      particles[i] = p;
      poses[i] = p->getPose();
    }
  }else{
    np = 0;

    for(i = 0; i < slam->numParticles() && np < mon.np; ++i){
      SLAMParticle *p = (SLAMParticle*)slam->get(i);

//       if(p == NULL){ABORT("Error: null particle\n");}
//       if(p->getOwner() == NULL){ABORT("Error: particle without owner!");}

      if(p->getId() == id){
	particles[np] = p;
	poses[np] = p->getPose();

	np += 1;
      }
    }
  }

  //Transfer particles

  int *ind = new int[np];

  int hop;
  int m = currentMap;
  PHist *h = hist[currentMap];

  if(h->h[0]->getMap(mapId) < 0){
    //If destination map is unknown
    sampleBlock(mapId, ind, np);
  }

  for(hop = 0; hop < nhops; ++hop){
    const MapTransition *t = path->get(hop);
    int transId = path->ind(hop);

    int m_dest = t->transit(m);

    if((*h)[0]->getTransition(transId) < 0){
      if(hist[m]->h[0]->getTransition(transId) >= 0){
	sampleBlock(m, ind, np);
      }else if(hist[m_dest]->h[0]->getTransition(transId) >= 0){
	sampleBlock(m_dest, ind, np);
      }else{
	t->sample(ind, np);
	shuffle_ind(ind, np, np*2);

	for(i = 0; i < np; ++i){
	  (*h)[i]->setTransition(transId, ind[i]);
	}
      }
    }

    for(i = 0; i < np; ++i){
      t->transit(poses + i, m, (*h)[i]->getTransition(transId));
    }

    m = m_dest;
  }

  //Update particles;
  for(i = 0; i < np; ++i){
    int imap = h->h[i]->getMap(mapId);

    if(imap < 0){
      ABORT("Transition %d->%d: Map index is not set! [particle %d]\n"
            , currentMap+1, mapId +1, i );
    }

    particles[i]->resetPath();
    particles[i]->setPose(poses[i]);
    particles[i]->setMap(lmap.getMap(imap));
    particles[i]->setHist(i);

    DUMP_ODO(particles[i]->getPose()); 
    DUMP_INT(imap);
    DUMP_EOL;
  }


  //Clean up
  delete[] poses;
  delete[] particles;
  delete[] ind;

  DUMP_END_ARRAY;
  DUMP_FINISH;

  prevMap    = currentMap;
  currentMap = mapId;
  gmap.setReferenceFrame(currentMap);

  regMap = gmap[currentMap].getRegion();
  mapper->setMapArea(regMap);

  mappingNew = false;
}

void MappingAgent::animationDump(FILE *f,const char *var){
  int i;

  fprintf(f,"%smap_ref = [...\n",var);
  for(i = 0; i < gmap.numMaps(); ++i){
    const RobotPoseCov &ref = gmap.getMapRef(i);
    fprintf(f,"%.3e %.3e %.3e\n"
	    ,ref.x
	    ,ref.y
	    ,ref.rot);

  }
  fprintf(f,"];\n");

  char buf[256];
  sprintf(buf,"%sregion",var);
  regMap->matlabDump(f,buf);

  fprintf(f,"%sprevMap = %d;\n", var, prevMap+1);

  if(prevMap >= 0){
    sprintf(buf,"%sregion_prev",var);
    gmap[prevMap].getRegion()->matlabDump(f,buf);
  }

  gmap.animationDump(f,var);
}

void MappingAgent::dumpCurrentState(FILE * f, const char* var)const{
  int i;
  const SLAMParticle *bestP = NULL;
  double max_w = -1.0;

  fprintf(f,"%sid = [ %d ];\n",var,id);
  fprintf(f,"%scurrent_map = [ %d ];\n",var,currentMap+1);
  fprintf(f,"%sodo = [ ...\n",var);
  for(i = 0; i < slam->numParticles(); ++i){
    const SLAMParticle *p = (const SLAMParticle*)slam->get(i);
    if(p->getId() == id){
      fprintf(f,"%+.3e %+.3e %+.3e %.3e\n"
	      ,p->getPose().x
	      ,p->getPose().y
	      ,p->getPose().rot
	      ,p->getWeight());
      if(max_w < p->getWeight()){
	max_w = p->getWeight();
	bestP = p;
      }

    }
  }
  fprintf(f,"];\n");

  if(bestP != NULL){
    RangeMature range;
    SimpleMap *map = bestP->getMap()->getSubMap(range);

    fprintf(f,"%smap = [ ...\n",var);
    int j;
    for(j = 0; j < map->numMap(); ++j){
      const Gaussian2d *m = (const Gaussian2d *) map->get(j);

      fprintf(f,"%.9e %.9e %.9e %.9e %.9e %.9e\n",
	      m->x,
	      m->y,
	      m->cov[0][0],
	      m->cov[0][1],
	      m->cov[1][0],
	      m->cov[1][1]);

    }

    fprintf(f,"];\n");
    delete map;
  }else{
    fprintf(f,"%smap = []\n",var);
  }

  fprintf(f,"%smap_ref = [...\n",var);
  for(i = 0; i < gmap.numMaps(); ++i){
    const RobotPoseCov &ref = gmap.getMapRef(i);
    fprintf(f,"%.3e %.3e %.3e\n"
	    ,ref.x
	    ,ref.y
	    ,ref.rot);

  }
  fprintf(f,"];\n");
}


void MappingAgent::initMappingRegion(){;}

void MappingAgent::initNeighbours(){
  int i;

  //First clean old ones.
  for(i = 0; i < n_neighbours; ++i) delete neighbours[i];

  n_neighbours = 0;

  for(i = 0; i < gmap.numMaps(); ++i){
    if(gmap.isNeighbour(i)){
      neighbours[n_neighbours] = new Neighbour(i,gmap.getRegion(i));
      n_neighbours += 1;
    }
  }

}

int MappingAgent::getFinalPoses(RobotPose *poses, 
                                int *histId){
  int i;
  int np = 0;

  if(superMode){
    np = slam->numParticles();
    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle *p = (SLAMParticle*)slam->get(i);
      poses[i]  = p->getPose();
      histId[i]   = p->getHist();
    }
  }else{
    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle *p = (SLAMParticle*)slam->get(i);
      if(p->getId() == id){
	poses[np]  = p->getPose();
	histId[np]   = p->getHist();
	np += 1;
      }
    }
  }

  return np;
}

void MappingAgent::updateParticleHistory(SLAMParticle** p, int np){
  int i;

  if(mappingNew){
    //Make sure the size if right, otherwise resize
    if(sz_hist <= currentMap){
      int      sz = currentMap + 20;
      PHist **tmp = new PHist*[sz];
      PHist **tmp2 = new PHist*[sz];

      if(hist != NULL){
	memcpy(tmp, hist, sz_hist*sizeof(PHist*));
	delete[] hist;
      }

      if(blocks != NULL){
	memcpy(tmp2, blocks, sz_hist*sizeof(PHist*));
	delete[] blocks;
      }

      hist   = tmp;
      blocks = tmp2;

      memset(hist  + sz_hist, 0, (sz-sz_hist)*sizeof(PHist*));
      memset(blocks + sz_hist, 0, (sz-sz_hist)*sizeof(PHist*));

      sz_hist = sz;
    }

    PHist *h         = new PHist(np);
    blocks[n_blocks]   = h;
    n_blocks += 1;

    hist[currentMap] = h;

    for(i = 0; i < h->n; ++i){
      h->h[i]->setMap(currentMap, i);
    }

  }else{
    PHist    *h_current = hist[currentMap];
    ParticleHistory** h = new ParticleHistory*[np];

    for(i = 0; i < np; ++i){
      int ii = p[i]->getHist();
      //printf(" %d <- %d (%p)\n", i, ii, (*h_current)[ii]);

      h[i] = (*h_current)[ii]->clone();
      h[i]->setMap(currentMap,i);
    }

    h_current->destroy();
    h_current->h = h;
    h_current->n = np;
  }
}

void MappingAgent::updateCurrentMap(){
  SLAMParticle ** p;
  int np;
  bool shouldDelete = false;
  int i;

  if(superMode){
    p = slam->getParticles();
    np = slam->numParticles();
  }else{
    p = new SLAMParticle*[slam->numParticles()];
    np = 0;

    for(i = 0; i < slam->numParticles(); ++i){
      SLAMParticle *particle = (SLAMParticle*)slam->get(i);
      if(particle->getId() == id){
	p[np++] = particle;
      }
    }

    if(np == 0){ 
      printf("Request to update map, but no particles.!!\n");
    }

    shouldDelete = true;
  }


  //Update the local map
  gmap[currentMap].update(p,np);

  //Update the Particle History
  updateParticleHistory(p,np);

  //Clean up.
  if(shouldDelete) delete[] p;
}

void MappingAgent::storePath(){
  if(mon.bestP == NULL){
    ABORT("Error: storePath -- no best particle!\n");
    return;
  }

  const OdometryStore &odo = mon.bestP->getOdo();
  RobotPose poses[odo.numOdo()];
  odo.copyToArray(poses, odo.numOdo());

  int i;
  for(i = 0; i < odo.numOdo(); ++i){
    path.add(new Location(currentMap,poses[i]));
  }

}

bool MappingAgent::dumpPath(FILE* f, const char* name){
  Location *loc[path.numElements()];
  path.getAll((GenericStackType*)loc);
  int i;

  if( fprintf(f,"%s = [...\n", name) < 0) return false;

  for(i = 0; i < path.numElements(); ++i){
    const RobotPose &p = loc[i]->pose;
    if( fprintf(f,"%3d %+.9e %+.9e %+.9e\n"
	    , loc[i]->mapId + 1, p.x, p.y, p.rot) < 0) return false;

  }

  if( fprintf(f,"]; %% %s\n", name) < 0) return false;

  return true;
}


//Adds another decision to the array, keeping
//the order of decreasing weight.
//overwrites the one with lowest weight if array is full already.

void MappingAgent::addDecision(struct MappingAgent::MappingDecision decs,
			       struct MappingAgent::MappingDecision *decs_out,
			       int &ndecs,
			       int nmax){
  int i;
  int put = -1;

  //  printf("Add[%d]: %f\n",ndecs,decs.weight);
  for(i = 0; i < ndecs; ++i){
    if( decs_out[i].weight < decs.weight){
      int j;
      int nlimit = min(nmax-1, ndecs);

      //      printf("Shifting: %d=>%d\n",i,nlimit);
      //Shift it down
      for(j = nlimit; j > i; --j){
	decs_out[j] = decs_out[j-1];
      }

      put = i;
      break;
    }
  }
  
  //  printf("Put to: %d\n",put);
  if(put < 0){//Put it to the end (if there is room) and increase count
    if(ndecs < nmax)
      decs_out[ndecs++] = decs;
  }else{
    decs_out[put] = decs;
    if(ndecs < nmax) ndecs+=1;
  }
}

MappingAgent::PHist::PHist(const MappingAgent::PHist &o):n(o.n),h(NULL){
  if(n > 0){
    h = new ParticleHistory*[n];
    int i;

    for(i = 0; i < n; ++i){
      h[i] = CLONE(o.h[i]);
    }
  }
}

MappingAgent::PHist* MappingAgent::PHist::clone(){
  return new PHist(*this);
}

void MappingAgent::PHist::destroy(){
  if(h != NULL){
    int i;

    for(i = 0; i < n; ++i){
      if(h[i] != NULL) delete h[i];
    }

    h = NULL; n = 0;
  }
}

///////////////////////////////////////////////////////////////////////////
// Misc.
///////////////////////////////////////////////////////////////////////////
const int MAX_MATCH_SIZE = 40;
const int MAX_MATCH_ATTEMPTS = 5;
const int MIN_MATCH = 3;


void linspace(int *out, int from, int to, int step){
  int i;
  for(i = from; i <= to; i += step)
    out[i-from] = i;
}

bool match_maps(LoopCloseResult *res, MapMatcherInterface *matcher,
                const SimpleMap *m1, const SimpleMap *m2
               ,const RobotPoseCov *r2in1
               ,double MD_MAX){

  if(m1->numMap() < MIN_MATCH || m2->numMap() < MIN_MATCH){
    return false;
  }

  printf("...match size %d => %d\n",m1->numMap(), m2->numMap());
  int n_attempts = 0;
  const SimpleMap *M1;
  const SimpleMap *M2;
  SimpleMap *tmp1 = NULL, *tmp2 = NULL;
  int rand_ind1[m1->numMap()];
  int rand_ind2[m2->numMap()];

  bool multiRun = false;
  bool matched = false;

  if(m1->numMap() > MAX_MATCH_SIZE){
    linspace(rand_ind1,0,m1->numMap()-1,1);
    multiRun = true;
  }
  if(m2->numMap() > MAX_MATCH_SIZE){
    linspace(rand_ind2,0,m2->numMap()-1,1);
    multiRun = true;
  }


  do{
    if(m1->numMap() > MAX_MATCH_SIZE){

      shuffle_ind(rand_ind1, m1->numMap(), 100);

      DUMP_INT_ARRAY("...SubSampling M1:",rand_ind1,MAX_MATCH_SIZE);
      DESTROY(tmp1);
      tmp1 = m1->subMap(rand_ind1, MAX_MATCH_SIZE);
      M1 = tmp1;
    }else M1 = m1;

    if(m2->numMap() > MAX_MATCH_SIZE){
      shuffle_ind(rand_ind2,m2->numMap(), 100);
      DUMP_INT_ARRAY("...SubSampling M2:",rand_ind2,MAX_MATCH_SIZE);
      DESTROY(tmp2);
      tmp2 = m2->subMap(rand_ind2, MAX_MATCH_SIZE);
      M2 = tmp2;
    }else M2 = m2;

    n_attempts += 1;
    printf("...matching[%d]:\n",n_attempts);

    if(r2in1 != NULL)
        matcher->match(*M1, *M2, *r2in1);
    else
      matcher->match(*M1, *M2);

    if(matcher->numMatch() >= MIN_MATCH){
      double md2 ;
      res->r2in1 = matcher->getTranslation();

      if(r2in1 != NULL){      
        md2 = r2in1->mahalanobis2(res->r2in1);
      }else{
	md2 = 0;
      }

#if 1
      int i;
      printf("...Matched %d nodes\n", matcher->numMatch());
      printf("...M1 => M2\n");
      for( i = 0; i < M1->numElements(); ++i){
        int i2 = matcher->mapping(i);
	if(i2 < 0){
	  printf("...%03d => none\n", (tmp1 ? rand_ind1[i] : i) + 1);
	}else{
	  printf("...%03d => %03d\n", (tmp1 ? rand_ind1[i] : i) + 1, 
		                      (tmp2 ? rand_ind2[i2]:i2) + 1);
	}
      }
      if(r2in1 != NULL) print_rcov("r2in1",*r2in1);
      print_robot("m2in1",&res->r2in1);
      printf("MD2: %f\n",md2);

#endif

      if(md2 < MD_MAX){
	int ind[matcher->numMatch()];
	int n;
	n = matcher->commonElements1(ind);
	res->map1 = M1->subMap(ind,n);
	n = matcher->commonElements2(ind);
	res->map2 = M2->subMap(ind,n);
	matched = true;
      }
    }

  }while(multiRun && !matched &&
         n_attempts < MAX_MATCH_ATTEMPTS );

  DESTROY(tmp1); DESTROY(tmp2); //Delete if not null and set to null

  return matched;
}

void dumpFinalPose(FILE *f, const RobotPose *pFinal, 
                   const double *w, int np){
  int i;

  for( i = 0; i < np; ++i){
    fprintf(f,"%.9e, %.9e, %.9e, %.9e\n"
	    ,pFinal[i].x
	    ,pFinal[i].y
	    ,pFinal[i].rot
            ,w[i]);
  }

}

void matlabDump(FILE *f, 
                int i1, int i2,
		const RobotPoseCov r1in2,
		const RobotPose *fp, const double *w, int np,
		const Array2d       & path,
		const LoopCloseResult *res){

  fprintf(f,"imap1 = %d;\n",i1+1);
  fprintf(f,"imap2 = %d;\n",i2+1);
  print_rcov(f,"r1in2",r1in2);

  res->map1->matlabDump(f,"map1");
  res->map2->matlabDump(f,"map2");

  int imax = find_max(w,np);
  fprintf(f,"best_path = [");
  int i;
  for(i = 0; i < path.numCols(); ++i)
    fprintf(f," %2d", path.get(imax,i));
  fprintf(f,"];\n");


  fprintf(f,"fp = [\n");
  dumpFinalPose(f,fp,w,np);
  fprintf(f,"];\n");

}

void MappingAgent::evaluatePath(const MapPath *path, const Array2d &pth
		  ,const double *log_w){
#if 0
  const double P_LOOP = 0.2;
  int i,j;
  MapTransition *tr = new MapTransition[path->getNHops()];

  for(i = 0; i < path->getNHops(); ++i){
    tr[i] = *(path->get(i));
    tr[i].setWeight(0.0);
  }

  double log_w_max = log_w[0];
  for(i = 1; i < pth.numRows(); ++i){
    if(log_w_max <= log_w[i]) log_w_max = log_w[i];
  }

  //  printf("Max log: %.9e\n",log_w_max);

  for(i = 0; i < pth.numRows(); ++i){
    register double w = exp(log_w[i] - log_w_max);
    const int *row = pth.getRow_c(i);

    for(j = 0; j < path->getNHops(); ++j){
      //printf("Adding (%d,%d) => %d\n", i,j,pth.get(i,j));
      tr[j].addWeight(row[j],w);
    }
  }

  for(i = 0; i < path->getNHops(); ++i){
    tr[i].normalise();
  }


  for(i = 0; i < path->getNHops(); ++i){
    MapTransition *tmap = gmap.getTransition(tr[i].getMap1(),
					     tr[i].getMap2());

    for(int j=0; j < tr[i].numP(); ++j){
      register double w = tr[i].getWeight(j)*P_LOOP + 
     	                  tmap->getWeight(j)*(1 - P_LOOP);

      tmap->setWeight(j,w);
    }

    tmap->normalise();
    tmap->computeFinalPose();

    //Dodge:
    //    gmap[tr[i].getMap1()].update(tmap);
  }

  delete[] tr;
#endif
}


////////////////////////////////////////////////////////////
// Support  functions
////////////////////////////////////////////////////////////

void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, 
                      double *weight, int nsamples
                      ,int nTrialsMax){

  RobotPoseCov distEstimate(*m2in1);
  RobotPoseSampler sampler(distEstimate);

  int i;

  int Ninit = min(nsamples,500);
  int nTrials = 0;
  double sumW;

  while(nTrials < nTrialsMax){
    sumW = 0.0;
    for(i = 0; i < Ninit; ++i){
      samples[i] = sampler.sample();
      weight[i] = computeWeight(samples + i, m1, m2);
      sumW += weight[i];

#if 0
      printf(" smpl0(%d,:) = [%e %e %e %e];\n"
	     ,i+1
	     ,samples[i].x
	     ,samples[i].y
	     ,samples[i].rot, weight[i]);
#endif

    }
    if(sumW <= 0.0){
      printf("Failed to sample transition. (SumW <= 0.0)\n");
      nTrials = 10000;
    }else{
      //Normalise the weights
      register double sumW_inverse = 1.0/sumW;
      for(i = 0; i < Ninit; ++i) weight[i] *= sumW_inverse;

      //Estimate the gaussian
      weightedMean(&distEstimate,samples,weight,Ninit);
      sampler.set(distEstimate);
      nTrials += 1;
    }
  }

#if 1
  print_rcov("SampleTransition:\n  distEstimate",distEstimate);
#endif

  for(i = 0; i < nsamples; ++i){
    samples[i] = sampler.sample();
    weight[i]  = computeWeight(samples + i,m1,m2);
  }
  normalise(weight,weight,nsamples);

} 


