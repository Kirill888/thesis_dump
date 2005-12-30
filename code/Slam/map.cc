#ifndef DEPEND
#include <string.h>
#include <math.h>
#endif

#include "map.h"
#include "memstat.h"
#include "matlab.h"
#include "debug.h"
#include "slam.h"
#include "mappingProcess.h"


//=============================================================================
//MAP Interface
//=============================================================================

//////////////////////////////////////////////////////////////////////
// Simple Map Class
//////////////////////////////////////////////////////////////////////

SimpleMap::SimpleMap(){
  map = NULL;
  n_map = 0;
}
SimpleMap::SimpleMap(int Nmap){
  n_map = Nmap;
  if(n_map > 0) map   = new MapEntryInterface*[n_map];
  else map = NULL;
}

SimpleMap::SimpleMap(const SimpleMap &other){
  copy(&other);
}

SimpleMap::SimpleMap(const SimpleMap *other){
  copy(other);
}

const SimpleMap & SimpleMap::operator=(const SimpleMap &other){
  if(this != &other){
    destroy();
    copy(&other);
  }
  return *this;
}

void SimpleMap::translateMe(const RobotPose &p){
  int i;
  double ca,sa;
  ca = cos(p.rot); sa = sin(p.rot);

  for(i = 0; i < n_map; ++i) map[i]->translateMe(p.x,p.y,ca,sa);
}

void SimpleMap::translateMe(const RobotPoseCov &p){
  int i;

  for(i = 0; i < n_map; ++i) map[i]->translateMe(p);
}

void SimpleMap::translateMe(const Point &p){
  int i;

  for(i = 0; i < n_map; ++i) map[i]->translateMe(p);
}


SimpleMap* SimpleMap::subMap(const int *ind, int n)const{
  SimpleMap *m = new SimpleMap(n);

  for(int i = 0; i < n; ++i) m->map[i] = map[ind[i]]->clone();

  return m;
}

void SimpleMap::merge(const SimpleMap &other){

  if(other.isEmpty()) return;

  if(isEmpty()){
    copy(&other);
    return;
  }

  //Only get here if both maps are non-empty.
  int i;
  int n = n_map + other.n_map;
  MapEntryInterface **map_ = new MapEntryInterface*[n];

  //Copy its own 
  for(i = 0; i < n_map; ++i) map_[i] = map[i];

  //Clone other
  for(i = n_map; i < n; ++i) map_[i] = other.map[i - n_map]->clone();

  delete[] map;
  map = map_;
  n_map = n;

}

double SimpleMap::probMatchLog(const SimpleMap &m)const{
  double sumW = 0.0;
  int i;

  for(i = 0; i < n_map; ++i ) sumW += map[i]->probMatchLog(m.get(i));

  return sumW;
}


void SimpleMap::copy(const SimpleMap *other){
  n_map = other->n_map;
  if(n_map == 0){
    map = NULL;
  }else{
    map = new MapEntryInterface*[n_map];
    int i;

    for( i = 0; i < n_map; ++i)
      map[i] = other->map[i]->clone();
  }
}

void SimpleMap::destroy(){
  if(map != NULL){
    for(int i = 0; i < n_map; ++i){
      if(map[i] != NULL) delete map[i];
    }

    delete[] map;
    map = NULL;
    n_map = 0;
  }
}

void SimpleMap::matlabDump(FILE *f, const char* name)const{
  //SPAM: assumes it is a map for PointLandmarks

  if(n_map > 0){
    fprintf(f,"%s = [\n",name);
    int i;
    for(i = 0; i < n_map; ++i){
      map[i]->matlabDump(f);
      fprintf(f,"\n");
    }

    fprintf(f,"]; %%%s\n",name);
  }else{
    fprintf(f,"%s = zeros(0,2);\n",name);
  }
}

//============================================================================
// MapBuilderInterface
//============================================================================

const SensorRangeInterface MapBuilderInterface::DefaultSensorRange;

//=============================================================================
//Map::SubMAP
//=============================================================================
const int Map::SubMap::SIZE_INCREMENT = 10;

Map::SubMap::SubMap(const Map::SubMap &other){

  nElements = other.nElements;
  arraySize = other.arraySize;
  parent    = other.parent;

  elements = new MapEntryInterface*[arraySize];
  map_index = new int[arraySize];

  memcpy(elements,other.elements,nElements*sizeof(MapEntryInterface *));
  memcpy(map_index,other.map_index,nElements*sizeof(int));

  //MEM_STAT_ADD(subMap);
}

Map::SubMap::SubMap(int sz,Map *p){
  elements = new MapEntryInterface*[sz];
  map_index = new int[sz];
  arraySize = sz;
  nElements = 0;
  parent = p;

  //MEM_STAT_ADD(subMap);
}


Map::SubMap::~SubMap(){
  if(map_index != NULL) delete[] map_index;
  if(elements  != NULL) delete[] elements;

  //MEM_STAT_REMOVE(subMap);
}

const Map::SubMap & Map::SubMap::operator=(const Map::SubMap &other){

  if(&other == this) return *this;

  if(map_index != NULL) {
    delete[] elements;
    delete[] map_index;
  }

  nElements = other.nElements;
  arraySize = other.arraySize;

  elements = new MapEntryInterface*[arraySize];
  map_index = new int[arraySize];

  memcpy(elements,other.elements,nElements*sizeof(MapEntryInterface *));
  memcpy(map_index,other.map_index,nElements*sizeof(int));

  return *this;
}

void Map::SubMap::add(MapEntryInterface *m, int ind){
  if(nElements >= arraySize){ //Grow the buffer
    arraySize += SIZE_INCREMENT;

    MapEntryInterface **e  = new MapEntryInterface*[arraySize];
    int *mi = new int[arraySize];

    if(nElements > 0){
      memcpy(e,  elements,  nElements*sizeof(MapEntryInterface *));
      memcpy(mi, map_index, nElements*sizeof(int));
      delete[] elements;
      delete[] map_index;
    } 

    map_index = mi;
    elements  = e;
  }

  map_index[nElements] = ind;
  elements[nElements]  = m;
  nElements += 1;
}


//============================================================================
//MAP
//============================================================================


void Map::init(const SimpleMap *map){
  int i;

  reset();
  //  printf("Map::init\n");
  for(i = 0; i < map->numMap(); ++i){
    //printf("...state[%d] = %d\n",i,map->get(i)->getState());
    add(map->get(i)->clone());
  }

}

void Map::destroyTree(struct TreeNode *n){
  if(n != NULL){

    if(n->copyCount > 1){
      n->copyCount -= 1;
    }else{
      destroyTree(n->left);
      destroyTree(n->right);

      delete n;
    }
  }
}

void Map::set(int ind, MapEntryInterface *e){

  //  DEBUG_error("set(%d)\n",ind);

  //Check if we need to add another layer(s).
  while(ind >= nLeafs){
    if( top != NULL){
      top = new struct TreeNode(top, NULL, nLeafs, NULL);
      nLeafs   *= 2;
      topLevel += 1;
	
      //	DEBUG_error("Adding Top Layer!\n");
    }else{
      top = new struct TreeNode(NULL, NULL, 1, NULL);
      nLeafs    = 2;
      topLevel += 1;

      //	DEBUG_error("Creating root node!\n");
    }
  }

  struct TreeNode *n = top;
  struct TreeNode **link = &top;
  int level = topLevel;
  int index = n->index;

  //  DEBUG_error("set start %d(%d)\n", level, index);

  while( n != NULL){
    
    if( n->copyCount > 1){ //Found shared node

      //Replace shared node and the whole branch
      //  leading to ind.

      n->copyCount -= 1;

      (*link) = createBranch(n, level, index, ind, e);
      return;

    }else{
      //So far all of the branch belongs to us,

      if(level < 0){ //Reached ground level
	//	 DEBUG_error("Reached ground level %d(%d), setting data.\n",
	//	             level, index);

	//Set data and quit.
	if(n->data != NULL) delete n->data;

	n->data = e;
	return;
      }else{
	//Traverse one level down.

	level -= 1;

	if( ind < n->index){ //Traverse left
	  link = &(n->left);
	  n = n->left;

	  index -= (1<<level);

	  //	   DEBUG_error("Steping Down Left: %d (%d)\n",level,index);
	}else{               //Traverse right
	  link = &(n->right);
	  n = n->right;

	  index += (1<<level);
	  //	   DEBUG_error("Steping Down Right: %d (%d)\n",level,index);
	}

      }

    }
    
  }//end while

  //  DEBUG_error("Hit the NULL branch on level %d (%d)\n",level,index);

  //If we got here we must have hit the NULL pointer.
  (*link) = createBranch(NULL, level, index, ind, e);

}

struct Map::TreeNode* Map::createBranch(struct TreeNode *copy,
					int startLevel, int startIndex,
					int leafIndex,
					MapEntryInterface *e){
  
  struct TreeNode *root = new struct TreeNode(NULL, NULL, startIndex, NULL);
  struct TreeNode *n = root;

  int level = startLevel;

//   DEBUG_error("Create Branch: level = %d(%d) => %d\n"
//               ,startLevel, startIndex, leafIndex);

  while(level >= 0){
    level -= 1;

    if(leafIndex < n->index){ //Grow left
      if(copy != NULL){
	n->right = copy->right;
	if(n->right != NULL) n->right->copyCount += 1;

	copy     = copy->left;
      }

      n->left = new struct TreeNode(NULL, NULL, n->index - (1<<level),NULL);
      n = n->left;

      //	 DEBUG_error("Grow Left %d(%d)\n", level, n->index);

    }else{                    //Grow right
      if(copy != NULL){
	n->left = copy->left;
	if(n->left != NULL) n->left->copyCount += 1;

	copy    = copy->right;
      }

      n->right = new struct TreeNode(NULL, NULL, n->index + (1<<level),NULL);
      n = n->right;

      //	 DEBUG_error("Grow Right %d(%d)\n", level, n->index);
    }

  }

  n->data = e;
  n->index = leafIndex;

  //   DEBUG_error("Setting Data %d(%d)\n",level, n->index);

  return root;
}

const MapEntryInterface* Map::get(int ind)const{

  if(ind < 0 || ind >= nElements){
    DEBUG_error("Trying to access element %d, valid range 0->%d\n",
		ind, nElements - 1);

    return NULL;
  }

  struct TreeNode *n = top;
  int i;

  for(i = topLevel; i >= 0 ; i -= 1){
    if( ind < n->index)
      n = n->left;
    else
      n = n->right;
  }

  return n->data;
}

void Map::forEach(MapIterator &iterator)const{
  forEach(top,iterator);
}

bool Map::forEach(const struct TreeNode *n, MapIterator &iterator){
  if(n == NULL) return true;

  if(n->data != NULL) return iterator.process(n->data, n->index);

  if( forEach(n->left, iterator) ) return forEach(n->right, iterator);
  else return false;
}

extern RobotPose robot2laser(const RobotPose &);

SimpleMap * Map::getSubMap(const RangeInterface &range)const{
  SubMap *sub = new SubMap(10,NULL);
  findSubmap(top, sub, &range);

  int i;
  SimpleMap *map = new SimpleMap(sub->numElements());

  for(i = 0; i < sub->numElements(); ++i){
    map->set(i, sub->get(i));
  }

  delete sub;

  return map;
}


MapInterface* Map::findSubmap(const RangeInterface *range){
  SubMap *sub = new SubMap(10,this);
  findSubmap(top, sub, range);
  return sub;
}

void Map::findSubmap(const struct TreeNode *n, Map::SubMap *visibleMap, 
                     const RangeInterface *range){
  if(n == NULL) return;

  if(n->data != NULL){
    if(range == NULL || range->isWithinRange(n->data)){
      visibleMap->add(n->data, n->index);
    }

    return;
  }

  findSubmap(n->left,  visibleMap, range);
  findSubmap(n->right, visibleMap, range);
}

MapInterface* Map::flatten(){
  return findSubmap(NULL);
}

void Map::update(int map_ind,
		   const ObservationStore &obs_store,
		   int obs_ind, 
		   const RobotPose *robot_pose){
  MapEntryInterface *e = get(map_ind)->clone();
  bool wasMature = e->isMature();

  e->update(obs_store, obs_ind, robot_pose);

  if(!wasMature && e->isMature()) nMature+=1;

  set(map_ind, e);
}

void Map::update(int map_ind, MapEntryInterface **map_entry,
		   const ObservationStore &obs_store,
		   int obs_ind, 
		   const RobotPose *robot_pose){

  MapEntryInterface *e = (*map_entry)->clone();
  bool wasMature = e->isMature();

  e->update(obs_store, obs_ind, robot_pose);

  if(!wasMature && e->isMature()) nMature+=1;

  set(map_ind, e);

  (*map_entry) = e;
}

//============================================================================
// MapBuilder
//============================================================================

const double MapBuilder::MAX_DIST2 = POW2(3);

double MapBuilder::update(MapInterface* m,
                          const ObservationStore &obs_store,
			  const RobotPose *robot_pose,
			  SLAMParticle *p,
			  SLAMMonitor* monitor)const{

  int nobs = obs_store.numObsLastScan();
  if(nobs <= 0) return 0.0;

  Map* map = (Map*) m;

  int corr[nobs];
  double Sw;
  int i;

  //For now consider the whole map
  RangeInterface *range = 
      mappingProcess->getNeighbourhoodRange(robot_pose);

  MapInterface *submap = map->findSubmap(range);

  DESTROY(range);

  Sw = mappingProcess->dataAssociate(corr, obs_store.getLastScan()
                                         , obs_store.numObsLastScan()
					 , robot_pose, submap);

  for(i = 0; i < nobs; ++i){
    int mapi = corr[i];

    if(mapi >= 0){ //Matched old one

      if(monitor) monitor->observe(p,obs_store.scan2Ind(i),submap->get(mapi));

      submap->update(mapi, obs_store, obs_store.scan2Ind(i), robot_pose);

    }else{            //New Map Element
      double probLog = 0;

      if(monitor) monitor->noMatch(p,obs_store.scan2Ind(i));

      MapEntryInterface *m = mappingProcess->makeNewEntry(&probLog, 
                                                   obs_store, 
                                                   obs_store.scan2Ind(i), 
                                                   robot_pose);
      if(isnan(probLog)){
	ABORT("ISNAN: while adding new element.\n");
      }

      if(mapArea->probIsInside(m) > 0.9){
	m->setCore();
      }else{
	m->setPeripheral();
      }

      Sw += probLog;

      map->add(m);
    }
  }

  delete submap;
//   DEBUG_print("Map size: %d\n",map->numElements());

  return exp(Sw);
}




//============================================================================
// MapPrinter
//============================================================================

bool MapPrinter::process(const MapEntryInterface *ee, int ind){
  if(f == NULL){
    err = true;
    return false;
  }

  if(print_callback != NULL){
    return print_callback(f,ind,ee);
  }

  return false;
}

bool MapPrinter::print(const char *fname, const MapInterface *map){
  f = fopen(fname,"w");
  if(f == NULL) return false;

  err = false;

  map->forEach(*this);

  fclose(f);
  
  f = NULL;

  return !err;
}

bool MapPrinter::print(FILE *fp, const MapInterface *map){
  f = fp;

  err = false;

  map->forEach(*this);

  f = NULL;

  return !err;
}
