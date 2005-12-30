#ifndef DEPEND
#include <stdio.h>
#include <string.h>
#include <math.h>
#endif

#include "globalmap.h"
#include "util.h"
#include "mappingProcess.h"

#define ERR(msg) fprintf(stderr,msg);

static const int MAP_SZ_INCREMENT   = 100;
static const int TRANS_SZ_INCREMENT = 100;

extern MappingProcessInterface *mappingProcess;

////////////////////////////////////////////////////////////////////////
//  Map Transition
///////////////////////////////////////////////////////////////////////

void MapTransition::set(int m1, int m2, 
			const RobotPose* p2in1,
			const double *weight,
			int n){
  destroy();

  map1 = m1;
  map2 = m2;

  np = n;
  w = new double[np];
  w_log = new double[np];
  w_cumsum = new double[np];
  biSearch.set(w_cumsum,np);

  p = new RobotPose[np];

  memcpy(w, weight,np*sizeof(double));
  memcpy(p, p2in1, np*sizeof(RobotPose));

  normalise();
  computeFinalPose();

#if 0
  printf("Transition::set(%d =>%d, np=%d)\n",m1+1,m2+1,n);
  print_rcov("  r2in1",r2in1);
  print_rcov("  r1in2",r1in2);
#endif
}

void MapTransition::set(int m1, int m2, 
			const RobotPose* p2in1,
			int n){
  destroy();

  map1 = m1;
  map2 = m2;

  np = n;
  w = new double[np];
  w_log = new double[np];
  w_cumsum = new double[np];
  biSearch.set(w_cumsum,np);

  p = new RobotPose[np];

  int i;

  //Set all equal
  double w0 = 1.0/double(np);
  double w0_log = log(w0);

  double sum = 0;

  for(i = 0; i < np; ++i){
    w[i] = w0;
    w_log[i] = w0_log;
    sum += w0;
    w_cumsum[i] = sum;
  }

  memcpy(p, p2in1, np*sizeof(RobotPose));

  computeFinalPose();

#if 0
  printf("Transition::set(%d =>%d, np=%d)\n",m1+1,m2+1,n);
  print_rcov("  r2in1",r2in1);
  print_rcov("  r1in2",r1in2);
#endif
}


void MapTransition::set(const MapTransition &other){
  destroy();

  map1 = other.map1;
  map2 = other.map2;
  np   = other.np;

  if(np > 0){
    w = new double[np];
    w_cumsum = new double[np];
    w_log = new double[np];
    p = new RobotPose[np];

    memcpy(w,        other.w, np*sizeof(double));
    memcpy(w_log,    other.w_log, np*sizeof(double));
    memcpy(w_cumsum, other.w_cumsum, np*sizeof(double));

    memcpy(p, other.p, np*sizeof(RobotPose));

    biSearch.set(w_cumsum,np);
  }
 
  r2in1 = other.r2in1;
  r1in2 = other.r1in2;
}

void MapTransition::normalise(){
  if(np <= 0) return;

  register int i;
  register double sum = 0.0;

  for(i = 0; i < np; ++i) sum += w[i];

  sum = 1.0/sum;

  w[0] *= sum;
  w_cumsum[0] = w[0];
  w_log[0] = log(w[0]);

  for(i = 1; i < np; ++i){
    w[i] *= sum;
    w_cumsum[i] = w_cumsum[i-1] + w[i];
    w_log[i] = log(w[i]);
  }
}

void MapTransition::sample(int *ind_out, int nsample)const{
  sample_cumsum(ind_out, w_cumsum, np, nsample);
}

void MapTransition::computeFinalPose(){

  if(np <= 0) return;

  //  weightedMean(&r2in1,p,w,np);
  robotPoseMean(&r2in1,p,np);

  r1in2.set(-r2in1.x, -r2in1.y, 0.0, r2in1.cov);
  r1in2.translateMe(0.0, 0.0, -r2in1.rot);
}

void MapTransition::invert(){
  int i;
  double rot;

  for(i = 0; i < np; ++i){
    rot = p[i].rot;
    p[i].set(-p[i].x, -p[i].y, 0.0);
    p[i].translateMe(0,0,-rot);
  }

  int tmp = map1;
  map1 = map2;
  map2 = tmp;

  RobotPoseCov tmp2 = r2in1;
  r2in1 = r1in2;
  r1in2 = tmp2;
}

double MapTransition::computeEntropy()const{
  double e = 0.0;

  int i;

  for(i = 0; i < np; ++i){
    if(w[i] != 0.0) e += w[i]*w_log[i];
  }

  e /= (-1.0*np);

  return e;
}


void MapTransition::transit(RobotPose *r, int source_map, int ind)const{
  const RobotPose &p0 = p[ind];
  register double ca = cos(p0.rot);
  register double sa = sin(p0.rot);

  register double x = r->x;
  register double y = r->y;

  if(source_map == map1){
    x -= p0.x;
    y -= p0.y;

    r->x =  x*ca + y*sa;
    r->y = -x*sa + y*ca;
    r->rot = angleDiffRad(r->rot, p0.rot);
  }else{
    r->x    = +x*ca - y*sa + p0.x;
    r->y    = +x*sa + y*ca + p0.y;
    r->rot +=  p0.rot;
  }
}
void MapTransition::propagate(RobotPoseCov *r, int source_map)const{
  if(source_map == map1){
    r->propagateMe(r1in2);
  }else{
    r->propagateMe(r2in1);
  }
}

bool MapTransition::store(FILE *f, int id)const{
  int i;

  for(i = 0; i < np; ++i){
    if( fprintf(f,"%d, %.9e, %9e, %9e, %.9e\n"
	    ,id
	    ,p[i].x
	    ,p[i].y
	    ,p[i].rot
	    ,w[i])              < 0) return false;
  }
  return true;
}


bool MapTransition::load(FILE *f){
  MatlabVariable imap, transitions;

  if(imap.read(f)        != MatlabVariable::noError ||
     transitions.read(f) != MatlabVariable::noError){

    ERR("Failed to read matlab file.\n");
    return false;
  }

  if(imap.numCols() != 2 ||
     transitions.numCols() != 4){    
    ERR("Wrong matrix size.\n");
    return false;
  }

  //First clean up.
  destroy();

  map1 = (int)imap(0,0);
  map2 = (int)imap(0,1);

  np = transitions.numRows();
  w = new double[np];
  w_cumsum = new double[np];
  w_log = new double[np];
  biSearch.set(w_cumsum,np);


  p = new RobotPose[np];

  int i;
  for(i = 0; i < np; ++i){
    p[i].x   = transitions(i,0);
    p[i].y   = transitions(i,1);
    p[i].rot = transitions(i,2);

    w[i]     = transitions(i,3);
  }

  normalise();

  //Compute Gaussian approximation,
  computeFinalPose();

  return true;
}


void MapTransition::destroy(){
  if(w != NULL)        delete[] w;
  if(w_cumsum != NULL) delete[] w_cumsum;
  if(w_log != NULL)    delete[] w_log;
  if(p != NULL)        delete[] p;

  p = NULL;
  w = NULL;
  w_cumsum = NULL;
  w_log = NULL;
  np = 0;
}


///////////////////////////////////////////////////////////////////////
// MapPath
///////////////////////////////////////////////////////////////////////

//Path structure:
//--------------
//  Rows -- number of hops, getNHops()
//  Cols -- number of particles (say 10,000)
void MapPath::samplePath(Array2d &path_out)const{
  int r;

  for( r = 0; r < path_out.numRows(); ++r ){
    int *row = path_out.getRow(r);
    transitions[r]->sample(row, path_out.numCols());
    shuffle_ind(row,path_out.numCols(),path_out.numCols()*2);
  }
}

void MapPath::samplePath(Array2d &path_out, double *w_log)const{
  int r,c;

  for( r = 0; r < path_out.numRows(); ++r ){ //For every particle.
    w_log[r] = 0.0;

    for( c = 0; c < n; ++c){                //For every transition.
      int ind = transitions[c]->sample();      
      path_out(r,c) = ind;
      w_log[r] += transitions[c]->getLogWeight(ind);
    }
  }
}


//Compute pose from the path,
//  p   -- should be large enough to hold all partucles path.numCols();
// path -- see samplePath
void MapPath::path2pose(RobotPose *p, const Array2d &path)const{
    int map = startMap;
    int j,i;

    for( i = 0 ; i < n; ++i){
      for( j = 0; j < path.numCols(); ++j){
         transitions[i]->transit(p+j, map, path.get(i,j));
      }
      map = transitions[i]->transit(map);
    }
}

bool MapPath::matlabDump(FILE*f, const char * var)const{
  int i,j;
  fprintf(f,"%s = [...\n",var);

  for(i = 0; i < n; ++i){
    const MapTransition* t = transitions[i];

    for(j = 0; j < t->numP(); ++j){
      fprintf(f,"%.8e %.8e %.8e %.8e %d %d\n"
	      ,t->get(j).x
	      ,t->get(j).y
	      ,t->get(j).rot
	      ,t->getWeight(j)
	      ,t->getMap1()
	      ,t->getMap2());
    }
  }

  fprintf(f,"];\n");
  return true;
}

////////////////////////////////////////////////////////////////////////
// Global Map
///////////////////////////////////////////////////////////////////////
static const char *MAP_FNAME_MASK    = "%s_%03d.m";
static const char *MAP_FNAME_MASK2   = "%s_%03d";


GlobalMap::GlobalMap(){
  nullAll();
}

void GlobalMap::nullAll(){
  sz_map = 0;
  n_map = 0;

  maps           = NULL;
  mapRef         = NULL;
  path2map       = NULL;
  trans2map      = NULL;
  regions        = NULL;
  is_neighbour   = NULL;
  neighbours     = NULL;
  n_neighbours   = 0;
  pred           = NULL;
  travesal_order = NULL;

  n_trans = 0;
  sz_trans = 0;
  transition = NULL;

  currentRef = -1;
}

void GlobalMap::add(LocalMap *lmap){

  if(n_map >= sz_map) resize(sz_map + MAP_SZ_INCREMENT);

  maps[n_map] = lmap;
  n_map += 1;
}



//Add transtion to the map
void GlobalMap::addTransition(MapTransition *trans){
  if(sz_trans <= n_trans){
    //Do resizing;
    int sz = sz_trans +  TRANS_SZ_INCREMENT;
    MapTransition **tr = new MapTransition*[sz];

    if(transition != NULL){
       memcpy(tr,transition,n_trans*sizeof(MapTransition*));
       delete[] transition;
    }

    transition = tr;
    sz_trans = sz;
  }

  transition[n_trans] = trans;
  nodes2edge(trans->getMap1(),trans->getMap2()) = n_trans;

  n_trans += 1;
}


void GlobalMap::getNeighbouringRegions(const MapAreaInterface ** out)const{
  int i;
  for(i = 0; i < n_neighbours; ++i)
    out[i] = regions[neighbours[i]];
}

void GlobalMap::updateCoreLandmarks(int mapId, bool ignoreCore){
}

void GlobalMap::animationDump(FILE *f, const char* prefix)const{
  int i;
  fprintf(f,"%slinks = [ ...\n",prefix);

  for(i = 0; i < n_trans ; ++i){
    RobotPoseCov fp = transition[i]->getR2In1();

    fprintf(f,"%d,%d,%.9e,%.9e,%.9e,"
	    "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n"
	    , transition[i]->getMap1()+1, transition[i]->getMap2()+1
	    , fp.x, fp.y, fp.rot
	    , fp.cov[0][0], fp.cov[0][1], fp.cov[0][2]
	    , fp.cov[1][0], fp.cov[1][1], fp.cov[1][2]
	    , fp.cov[2][0], fp.cov[2][1], fp.cov[2][2] );

  }
  fprintf(f,"];\n");
}

bool GlobalMap::store(const char *out_prefix ,
             bool (*store_local_map)(FILE*, const LocalMap*) )const{
  char fname[1024];
  int i;
  FILE *f;


  //Store local maps
  for(i = 0; i < n_map; ++i){
    sprintf(fname,MAP_FNAME_MASK, out_prefix, i + 1);

    f = fopen(fname,"w");

    if(f == NULL){
      fprintf(stderr,"Error opening file:%s\n", fname);
      return false;
    }

    if( ! store_local_map(f, maps[i])) { 
      fclose(f);
      return false;
    }
    fclose(f);
  }

  sprintf(fname,"%s_links.m",out_prefix);
  f = fopen(fname,"w");

  if(f == NULL){
    fprintf(stderr,"Error opening file:%s\n", fname);
    return false;
  }


  bool res = fprintf(f,"sz_map = [%d,%d];\n", n_map, n_trans) > 0;
  
  res &= fprintf(f,"links = [ ...\n") > 0;

  for(i = 0; i < n_trans ; ++i){
    if(res){
      RobotPoseCov fp = transition[i]->getR2In1();

      res = fprintf(f,"%d,%d,%.9e,%.9e,%.9e,"
		    "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n"
		    , transition[i]->getMap1()+1, transition[i]->getMap2()+1
		    , fp.x, fp.y, fp.rot
		    , fp.cov[0][0], fp.cov[0][1], fp.cov[0][2]
		    , fp.cov[1][0], fp.cov[1][1], fp.cov[1][2]
		    , fp.cov[2][0], fp.cov[2][1], fp.cov[2][2] ) > 0;

    }else{
      fclose(f);
      return false;    
    }
  }

  res &= fprintf(f,"]; %%end of links\n") > 0;

  const char *f_prefix = rindex(out_prefix,'/');
  if(f_prefix == NULL) f_prefix = out_prefix;
  else f_prefix += 1;


  //Store transitions
  res &= fprintf(f,"transitions = [ ...\n") > 0;

  for(i = 0; res && i < n_trans; ++i){
    res = transition[i]->store(f,i+1);
  }

  res &= fprintf(f,"]; %%end of transitions\n") > 0;

  //Store map file names
  res &= fprintf(f,"map_files = { ...\n") > 0;
  for(i = 0; res && i < n_map ; ++i){
    sprintf(fname, MAP_FNAME_MASK2, f_prefix, i + 1);
    res = fprintf(f,"'%s'\n",fname) > 0;
  }

  res &= fprintf(f,"}; %%end of map_files\n") > 0;

  //Store uncertainties of maps in current reference frame
  res &= fprintf(f,"map_poses = [ ...\n") > 0;
  for(i = 0; res && i < n_map ; ++i){
    res = fprintf(f,"%.9e,%9e,%9e,%9e,%9e,%9e,%9e,%9e,%9e,%9e,%9e,%9e\n"
		  ,mapRef[i].x
		  ,mapRef[i].y
		  ,mapRef[i].rot
		  ,mapRef[i].cov[0][0]
		  ,mapRef[i].cov[0][1]
		  ,mapRef[i].cov[0][2]
		  ,mapRef[i].cov[1][0]
		  ,mapRef[i].cov[1][1]
		  ,mapRef[i].cov[1][2]
		  ,mapRef[i].cov[2][0]
		  ,mapRef[i].cov[2][1]
		  ,mapRef[i].cov[2][2]

		  ) > 0;
  }
  res &= fprintf(f,"]; %%end of map_poses\n") > 0;

  fclose(f);

  return res;
}

bool GlobalMap::load(const char* prefix){
  FILE *f;
  char fname[1024];

  sprintf(fname,"%s_links.m", prefix);
  f = fopen(fname,"r");

  if(f == NULL){
    ERR("Can't open _links.m file\n");
    return false;
  }

  MatlabVariable m_sz;
  MatlabVariable m_links;
  MatlabVariable m_trans;

  if( m_sz.read(f)    != MatlabVariable::noError ||
      m_links.read(f) != MatlabVariable::noError ||
      m_trans.read(f) != MatlabVariable::noError){
    ERR("Failed to read matrices\n");
    return false;
  }

  if( m_sz.numCols() != 2    || 
      m_links.numCols() < 2  ||
      m_trans.numCols() != 5){
    ERR("Wrong dimensions of the matrix.");
    return false;
  }

  destroy();

  resize((int)m_sz(0,0));

  int i;

  //Load local maps
  for(i = 0; i < sz_map; ++i){
    sprintf(fname,MAP_FNAME_MASK,prefix,i+1);
    maps[i] = new LocalMap();

    if(!maps[i]->load(fname)){
      fprintf(stderr,"Failed to load map %d (%s)\n",i+1,fname);
      delete maps[i];
      maps[i] = NULL;
      return false;
    }

    n_map += 1;
  }

  //Load transitions
  sz_trans = (int)m_sz(0,1);
  transition = new MapTransition*[sz_trans];

#if 0
  for( i = 0; i < sz_trans; ++i){
    sprintf(fname,TRANS_FNAME_MASK,prefix,i+1);
    transition[i] = new MapTransition();

    if(!transition[i]->load(fname)){
      fprintf(stderr,"Failed to load transition %d (%s)\n",i+1,fname);
      delete transition[i];
      transition[i] = NULL;
      return false;
    }

    nodes2edge(transition[i]->getMap1(),transition[i]->getMap2()) = i;

    n_trans += 1;
  }
#else
  //TODO: Process m_links and m_trans

#endif

  fclose(f);

  return true;
}

void GlobalMap::computeBFT(int ref){
  FIFO list(n_map);
  int marked[n_map];
  int i;
  for(i = 0; i < n_map; ++i){
    marked[i] = false;
    if(path2map[i] != NULL){
      delete path2map[i];  path2map[i] = NULL;
    }
    if(trans2map[i] != NULL){
      delete trans2map[i]; trans2map[i] = NULL;
    }
  }

  marked[ref]    = true;
  mapRef[ref]    = RobotPoseCov(); //set to 0,0,0 100% certainty
  path2map[ref]  = new MapPath(0,ref,ref); //Identity path
  //  trans2map[ref] = NULL;

  list.add(ref);

  while(!list.isEmpty()){
    int parent = list.remove();
    //Process
    //nothing to be done here
    //    printf("Processing node: %d\n", parent +1);

    //Add unmarked children for processing
    for(i = 0; i < n_map; ++i){
      if(i != parent){
	int edge = nodes2edge(i, parent);
	if( !marked[i] && edge >= 0){
	  path2map[i] = new MapPath(*path2map[parent], transition[edge], edge);
	  mapRef[i]   = transition[edge]->getRef(i).propagate(mapRef[parent]);

	  //mark
	  marked[i] = true;

	  //Add to the list for future processing
	  list.add(i);

	  //printf("====> Adding %d\n",i+1);

	  trans2map[i] = mergeTransitions(trans2map[parent]
                                         ,transition[edge],i);
	}
      }
    }
  }
}

void GlobalMap::computeShortestPath(int ref){
  int marked[n_map];
  int i;
  double *dist = new double[n_map];

  for(i = 0; i < n_map; ++i){
    marked[i] = false;
    dist[i]   = Inf;
    pred[i]   = -1;

    if(path2map[i] != NULL){
      delete path2map[i];  path2map[i] = NULL;
    }
    if(trans2map[i] != NULL){
      delete trans2map[i]; trans2map[i] = NULL;
    }
  }

  mapRef[ref] = RobotPoseCov();
  dist[ref]   = 0;
  pred[ref]   = ref;

  int n  = 0;
  int c  = ref;


  do{
    marked[c]         = true;
    travesal_order[n] = c;
    n  += 1;

    if(n >= n_map) break;

    //Relaxation step
    //  Compute distance for all nodes adjacent to c.
    //   Update distance and predecessor if distance is less than current.

    for(i = 0; i < n_map; ++i){	
      if(!marked[i]){
	int edge = nodes2edge(i, c);

	if(edge >= 0){ //For every adjacent node
	  RobotPoseCov p = transition[edge]->getRef(i).propagate(mapRef[c]);
	  double d = p.norm();

	  if(d < dist[i]){
	    dist[i]   = d;
	    pred[i]   = c;
	    mapRef[i] = p;
	  }
	}
      }
    }

    //Find closest of the rest
    double d_min = Inf;
    int i_min = -1;

    for(i = 0; i < n_map; ++i){
      if(!marked[i] && d_min > dist[i]){
	i_min = i;
	d_min = dist[i];
      }
    }

    c = i_min;

  }while(1);

  //Assign Paths and Transitions

  path2map[ref] = new MapPath(0,ref,ref);
  printf("BFT: %d ", ref);
  for(i = 1; i < n_map; ++i){
    int node = travesal_order[i];
    int p    = pred[node];
    int edge = nodes2edge(node,p);
    path2map[node] = new MapPath(*path2map[p], transition[edge], edge);
    trans2map[node] = mergeTransitions(trans2map[p], transition[edge], node);

    printf(" %d(%d)",node,p);
  }

  printf("\n");

  delete[] dist;
}


//Sets current reference frame to the reference frame of map i,
//compute relative locations of all other maps relative
//to this one.
void GlobalMap::setReferenceFrame(int ref){

  currentRef = ref;

  //For now just do Breadth First Traversal
  //computeBFT(ref);
  computeShortestPath(ref);

  computeNeighbours();

  int i;
  for(i = 0; i < n_map; ++i){
    if(i!=ref) trans2map[i]->invert();
  }

#if 0
  printf("%%setReferenceFrame %d\n",currentRef+1);
  printf("maps = [\n");
  for(i = 0; i < n_map; ++i){
    RobotPoseCov m2_inRef = trans2map[i]->getR2In1();
    print_rcov(NULL, m2_inRef);
  }
  printf("];\n");

  printf("odo = [\n");
  for(i = 0; i < n_map; ++i){
    RobotPose m2_inRef = trans2map[i]->get(0);
    print_robot(NULL, &m2_inRef);
  }
  printf("];\n");
#endif
}

MapTransition* GlobalMap::mergeTransitions(const MapTransition* parent,
				           const MapTransition* trans,
                                           int m_dest)const{
  if(parent == NULL){
    if(m_dest == trans->getMap1()){
      return new MapTransition(*trans);
    }else{
      MapTransition *t = new MapTransition(*trans);
      t->invert();
      return t;
    }
  }else{
    RobotPose pose[parent->numP()];
    double w[parent->numP()];
    int i;

    for(i = 0; i < parent->numP(); ++i){
      parent->get(pose+i, i);
      w[i] = parent->getWeight(i);
      int ind;

      ind   = trans->sample();
      w[i] *= trans->getWeight(ind);
      trans->transit(pose+i, parent->getMap1(), ind);
    }

    return new MapTransition(m_dest,currentRef,pose,w,parent->numP());
  }
}


void GlobalMap::computeNeighbours(){
  int i;
  n_neighbours = 0;

  for(i = 0; i < n_map; ++i){
    const MapAreaInterface* reg = maps[i]->getRegion();
    //    printf("GLOBALMAP_%p[%d]: %p\n",this, i,reg); fflush(stdout);

    if(i != currentRef && 
       mappingProcess->isNeighbour(getNHops(i), mapRef + i, reg) ){

      is_neighbour[i] = true;

      if(regions[i]!=NULL) delete regions[i];
      regions[i] = reg->clone();
      regions[i]->translateMe(mapRef[i]);

      neighbours[n_neighbours] = i;
      n_neighbours += 1;
    }else{
      is_neighbour[i] = false;
      DESTROY(regions[i]); //delete and set to NULL
    }
  }

  regions[currentRef] = maps[currentRef]->getRegion()->clone();
}

SimpleMap* GlobalMap::neighbourhoodMap()const{
  SimpleMap* map = new SimpleMap();
  int i;

  for(i = 0; i < n_map; ++i){
    if(is_neighbour[i]){
      SimpleMap* m = maps[i]->getMap()->clone();
      m->translateMe(mapRef[i]);

      map->merge(m);

      delete m;
    }
  }

  for(i = 0; i < map->numElements(); ++i){
    map->get(i)->setNeighbour();
    map->get(i)->setNew();
    map->get(i)->setNumObs(0); 
  }

  return map;
}

void GlobalMap::resize(int sz_new){
  ARRAY_RESIZE(maps           ,LocalMap*        ,n_map, sz_new);
  ARRAY_RESIZE(mapRef         ,RobotPoseCov     ,n_map, sz_new);
  ARRAY_RESIZE(path2map       ,MapPath*         ,n_map, sz_new);
  ARRAY_RESIZE(trans2map      ,MapTransition*   ,n_map, sz_new);
  ARRAY_RESIZE(regions        ,MapAreaInterface*,n_map, sz_new);
  ARRAY_RESIZE(is_neighbour   ,bool             ,n_map, sz_new);
  ARRAY_RESIZE(neighbours     ,int              ,n_map, sz_new);
  ARRAY_RESIZE(pred           ,int              ,n_map, sz_new);
  ARRAY_RESIZE(travesal_order ,int              ,n_map, sz_new);


  memset(maps      + sz_map, 0, (sz_new - sz_map)*sizeof(LocalMap*));
  memset(path2map  + sz_map, 0, (sz_new - sz_map)*sizeof(MapPath*));
  memset(trans2map + sz_map, 0, (sz_new - sz_map)*sizeof(MapTransition*));
  memset(regions   + sz_map, 0, (sz_new - sz_map)*sizeof(MapAreaInterface*));
  nodes2edge.resize(sz_new,-1);
  sz_map = sz_new;
}

//Should be called after destroy()
void GlobalMap::set(const GlobalMap &other){
  int i;

  nullAll();

  n_map      = other.n_map;
  currentRef = other.currentRef;

  nodes2edge = other.nodes2edge;

  //Copy Maps
  if(n_map > 0){
    resize(other.sz_map);

    //memset(path2map, 0, sz_map*sizeof(MapPath*));
    //memcpy(mapRef, other.mapRef, n_map*sizeof(RobotPoseCov));

    for(i = 0; i < n_map; ++i){
      maps[i] = new LocalMap(*other.maps[i]);
    }

    //    memset(maps + n_map, 0, (sz_map-n_map)*sizeof(LocalMap*));
  }

  //Copy Transitions
  sz_trans = other.sz_trans;
  n_trans = other.n_trans;

  if(other.transition != NULL){
    transition = new MapTransition*[sz_trans];

    for(i = 0; i < n_trans; ++i){
      transition[i] = new MapTransition(*other.transition[i]);
    }
  }else{
    transition = NULL;
  }

  if(other.currentRef >= 0){
    //DODGE: for now just recompute the whole thing.
    setReferenceFrame(other.currentRef);
  }
}


void GlobalMap::destroy(){
  if(maps == NULL) return;

  DESTROY_ARRAYP(maps     ,n_map);
  DESTROY_ARRAYP(path2map ,n_map);
  DESTROY_ARRAYP(trans2map,n_map);
  DESTROY_ARRAYP(regions  ,n_map);

  DESTROY_ARRAY(mapRef);
  DESTROY_ARRAY(is_neighbour);
  DESTROY_ARRAY(neighbours);
  DESTROY_ARRAY(pred);
  DESTROY_ARRAY(travesal_order);

  DESTROY_ARRAYP(transition,n_trans);

  nodes2edge.resize(0);
  n_map = n_trans =  0;
}

