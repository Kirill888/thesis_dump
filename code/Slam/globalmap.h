#ifndef __GLOBAL_MAP_H__
#define __GLOBAL_MAP_H__

#include "localmap.h"
#include "array2d.h"
#include "util.h"
#include "bisearch.h"

//============================================================================
// Map Transition
//============================================================================

class MapTransition{
 private:
  int map1, map2;
  int np;
  double *w;
  double *w_cumsum;
  double *w_log;

  RobotPose *p;

  RobotPoseCov r2in1;
  RobotPoseCov r1in2;
  BinarySearch biSearch;

 public:

  MapTransition():map1(-1),map2(-1),np(0)
                 ,w(NULL),w_cumsum(NULL),w_log(NULL),p(NULL){;}

  MapTransition(int m1, int m2, 
		const RobotPose* p2in1,
		const double *weight,
		int n):map1(m1),map2(m2),np(0)
                 ,w(NULL),w_cumsum(NULL),w_log(NULL),p(NULL){
    set(m1,m2,p2in1,weight,n);
  }

  MapTransition(int m1, int m2, 
		const RobotPose* p2in1,
		int n):map1(m1),map2(m2),np(0)
                 ,w(NULL),w_cumsum(NULL),w_log(NULL),p(NULL){
    set(m1,m2,p2in1,n);
  }

  MapTransition(const MapTransition &other):map1(-1),map2(-1),np(0)
                 ,w(NULL),w_cumsum(NULL),w_log(NULL),p(NULL){
    set(other);
  }

  const MapTransition & operator=(const MapTransition &other){
    if(this != &other)
       set(other);
    return *this;
  }

  ~MapTransition(){destroy();}

  int getMap1()const{ return map1;}
  int getMap2()const{ return map2;}

  void destroy();

  void set(int m1, int m2, 
           const RobotPose* p2in1,
	   const double *weight,
	   int n);

  void set(int m1, int m2, 
           const RobotPose* p2in1,
	   int n);

  void update(int m1, int m2, 
	      const RobotPose* p2in1,
	      const double *weight,
	      int n){
    destroy();
    set(m1,m2,p2in1,weight,n);
  }

  void set(const MapTransition &other);

  void setWeight(double weight){
    for(int i = 0; i < np; ++i) w[i] = weight;
  }

  void scaleWeight(double scale){
    for(int i = 0; i < np; ++i) w[i] *= scale;
  }

  void setWeight(int i, double weight){ w[i] = weight;}
  void addWeight(int i, double weight){ w[i]+= weight;}

  void scaleWeight(int i, double scale){ w[i]*= scale;}

  void normalise();

  void invert(); //Change from m1->m2 to m2->m1

  RobotPose transit(const RobotPose& r, int source_map, int ind)const{
    RobotPose rr = r;
    transit(&rr, source_map, ind);
    return rr;
  }

  void transit(RobotPose *r, int source_map, int ind)const;

  RobotPoseCov propagate(const RobotPoseCov &r, int source_map){
    RobotPoseCov pp(r);
    propagate(&pp,source_map);
    return pp;
  }

  void propagate(RobotPoseCov *r, int source_map)const;

  int transit(int source_map)const{ 
    return source_map == map1 ? map2 : map1; }

  int sample()const{
    double r = ((double)rand())/(double)RAND_MAX;
    return biSearch.find(r);
  }

  void sample(int *ind_out, int nsample)const;

  RobotPose sample(const RobotPose& p0, int source_map)const{
    RobotPose pp = p0;
    transit(pp, source_map, sample());
    return pp;
  }

  const RobotPoseCov& getR2In1()const{return r2in1;}
  const RobotPoseCov& getR1In2()const{return r1in2;}
  const RobotPoseCov& getRef(int map) const{
    if(map == map1) return r1in2;
    else            return r2in1;
  }

  const RobotPose& get(int i)const{ return p[i];}
  void get(RobotPose*pose, int i)const{ pose->set(p[i]);}
  double getWeight(int i)const{ return w[i];}
  double getLogWeight(int i)const{ return w_log[i];}

  int numP()const{return np;}

  double computeEntropy()const;

  bool load(const char *fileName){
    FILE *f = fopen(fileName,"r");
    if(f == NULL) return false;

    bool res = load(f);
    fclose(f);
    return res;
  }

  bool store(FILE *f, int id)const;
  bool load(FILE *f);

  void computeFinalPose();

};

//============================================================================
// Map Path
//============================================================================

class MapPath{
 private:

  int startMap;
  int endMap;
  int n;
  MapTransition const **  transitions;
  int *itrans;

 public:

  MapPath():startMap(-1),endMap(-1),n(0),transitions(NULL),itrans(NULL){;}

  MapPath(int sz, int start, int end):startMap(start),endMap(end)
                                     ,n(sz),transitions(NULL),itrans(NULL){
    if(sz >0 ){
       transitions = new const MapTransition*[sz];
       itrans = new int[sz];
    }
  }

  MapPath(const MapPath &o):startMap(o.startMap),endMap(o.endMap)
                                     ,n(o.n),transitions(NULL),itrans(NULL){
    if(n > 0){
      transitions = new const MapTransition*[n];
      itrans = new int[n];

      memcpy(transitions, o.transitions, n*sizeof(const MapTransition*));
      memcpy(itrans, o.itrans, n*sizeof(int));
    }
  }

  //Grow path, take previous path and add transition to it
  //transition should start from the end map of the initial path
  MapPath(const MapPath &o, 
          const MapTransition * t, 
          int transInd):startMap(o.startMap),endMap(o.endMap)
                                     ,n(o.n+1),transitions(NULL),itrans(NULL){

    transitions = new const MapTransition*[n];

    if(o.transitions != NULL){
      memcpy(transitions, o.transitions, 
	     o.n*sizeof(const MapTransition*));
    }
    transitions[n-1] = t;

    itrans = new int[n];
    if(o.itrans != NULL){
      memcpy(itrans, o.itrans, o.n*sizeof(int));
    }
    itrans[n-1] = transInd;
  }

  MapPath* clone()const{ return new MapPath(*this);}

  const MapPath& operator=(const MapPath & o){
    if(this == &o) return *this;

    if(transitions != NULL){ delete[] transitions;}
    if(itrans != NULL){ delete[] itrans;}

    n        = o.n;
    startMap = o.startMap;
    endMap   = o.endMap;

    if(n > 0){
      transitions = new const MapTransition*[n];
      itrans = new int[n];

      memcpy(transitions, o.transitions, n*sizeof(const MapTransition*));
      memcpy(itrans, o.itrans, n*sizeof(int));
    }else{
      transitions = NULL;
      itrans = NULL;
    }

    return *this;
  }

  ~MapPath(){
    if(transitions != NULL){ delete[] transitions;}
    if(itrans != NULL)     { delete[] itrans;}
  }

  int getStartMap()const{return startMap;}
  int getEndMap()const{return endMap;}
  int getNHops()const{ return n;}
  int nHops()const{ return n;}

  const MapTransition* operator[](int i)const{ return transitions[i];}
  const MapTransition* get(int i)const{ return transitions[i];}
  int ind(int i)const{ return itrans[i]; }

  void set(int i, const MapTransition * t){transitions[i] = t;}

  RobotPoseCov propogate(const RobotPoseCov p)const{
    RobotPoseCov pp(p);
    propogate(pp);
    return pp;
  }

  RobotPoseCov propogate_rev(const RobotPoseCov p)const{
    RobotPoseCov pp(p);
    propogate_rev(pp);
    return pp;
  }

  void propagate(RobotPoseCov *p)const{
    int map = startMap;
    for(int i = 0 ; i < n; ++i){
      transitions[i]->propagate(p,map);
      map = transitions[i]->transit(map);
    }
  }
  
  void propagate_rev(RobotPoseCov *p)const{
    int map = endMap;
    for(int i = n-1 ; i >= 0; --i){
      transitions[i]->propagate(p,map);
      map = transitions[i]->transit(map);
    }
  }

  void transit(RobotPose *r, int *path)const{
    int map = startMap;
    for(int i = 0 ; i < n; ++i){
      transitions[i]->transit(r,map,path[i]);
      map = transitions[i]->transit(map);
    }
  }

  void transit(RobotPose *r)const{
    int map = startMap;
    for(int i = 0 ; i < n; ++i){
      int ind = transitions[i]->sample();
      //      printf("Path::transit %d\n",ind);
      transitions[i]->transit(r,map,ind);
      map = transitions[i]->transit(map);
    }
  }

  //Path structure:
  //--------------
  //Cols -- number of particles (say 10,000)
  //Rows -- should be large enough to accomodate all transitions.
  void samplePath(Array2d &path_out)const;

  //In addition to finding path compute log likelihood as well
  void samplePath(Array2d &path_out, double *w_log)const;

  //Compute pose from the path,
  //  p   -- should be large enough to hold all partucles path.numRows();
  // path -- see samplePath
  void path2pose(RobotPose *p, const Array2d &path)const;

  bool matlabDump(FILE*f, const char * var)const;

};

//============================================================================
//  Global Map
//============================================================================

class GlobalMap{

 public:

  GlobalMap();

  GlobalMap(const GlobalMap &other){
    nullAll();
    set(other);
  }

  const GlobalMap & operator=(const GlobalMap &other){
    if(this != &other){
      destroy();
      set(other);
    }
    return *this;
  }

  ~GlobalMap(){ destroy(); }

  void add(LocalMap *lmap);

  LocalMap & operator[](int i){ return *maps[i];}
  const LocalMap & get(int i)const{ return *maps[i];}

  void setMap(int i, LocalMap *lmap){
    if(maps[i] != NULL) delete maps[i];
    maps[i] = lmap;
  }

  int numMaps()const{ return n_map;}

  bool isAdjacent(int m1, int m2) const{
    return nodes2edge.get(m1,m2) >= 0;
  }

  void updateCoreLandmarks(int mapId, bool ignoreCore);

  //Sets current reference frame to the reference frame of map i,
  //compute relative locations of all other maps relative
  //to this one.
  void setReferenceFrame(int i);

  //Get location (with uncertainty) of the reference frame of the map i
  //in the current reference frame.
  const RobotPoseCov & getMapRef(int i) const{ return mapRef[i];}
  const MapTransition* getCompoundTransition(int map)const{ 
        return trans2map[map];}

  //Get path from current reference frame to map "to"
  const MapPath* getPath(int to)const{ return path2map[to];}
  int getNHops(int to)const{ return path2map[to]->getNHops();}

  MapTransition *getTransition(int map1, int map2){
    int it = nodes2edge.get(map1,map2);

    if(it < 0) return NULL;
    else return transition[it];
  }

  const MapTransition *getTransition_c(int map1, int map2)const{
    int it = nodes2edge.get(map1,map2);

    if(it < 0) return NULL;
    else return transition[it];
  }

  int numTrans()const{ return n_trans; }

  int getTransitionIndex(int map1, int map2){
    return nodes2edge.get(map1,map2);
  }

  //Add transtion to the map
  void addTransition(const MapTransition &tr){
    MapTransition* tr2 = new MapTransition(tr);
    addTransition(tr2);
  }

  void addTransition(MapTransition *tr);

  void reset(){ destroy();}

  bool isNeighbour(int i)const{ return is_neighbour[i];}
  int nNeighbours()const{ return n_neighbours;}
  const MapAreaInterface* getRegion(int i)const{ 
     return regions[i];
  }
  void getNeighbouringRegions(const MapAreaInterface ** out)const;

  SimpleMap* neighbourhoodMap()const;


  MapTransition* mergeTransitions(const MapTransition* parent,
                                  const MapTransition* trans,int m_dest)const;

  // Storage/Retrieval functions
  bool store(const char* out_prefix, 
             bool (*store_local_map)(FILE*, const LocalMap*) )const;
  bool load(const char* prefix);

  void animationDump(FILE *f, const char* prefix)const;

 private:
  LocalMap **maps;
  int sz_map;
  int n_map;

  //These variables are recomputed when you call setReferenceFrame
  RobotPoseCov      *mapRef;
  MapPath          **path2map;
  MapTransition    **trans2map;
  MapAreaInterface **regions;
  bool              *is_neighbour;
  int               *neighbours;
  int                n_neighbours;
  int                currentRef;
  int               *pred;
  int               *travesal_order;

  MapTransition **transition;
  int n_trans;
  int sz_trans;

  //Map to transition mapping nodes2edge(m1,m2) == edge m1->m2
  UpperTriangularMatrix nodes2edge;


  void nullAll();
  void resize(int sz_new);
  
  void set(const GlobalMap & other);

  void computeNeighbours();
  void computeBFT(int);
  void computeShortestPath(int ref);


  //Returns id;

  void assignCoreLandmark(int id);
  void destroy();

};

MapTransition* mergeTransitions(const MapTransition* t1, 
                                const MapTransition* t2, 
                                int from, int to);

//============================================================================
//  Global Landmark
//============================================================================


#endif
