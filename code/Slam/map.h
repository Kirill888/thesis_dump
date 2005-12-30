#ifndef __MAP_H__
#define __MAP_H__

#ifndef DEPEND
#include <stdio.h>
#include <string.h>
#endif

#include "geometry.h"
#include "DataStore.h"
#include "memstat.h"
#include "AngleTools.h"
#include "util.h"


//====================================================================
//
//====================================================================
class ObservationInterface{
 public:
  virtual ObservationInterface* clone()const{return NULL;}
  virtual double logLikelihood(const ObservationInterface*)const{return 0.0;}
  virtual int getObsType()const{return 0;}

  virtual bool matlabDump(FILE*f)const{return false;}

  virtual ~ObservationInterface(){;}
};

//====================================================================
//
//====================================================================
class MapEntryInterface{
 protected:
  int numObs_;
  int state;
  int globalId;
  int lastScanId;

  void set(const MapEntryInterface* other){
   numObs_    = other->numObs_;
   state      = other->state;
   globalId   = other->globalId;
   lastScanId = other->lastScanId;
  }

 public:

  MapEntryInterface():numObs_(0),state(0),globalId(-1),lastScanId(-1)
  {;}

  virtual void update(const ObservationStore &obs_store,
		      int obs_ind, 
		      const RobotPose *robot_pose){;}

  virtual MapEntryInterface* clone()const{return NULL;}

  //----------------MULTISLAM-ONLY--------------------------------------//
  virtual double probMatch(const MapEntryInterface*)const{return 1.0;}
  virtual double probMatchLog(const MapEntryInterface*)const{return 0.0;}
  virtual double mahalanobis2(const MapEntryInterface*)const{return 0.0;}
  //----------------MULTISLAM-ONLY--------------------------------------//

  //--------NOT-USED-ANYMORE-(except localiser)------------------//
  virtual double probMatch(const ObservationInterface*
                          ,const RobotPose*)const{return 1.0;}
  virtual double probMatchLog(const ObservationInterface*,
                              const RobotPose*)const{return 0.0;}
  virtual double mahalanobis2(const ObservationInterface*,
                              const RobotPose*)const{return 0.0;}
  //------------END-NOT-USED-------------------------------------//


  //-------------MISC-------------------------------------------//
  virtual double det()const{return 0.0;}
  virtual double norm()const{return 0.0;}

  virtual void translateMe(const Point &){;}
  virtual void translateMe(const RobotPose &){;}
  virtual void translateMe(double x, double y, double ca, double sa){;}
  virtual void translateMe(const RobotPoseCov &){;}

  int numObs()const{    return numObs_;}
  void setNumObs(int n){ numObs_ = n;}

  int getLastScanId()const{ return lastScanId;}

  bool isNew()const{    return (state & 1) == 0;}
  bool isMature()const{ return (state & 1) != 0;}

  bool isPeripheral()const{ return (state & 2) == 0;}
  bool isCore()const{       return (state & 2) != 0;}
  bool isNeighbour()const{  return (state & 4) != 0;}

  int getState()const{return state;}
  int getGlobalId()const{ return globalId;}
  void setGlobalId(int id){ globalId = id; }

  void setState(int st){ state  = st;}
  void setCore()       { state |=  2;}
  void setPeripheral() { state &= ~2;}
  void setNeighbour()  { state |=  4;}
  void setMature()     { state |=  1;}
  void setNew()        { state &= ~1;}

  virtual bool matlabDump(FILE* f)const{return false;}

  virtual Gaussian2d getCenter()const{return Gaussian2d();}
  virtual void getCenter(double *x, double* y)const{;}

  virtual ~MapEntryInterface(){;}

};

//====================================================================
//
//====================================================================
class MapIterator{
 public:
  //Should return true to continue, false to stop
  virtual bool process(const MapEntryInterface *e, int ind){
    return false;
  }

  virtual ~MapIterator(){;}
};

//====================================================================
//
//====================================================================
class RangeInterface{
 public:
  virtual bool isWithinRange(const MapEntryInterface *)const{return true;}
  virtual ~RangeInterface(){;}
};

//====================================================================
//
//====================================================================
class SensorRangeInterface{
 public:
  virtual bool isWithinRange(const ObservationInterface*)const{return true;}
  virtual ~SensorRangeInterface(){;}
};

//====================================================================
//
//====================================================================
class SimpleMap{

 public:
  SimpleMap();
  SimpleMap(int Nmap);
  SimpleMap(const SimpleMap &other);
  SimpleMap(const SimpleMap *other);
  //  SimpleMap(const Gaussian2d *Map, int Nmap);

  SimpleMap* clone()const{ return new SimpleMap(this);}

  ~SimpleMap(){destroy();}

  const SimpleMap & operator=(const SimpleMap &other);

  void translateMe(const RobotPose &p);
  void translateMe(const RobotPoseCov &p);
  void translateMe(const Point &p);

  SimpleMap* subMap(const int *ind, int n)const;
  void merge(const SimpleMap &other);

  MapEntryInterface * operator[](int i){ return map[i]; }
  MapEntryInterface* get(int i)const{ return map[i];}

  void set(int i, const MapEntryInterface *e){
    map[i] = e->clone();
  }

  bool isEmpty()const{ return map == NULL;}
  int numMap()const{ return n_map; }
  int numElements()const{ return n_map; }

  MapEntryInterface** getMap(){ return map;}
  operator MapEntryInterface**(){ return map;}

  const MapEntryInterface*const* getMap_const()const{ return map;}

  double probMatchLog(const SimpleMap &m)const;

  void matlabDump(FILE *f, const char* name)const;

 private:
  MapEntryInterface **map;
  int n_map;

  void destroy();
  void copy(const SimpleMap *other);
};


//====================================================================
//
//====================================================================
class MapInterface{

 public:

  virtual MapInterface* clone()const{ return NULL;}

  virtual int numElements()const{return 0;}
  virtual int numMature()const{return 0;}

  virtual const MapEntryInterface* get(int ind)const{return NULL;}
  virtual const MapEntryInterface* operator[](int ind)const{return get(ind);}
     
  virtual void forEach(MapIterator &iterator)const{;}

  virtual void set(int ind, MapEntryInterface *e){;}

  virtual void add(MapEntryInterface *e){;}

  virtual double getWeight()const{return 1.0;}

  virtual SimpleMap * getSubMap(const RangeInterface &)const{
    return NULL;
  }

  virtual MapInterface* flatten(){ return NULL;}

  virtual void init(const SimpleMap * m){;}
  //void init(const Gaussian2d *map, int nmap){;}

  virtual void update(int id, const ObservationStore &obs_store,
		int obs_ind, 
		const RobotPose *robot_pose){;}

  virtual void setState(int id, int st){;}

  virtual ~MapInterface(){;}

};

//===============================================================
// Map area interfaces.
//===============================================================

class MapAreaInterface{
 public:

  virtual void translateMe(const RobotPose &r){
    translateMe(r.x, r.y, cos(r.rot), sin(r.rot));
  }

  virtual void translateMe(double x, double y, double ca, double sa){;}

  virtual void setPose(const RobotPose*r){
    setPose(r->x, r->y, r->rot);
  }
  virtual void setPose(double x, double y, double a){;}

  virtual double probIsInside(double x, double y)const{return 0.0;}
  virtual double probIsInside(const RobotPose *r)const{return 0.0;}
  virtual double probIsInside(const MapEntryInterface *)const{return 0.0;}
  virtual double probIsInside(const ObservationInterface *,
                              const RobotPose *)const{return 0.0;}

  virtual void getCenter(double *x, double *y)const{;}
  virtual void getMin(double *x, double* y)const{;}
  virtual void getMax(double *x, double* y)const{;}

  virtual bool matlabDump(FILE*f,const char*)const{return true;}
  virtual MapAreaInterface * clone()const{return NULL;}
  virtual ~MapAreaInterface(){;}
};

class MapAreaFactoryInterface{
 public:
  //  virtual bool isSuitableCoreLandmark(const MapEntryInterface*)const{
  //    return true;}

  virtual ~MapAreaFactoryInterface(){;}
};

//===============================================================
// Map Builder interface.
//===============================================================
class SLAMMonitor;
class SLAMParticle;

class MapBuilderInterface{
 protected:
  const SensorRangeInterface* sensorRange;
  const MapAreaInterface* mapArea;


  static const SensorRangeInterface DefaultSensorRange; 
                   //Accepts all observations

 public:

  MapBuilderInterface():sensorRange(&DefaultSensorRange),
                        mapArea(NULL){;}

  virtual MapInterface* makeEmptyMap()const{return NULL;}
  virtual MapInterface* makeMap(const SimpleMap *m)const{return NULL;}

  virtual double update(MapInterface* map, 
                        const ObservationStore &store,
		        const RobotPose *robot_pose,
		        SLAMParticle *p,
                        SLAMMonitor* monitor)const{return 0;}

  virtual void setMapArea(const MapAreaInterface* ma){mapArea = ma;}


  virtual MapBuilderInterface * clone()const{return NULL;}
  virtual ~MapBuilderInterface(){;}

  void setSensorRange(const SensorRangeInterface *sensor){
    sensorRange = sensor;
  }

  const SensorRangeInterface* getSensorRange()const{return sensorRange;}

  void set(const MapBuilderInterface* other){
    sensorRange = other->sensorRange;
  }
};



//===============================================================
// Implementations of interfaces.
//===============================================================

class Map: public MapInterface{
 public:

  class SubMap: public MapInterface{
    static const int SIZE_INCREMENT;

    MapEntryInterface **elements;
    int *map_index;

    int nElements;
    int arraySize;

    Map * parent;

  public:

    SubMap(){
      elements  = NULL;
      map_index = NULL;
      arraySize = 0;
      nElements = 0;
      parent = NULL;
      //MEM_STAT_ADD(subMap);
    }

    SubMap(int sz,Map* p);

    SubMap(const SubMap &other);

    const SubMap & operator=(const SubMap &other);

    ~SubMap();

    //const MapEntryInterface* getMapEntry(int i)const{return elements[i];}
    const MapEntryInterface* get(int i)const{return elements[i];}
    const MapEntryInterface* operator[](int i)const{return elements[i];}

    //int getMapIndex(int i)const{return map_index[i];}

    void add(MapEntryInterface *m, int ind);

    int numElements()const{ return nElements; }

    void update(int id, const ObservationStore &obs_store,
		  int obs_ind, 
		  const RobotPose *robot_pose){

      parent->update(map_index[id], &elements[id],
                     obs_store, obs_ind, robot_pose);
    }

    void setState(int id, int st){
      elements[id]->setState(st);
    }

  };


 private:
  struct TreeNode{
    int index;

    struct TreeNode *left;
    struct TreeNode *right;

    int copyCount;

    MapEntryInterface *data;

    TreeNode(){
      left = right = NULL;
      index = 0;
      copyCount = 1;
      data = NULL;

      MEM_STAT_ADD(mapNode);
    }

    TreeNode(struct TreeNode *l, struct TreeNode *r, 
	     int i, MapEntryInterface *d){

      left  = l;
      right = r;
      index = i;
      data  = d;
      copyCount = 1;

      MEM_STAT_ADD(mapNode);
    }

    ~TreeNode(){
      if(data != NULL) delete data;
      MEM_STAT_REMOVE(mapNode);
    }
  };
  struct TreeNode *top;
  int topLevel;
  int nLeafs;
  int nElements;
  int nMature;

  void destroyTree(struct TreeNode *n);

  struct TreeNode * createBranch(struct TreeNode *copy,
				 int startLevel, int startIndex,
				 int leafIndex,
				 MapEntryInterface *e);

  static bool forEach(const struct TreeNode *n, 
		      MapIterator &iterator);

  static void findVisible(struct TreeNode *n, 
			  SubMap *visibleMap, 
			  const RobotPose &p);

  static void findSubmap(const struct TreeNode *n, 
			 SubMap *visibleMap, 
			 const RangeInterface *range);
 public:
  Map(){
    topLevel = -1;
    top = NULL; 
    nLeafs = 0;
    nElements = 0;
    nMature = 0;
  }

  Map(const Map &copy){
    topLevel  = copy.topLevel;
    top       = copy.top;
    nLeafs    = copy.nLeafs;
    nElements = copy.nElements;
    nMature   = copy.nMature;

    if(top!=NULL) top->copyCount += 1;
  }

  const Map & operator=(const Map &rhs){
    if(&rhs == this) return *this;
       
    destroyTree(top);

    topLevel  = rhs.topLevel;
    top       = rhs.top;
    nLeafs    = rhs.nLeafs;
    nElements = rhs.nElements;
    nMature   = rhs.nMature;

    if(top!=NULL) top->copyCount += 1;

    return *this;
  }

  MapInterface* clone()const{ return new Map(*this);}

  ~Map(){ destroyTree(top); }

  int numElements()const{ return nElements; }
  int numMature()const{ return nMature;}

  void add(MapEntryInterface *e){
    set(nElements,e);
    nElements += 1;
    if(e->isMature()) nMature += 1;
  }

  void set(int ind, MapEntryInterface *e);
  const MapEntryInterface* get(int ind)const;

  void forEach(MapIterator &iterator)const;

  MapInterface* findSubmap(const RangeInterface *range);

  MapInterface* flatten();
  
  SimpleMap * getSubMap(const RangeInterface &)const;

  void update(int map_ind,
	      const ObservationStore &obs_store,
	      int obs_ind, 
	      const RobotPose *robot_pose);

  void update(int map_ind, MapEntryInterface **map_entry,
	      const ObservationStore &obs_store,
	      int obs_ind, 
	      const RobotPose *robot_pose);

  void reset(){
    destroyTree(top);
    topLevel = -1;
    top = NULL; 
    nLeafs = 0;
    nElements = 0;
    nMature = 0;
  }

  void init(const SimpleMap *map);

};


class MapBuilder: public MapBuilderInterface{
 public:
  static const double MAX_DIST2;

  MapBuilder(){;}

  MapBuilder(const MapBuilder &other){
    set(&other);
  }

  MapBuilder(const MapBuilder *other){
    set(other);
  }

  //  ~MapBuilder(){;}

  MapBuilderInterface* clone()const{
    MapBuilder *copy = new MapBuilder(this);
    return copy;
  }

  const MapBuilder & operator=(const MapBuilder &other){
    if(&other == this) return *this;
	 
    set(&other);
    return *this;
  }

  MapInterface* makeEmptyMap()const{
      return new Map();
  }

  MapInterface* makeMap(const SimpleMap *m)const{
    Map *map = new Map;
    map->init(m);
    return map;
  }

  double update(MapInterface* map,
              const ObservationStore &store,
	      const RobotPose *robot_pose,
	      SLAMParticle *p,
              SLAMMonitor* monitor)const;

};


//==============================================================
// Range Implementations
//==============================================================

class RangeMinObs: public RangeInterface{
  int min_obs;

 public:
  RangeMinObs(int minObs){
    min_obs = minObs;
  }

  bool isWithinRange(const MapEntryInterface *m)const{
    return m->numObs() >= min_obs;
  }
};

class RangeLastScanId: public RangeInterface{
  int min_scan;

 public:
  RangeLastScanId(int min):min_scan(min){;}

  bool isWithinRange(const MapEntryInterface *m)const{
    return m->getLastScanId() >= min_scan;
  }
};


class RangeMature: public RangeInterface{
 public:
  bool isWithinRange(const MapEntryInterface *m)const{
    //Accept mature non-neighbours core or peripheral and 
    // non-core mature neighbours
    return (m->isMature() && !m->isNeighbour())
         ||(m->isNeighbour() && m->isPeripheral() && m->numObs() >= 3)
      ;
  }
};

class RangeMinMax: public RangeInterface{
  double minx,miny,maxx,maxy;

 public:
  RangeMinMax(double Minx, double Miny, double Maxx, double Maxy):
     minx(Minx), miny(Miny), maxx(Maxx), maxy(Maxy){;}

  bool isWithinRange(const MapEntryInterface *m)const{
    double x,y;

    m->getCenter(&x,&y);

    if(x < minx || x > maxx || y < miny || y > maxy){
      return false;
    }

    return true;
  }
};


//==============================================================
// Map Printer
//==============================================================
class MapPrinter: public MapIterator{
 private:
  FILE *f;

  bool err;

  bool (*print_callback)(FILE *f, int ind, const MapEntryInterface *e);

 public:

  MapPrinter(){
    f = NULL;
    err = false;
    print_callback = NULL;
  }

  MapPrinter(bool (*callback)(FILE*f, int, const MapEntryInterface *e)){
    f = NULL;
    err = false;

    print_callback = callback;
  }
  MapPrinter(const MapPrinter &other){
    err = other.err;
    f   = other.f;
    print_callback = other.print_callback;
  }

  const MapPrinter & operator=(const MapPrinter &other){
    err = other.err;
    f   = other.f;

    return *this;
  }

  bool process(const MapEntryInterface *e, int ind);

  bool print(const char *fname, const MapInterface *map);
  bool print(FILE *fp, const MapInterface *map);
};


#endif
