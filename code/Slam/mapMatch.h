#ifndef __MAP_MATCH__
#define __MAP_MATCH__

#include "geometry.h"
#include "graph.h"
#include "map.h"

class MapMatcherInterface{
 public:
  virtual void match(const SimpleMap &m1, const SimpleMap &m2){;}
  virtual void match(const SimpleMap &m1, const SimpleMap &m2
                   , const RobotPoseCov &prior){;}

  virtual int commonElements1(int *ind){return 0;}
  virtual int commonElements2(int *ind){return 0;}

  virtual int numMatch()const{return 0;}
  virtual int mapping(int ind1)const{ return -1;}

  virtual RobotPoseCov getTranslation()const{return RobotPoseCov();}
  virtual void reset(){;}

  virtual MapMatcherInterface* clone()const{return NULL;}

  virtual ~MapMatcherInterface(){;}
};



class MapMatcher: public MapMatcherInterface{
 public:
  MapMatcher();

  MapMatcher(const MapMatcher& other){
    nullAll();
    set(other);
  }

  const MapMatcher& operator=(const MapMatcher& other){
    if(&other != this){
      reset();
      set(other);
    }
    return *this;
  }

  ~MapMatcher();

  void match(const SimpleMap &m1, const SimpleMap &m2);

  int commonElements1(int *ind);
  int commonElements2(int *ind);

  int numMatch()const{return n_match;}
  int mapping(int ind1)const{ return mapping_[ind1];}

  RobotPoseCov getTranslation()const;

  void reset(){ destroy(); nullAll();}

  void set(const MapMatcher&);

 private:
  
  int *mapping_;
  int n_match;

  Graph g1,g2;
  int    *taken;
  int    *corr;
  int     N_NODES1;
  int     N_NODES2;
  int     MAX_EDGE_OVERLAP;
  int     MAX_NODE_OVERLAP;
  double  P_EDGE_MAX;   
  double  P_MAX;         // = log(0.0001);
  double **md2;
  double  *p_node;

  void initGraphs(const SimpleMap &m1, const SimpleMap &m2);
  void init_p_node(int n);
  void compute_md2(const Graph &g1, const Graph &g2);

  void permut_process();
  void permut(int icurr,double md2sum, int num_matches, double alignment);

  void destroy();
  void nullAll();

  int getCommonMap1(Gaussian2d *)const;
  int getCommonMap2(Gaussian2d *)const;

};

#endif





