#ifndef __LANDMARK2D_H__
#define __LANDMARK2D_H__

#include "map.h"
#include "matrix2.h"
#include "localiser.h"
#include "mappingProcess.h"
#include "mapMatch.h"
#include "graph.h"
#include "image.h"

class LocalMap;

/////////////////////////////////////////////////////////
// Mapping Process
////////////////////////////////////////////////////////
//const double DROS_GridCellSize = 0.05;


class Landmark2dMappingProcess: public MappingProcessInterface{
 protected:
  double computeNegativeInfo(const int *corr
                             , const ObservationInterface*const* obs
			     , int nObs
			     , const RobotPose *robot
			     , const MapInterface *map);
  BYTE* robotMask;
  int robotMask_w;
  int robotMask_h;

  void initRobotMask(double RobotDiametre, double cellSz);

 public:
  double MAX_MD2;
  bool useNegativeInfo;

  Landmark2dMappingProcess();

  //For Map Builder

  virtual MapEntryInterface * makeNewEntry(double *logProb,
                                   const ObservationStore &obs_store, 
                                   int ind,
                                   const RobotPose* robot_pose)const;

  virtual RangeInterface* getNeighbourhoodRange(const RobotPose*);

  //Perform data association
  //   corr_out[i] -- should be set to index of the map element from
  //                  which observation i is originating,
  //                  -1 indicates no data association.
  //   obs,nObs    -- Array of observations and it's size
  //   robot       -- Robot pose
  //   map         -- Map
  // 
  // Return value: log(Prob obs->map match);
  virtual double dataAssociate(int *corr_out
		       ,const ObservationInterface*const* obs, int nObs
		       ,const RobotPose *robot
                       ,const MapInterface *map);

  //For Local Map
  virtual SimpleMap* loadLocalMap(const MatlabVariable&);
  //storeMap should also be here

  //For Global Map
  virtual bool isNeighbour(int nHops, const RobotPoseCov* mapRef,
		   const MapAreaInterface* region);

  virtual void getSuitableCoreLandmarks(int *core, const SimpleMap* m);

  //For Mapping Agent
  //   Compute Map Area and pose of the robot within it
  virtual void getInitialMapArea(MapAreaInterface **mapArea,
			 RobotPose* robot);
  virtual double subtractFromRegion(MapAreaInterface* ,
                            const MapAreaInterface *);

  virtual double subtractFromRegion(MapAreaInterface* ,
                            const MapAreaInterface **,
                            const RobotPose*, int);
  virtual void finaliseMapArea(MapAreaInterface*, const RobotPose&);
  virtual void finaliseMapArea(MapAreaInterface*);
  virtual void finaliseMapArea(MapAreaInterface*,const SimpleMap*);

  virtual void sampleTransition(const SimpleMap* m1, const SimpleMap* m2,
			const RobotPoseCov *m2in1,
			RobotPose *samples, double *weight
			, int nsamples);

  virtual double computeRegionScore(const MapAreaInterface*, const RobotPose*);

};


/////////////////////////////////////////////////////////
// Observation
////////////////////////////////////////////////////////
class Landmark2dObs: public ObservationInterface{

 public:
  double range;
  double bearing;

  double cov[2][2];

  Landmark2dObs():range(0), bearing(0){
    cov[0][0] = cov[0][1] = cov[1][0] = cov[1][1] = 0;
  }

  Landmark2dObs(double r, double sr2, 
		double a, double sa2):
    range(r),bearing(a)
  {
    cov[0][0] = sr2;
    cov[0][1] = cov[1][0] = 0; 
    cov[1][1] = sa2;
  }

  Landmark2dObs(double r, double a, const double COV[2][2]):
                range(r),bearing(a){
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
  }

  virtual ObservationInterface *clone()const{ 
    return new Landmark2dObs(range,bearing, cov);
  }

  
  virtual double logLikelihood(const ObservationInterface*)const;

  virtual Gaussian2d toCartesian(const RobotPose *r)const;
  virtual Gaussian2d toCartesian()const;

  virtual int getObsType()const{return 1;}

  virtual bool matlabDump(FILE *f)const{
    return fprintf(f,"%e %e %e %e %e %e"
		   ,range
		   ,bearing
		   ,cov[0][0], cov[0][1]
		   ,cov[1][0], cov[1][1]
                   ) > 0;
  }

};

/////////////////////////////////////////////////////////
// Landmark
////////////////////////////////////////////////////////
class Landmark2d: public MapEntryInterface{

 public:
  Gaussian2d point;

  Landmark2d(){
    MEM_STAT_ADD(mapEntry);
  }

  Landmark2d(const Gaussian2d &p):point(p){
    numObs_ = 1;
    MEM_STAT_ADD(mapEntry);
  }

  Landmark2d(const Gaussian2d &p, int st):point(p){
    state = st;
    numObs_ = 1;

    MEM_STAT_ADD(mapEntry);
  }

  Landmark2d(const Landmark2d &from){
    copy(from);

    MEM_STAT_ADD(mapEntry);
  }

  virtual ~Landmark2d(){
    MEM_STAT_REMOVE(mapEntry);
  }

  const Landmark2d & operator=(const Landmark2d & rhs){ 

    if(&rhs != this){     
      copy(rhs);
    }

    return *this;
  }

  void copy(const Landmark2d & other){
    MapEntryInterface::set(&other);
    point = other.point;
  }


  virtual MapEntryInterface * clone()const{
    return new Landmark2d(*this);
  }


  virtual void update(const ObservationStore &obs_store,
		      int obs_ind, 
		      const RobotPose *robot_pose);

  virtual  void translateMe(const RobotPose &pose){
    point.translateMe(pose);
  }

  virtual  void translateMe(double x, double y, double ca, double sa){
    point.translateMe(x,y,ca,sa);
  }

  //Translate with adding uncertainty
  virtual  void translateMe(const RobotPoseCov &pose){
    pose.propagate(&point);
  }

  virtual  double probMatch(const MapEntryInterface* )const;
  virtual  double probMatchLog(const MapEntryInterface*)const;
  virtual  double mahalanobis2(const MapEntryInterface*)const;

  virtual double probMatch(const ObservationInterface*,const RobotPose*)const;
  virtual double probMatchLog(const ObservationInterface*,
			      const RobotPose*)const; 
  virtual double mahalanobis2(const ObservationInterface*,
                              const RobotPose*)const;

  virtual double det()const{return matrix2_det(point.cov);}
  virtual double norm()const{return matrix2_norm(point.cov);}

  virtual  bool matlabDump(FILE *f)const;

  virtual Gaussian2d getCenter()const{return point;}
  virtual void getCenter(double *x, double* y)const{ 
    *x = point.x; *y = point.y;}

  virtual double x()const{return point.x;}
  virtual double y()const{return point.y;}
};

//---------------------------------------------------------------
// RangeAndBearingSensorRange
//---------------------------------------------------------------

class RangeAndBearingSensorRange: public SensorRangeInterface{
  double minRange;
  double dRange;

  double minRange2;
  double maxRange2;

  double minAngle;
  double dAngle;


 public:
  RangeAndBearingSensorRange():minRange(0),dRange(9.0)
    ,minAngle(-M_PI/2 - 5*DEGTORAD),dAngle(M_PI + 10*DEGTORAD){

    minRange2 = minRange*minRange;
    maxRange2 = minRange2 + 2*dRange*minRange + dRange*dRange;
  }

  RangeAndBearingSensorRange(double minR, double maxR, 
			     double minA, double maxA)
    :minRange(minR),dRange(maxR - minR)
    ,minAngle(minA),dAngle(maxA - minA){

    minRange2 = minRange*minRange;
    maxRange2 = maxR*maxR;
  }

  bool isWithinRange(const ObservationInterface* o)const;

  bool isWithinRange(double x, double  y)const;

  bool isWithinRange(double x, double  y, double a0)const;

};

/////////////////////////////////////////////////////////
// Landmark2dMapMatcher
////////////////////////////////////////////////////////

class Landmark2dMapMatcher: public MapMatcherInterface{
 private:
  GraphMatcher matcher;
  Graph g1;
  Graph g2;

 public:
  Landmark2dMapMatcher(){;}

  Landmark2dMapMatcher(const Landmark2dMapMatcher& o):
    matcher(o.matcher), g1(o.g1), g2(o.g2){;}

  //Interface functions

  void match(const SimpleMap &m1, const SimpleMap &m2);
  void match(const SimpleMap &m1, const SimpleMap &m2, const RobotPoseCov&);

  int commonElements1(int *ind){return matcher.commonElements1(ind);}
  int commonElements2(int *ind){return matcher.commonElements2(ind);;}

  int numMatch()const{return matcher.numMatch();}
  int mapping(int ind1)const{ return matcher.mapping(ind1);}

  RobotPoseCov getTranslation()const;
  void reset(){;}

  MapMatcherInterface* clone()const{return new Landmark2dMapMatcher(*this);}
};


class MatlabVariable;
SimpleMap* Landmark2d_LoadMap(const MatlabVariable &m);
bool Landmark2d_storeLocalMap(FILE *f, const LocalMap *lmap);


//---------------------------------------------------------------
// Landmark2dLocGuts
//---------------------------------------------------------------

class Landmark2dLocGuts: public LocaliserParticleGuts{
 private:
  double PROB_NO_MATCH;
  double PROB_NO_MATCH_LOG;

 public:
  Landmark2dLocGuts():PROB_NO_MATCH(0.1),PROB_NO_MATCH_LOG(log(0.1)){;}

  Landmark2dLocGuts(int ID, const SimpleMap*m, LocaliserMonitor* mon):
    LocaliserParticleGuts(ID,m,mon)
    ,PROB_NO_MATCH(0.1),PROB_NO_MATCH_LOG(log(0.1)){;}

  Landmark2dLocGuts(const Landmark2dLocGuts & other):
    LocaliserParticleGuts(other.id, other.map, other.monitor)
    ,PROB_NO_MATCH(other.PROB_NO_MATCH)
    ,PROB_NO_MATCH_LOG(other.PROB_NO_MATCH_LOG){;}

  int associate(int *corr_out, double *w_out,
                const RobotPose *p,
		const ObservationStore &obs_store,
		const int obs_ind);

  //Probability of the observation coming from some other source,
  //not present in the map = Prob(Landmark is not in the map) +
  //                         Prob(Spurious reading)
  //
  double probNoMatchLog(const ObservationInterface *obs,
			const RobotPose* robot){
    return PROB_NO_MATCH_LOG;
  }

  double probNoMatch(const ObservationInterface *obs,
		     const RobotPose* robot){
    return PROB_NO_MATCH;
  }

  LocaliserParticleGuts * clone()const{ 
    return new Landmark2dLocGuts(*this);
  }
};

#endif
