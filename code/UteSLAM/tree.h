#ifndef __TREE_H__
#define __TREE_H__

#include "map.h"
#include "matrix2.h"
#include "landmark2d.h"
#include "mappingProcess.h"
#include "image.h"
#include "gridmap.h"

/////////////////////////////////////////////////////////
// Mapping Process
////////////////////////////////////////////////////////

class TreeMappingProcess: public MappingProcessInterface{
 private:
  BYTE* robotMask;
  int robotMask_w;
  int robotMask_h;

  void initRobotMask(double cell_sz);
  double MAX_MD2;

  GridMap *region0;
  void initDefaultMapRegion(double cell_sz);

 public:

  TreeMappingProcess();
  virtual ~TreeMappingProcess(){;}

  //For MapBuilder
  MapEntryInterface * makeNewEntry(double *logProb,
                                   const ObservationStore &, int ind,
                                   const RobotPose*)const;

  RangeInterface* getNeighbourhoodRange(const RobotPose*);

  //Perform data association
  //   corr_out[i] -- should be set to index of the map element from
  //                  which observation i is originating,
  //                  -1 indicates no data association.
  //   obs,nObs    -- Array of observations and it's size
  //   robot       -- Robot pose
  //   map         -- Map
  // 
  // Return value: log(Prob obs->map match);
  double dataAssociate(int *corr_out
		       ,const ObservationInterface*const* obs, int nObs
		       ,const RobotPose *robot
                       ,const MapInterface *map);

  //For Local Map
  SimpleMap* loadLocalMap(const MatlabVariable&);
  //storeMap should also be here

  //For Global Map
  bool isNeighbour(int nHops, const RobotPoseCov* mapRef,
		   const MapAreaInterface* region);

  void getSuitableCoreLandmarks(int *core, const SimpleMap* m);

  //For Mapping Agent
  //   Compute Map Area and pose of the robot within it
  void getInitialMapArea(MapAreaInterface **mapArea,
			 RobotPose* robot);

  double subtractFromRegion(MapAreaInterface* ,
                            const MapAreaInterface *);

  double subtractFromRegion(MapAreaInterface* ,
                            const MapAreaInterface **,
                            const RobotPose*, int);
  void finaliseMapArea(MapAreaInterface*, const RobotPose&);
  void finaliseMapArea(MapAreaInterface*);
  void finaliseMapArea(MapAreaInterface*,const SimpleMap*);

  void sampleTransition(const SimpleMap* m1, const SimpleMap* m2,
			const RobotPoseCov *m2in1,
			RobotPose *samples, double *weight
			, int nsamples);

  double computeRegionScore(const MapAreaInterface*, const RobotPose*);

};


//--------------------------------------------------------------
// TreeObservation
//--------------------------------------------------------------
class TreeObservation: public Landmark2dObs{
 public:
  double diameter;
  double sigma_d2;

  TreeObservation():diameter(0),sigma_d2(0){;}

  TreeObservation(double r, double sr2, 
                  double a, double sa2, 
                  double d, double sd2):
                  Landmark2dObs(r,sr2,a,sa2)
                 ,diameter(d), sigma_d2(sd2){;}

  TreeObservation(double r, double a, const double COV[2][2], 
                  double d, double sd2):
                  Landmark2dObs(r,a,COV)
                 ,diameter(d), sigma_d2(sd2){;}

  ObservationInterface *clone()const{ 
    return new TreeObservation(range, bearing, cov
                             , diameter, sigma_d2);
  }

  
  double logLikelihood(const ObservationInterface*)const;

  int getObsType()const{return 1;}

  bool matlabDump(FILE *f)const{
    return fprintf(f,"%e %e   %e %e %e %e   %e %e"
		   ,range
		   ,bearing
		   ,cov[0][0], cov[0][1]
		   ,cov[1][0], cov[1][1]
		   ,diameter
		   ,sigma_d2
                   ) > 0;
  }

};

//--------------------------------------------------------------
// TreeLandmark
//--------------------------------------------------------------

class TreeLandmark: public Landmark2d{

 public:
  double diameter;
  double sigma_d2;

  //Constructors
  TreeLandmark():diameter(0),sigma_d2(0){
    state = 0;
    numObs_ = 0;
 }

  TreeLandmark(const Gaussian2d &p, double d, double sd): 
               Landmark2d(p),diameter(d),sigma_d2(sd){
    state = 0;
    numObs_ = 0;
  }

  TreeLandmark(const Gaussian2d &p, double d, double sd, int st): 
               Landmark2d(p),diameter(d),sigma_d2(sd){
    state = st;
    numObs_ = 0;
  }

  TreeLandmark(const TreeLandmark* other):Landmark2d(*other){
    diameter = other->diameter;
    sigma_d2 = other->sigma_d2;
  }


  //Interface Functions
  void update(const ObservationStore &obs_store,
		int obs_ind, 
		const RobotPose *robot_pose);

  double probMatch(const MapEntryInterface*)const;
  double probMatchLog(const MapEntryInterface*)const;
  double mahalanobis2(const MapEntryInterface*)const;

  double probMatch(const ObservationInterface*,    const RobotPose*)const;
  double probMatchLog(const ObservationInterface*, const RobotPose*)const;
  double mahalanobis2(const ObservationInterface*, const RobotPose*)const;

/*   double det()const{return point.det();} */
/*   double norm()const{return point.norm();} */

  bool matlabDump(FILE* f)const;

  //Inlines
  MapEntryInterface* clone()const{
      return new TreeLandmark(this);}

/*   void translateMe(const Point &r){point.translateMe(r);} */
/*   void translateMe(const RobotPose &r){point.translateMe(r);} */
/*   //Translate with adding uncertainty */
/*   void translateMe(const RobotPoseCov &r){ point.translateMe(r); } */

/*   Gaussian2d getCenter()const{return point.getCenter();} */
/*   void getCenter(double *x, double* y)const{ point.getCenter(x,y); } */

/*   double x()const{return point.x();} */
/*   double y()const{return point.y();} */

 private:
  void update(double,double);
  double probMatchD(double d, double sd)const;
  double probMatchDLog(double d, double sd)const;

};

//---------------------------------------------------------------
// Support Functions
//---------------------------------------------------------------

class MatlabVariable;
bool Tree_storeLocalMap(FILE *f, const LocalMap *lmap);

#endif
