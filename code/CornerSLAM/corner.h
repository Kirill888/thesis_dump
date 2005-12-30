#ifndef __CORNER_H__
#define __CORNER_H__

#include "landmark2d.h"
#include "LaserGridMapper.h"

enum CornerType{CONCAVE,CONVEX,CONVEX_HIDDEN, JUMP, CORNER_UNKNOWN};
CornerType Int2CornerType(int i);

//-------------------------------------------------------------
// CornerObservation
//-------------------------------------------------------------

class CornerObservation: public Landmark2dObs{
 public:

  CornerType type;
  double a1;
  double sa1;
  int    dir1;

  double a2;
  double sa2;
  int    dir2;

  CornerObservation():type(CORNER_UNKNOWN)
       ,a1(0),sa1(0),dir1(0),a2(0),sa2(0),dir2(0){;}

  CornerObservation(double r, double sr2, double a, double saa
		    ,CornerType type
		    ,double a1, double sa1, int dir1
		    ,double a2, double sa2, int dir2):
    Landmark2dObs(r,sr2,a,saa), type(type)
    , a1(a1), sa1(sa1), dir1(dir1)
    , a2(a2), sa2(sa2), dir2(dir2){;}

  CornerObservation(double r, double a, const double COV[2][2]
		    ,CornerType type
		    ,double a1, double sa1, int dir1
		    ,double a2, double sa2, int dir2):
    Landmark2dObs(r,a,COV), type(type)
    , a1(a1), sa1(sa1), dir1(dir1)
    , a2(a2), sa2(sa2), dir2(dir2){;}

  ObservationInterface *clone()const{ 
    return new CornerObservation(range, bearing, cov, type
                             , a1, sa1, dir1
                             , a2, sa2, dir2);
  }


  double logLikelihood(const ObservationInterface*)const;
  int getObsType()const{return 1;}
  bool matlabDump(FILE *f)const;

};

//-------------------------------------------------------------
// CornerLandmark
//-------------------------------------------------------------

class CornerLandmark: public Landmark2d{
 private:
  void updateA1(double,double);
  void updateA2(double,double);

 public:
  CornerType type;
  double a1;
  double sa1;
  int    dir1;

  double a2;
  double sa2;
  int    dir2;

  CornerLandmark():type(CONCAVE)
       ,a1(0),sa1(0),dir1(0),a2(0),sa2(0),dir2(0){;}

  CornerLandmark(const Gaussian2d &p
		 ,CornerType type
		 ,double a1, double sa1, int dir1
		 ,double a2, double sa2, int dir2):
    Landmark2d(p), type(type)
    , a1(a1), sa1(sa1), dir1(dir1)
    , a2(a2), sa2(sa2), dir2(dir2){;}

  CornerLandmark(const CornerLandmark *other):
    Landmark2d(*other), type(other->type)
    , a1(other->a1), sa1(other->sa1), dir1(other->dir1)
    , a2(other->a2), sa2(other->sa2), dir2(other->dir2){;}

  ~CornerLandmark(){;}

  ////////////////////////////////////////////////////////////////
  // Interface functions
  ///////////////////////////////////////////////////////////////
  void update(const ObservationStore &obs_store,
	      int obs_ind, 
	      const RobotPose *robot_pose);

  double probMatch(const MapEntryInterface*)const;
  double probMatchLog(const MapEntryInterface*)const;
  double mahalanobis2(const MapEntryInterface*)const;

  /*
  double probMatch(const ObservationInterface*,    const RobotPose*)const;
  double probMatchLog(const ObservationInterface*, const RobotPose*)const;
  double mahalanobis2(const ObservationInterface*, const RobotPose*)const;
  */

  /*   double det()const{return point.det();} */
  /*   double norm()const{return point.norm();} */

  bool matlabDump(FILE* f)const;

  MapEntryInterface* clone()const{ return new CornerLandmark(this); }

  /*   void translateMe(const Point &r){point.translateMe(r);} */

  void translateMe(const RobotPose &r);
  //Translate with adding uncertainty
  void translateMe(const RobotPoseCov &r);

  /*   Gaussian2d getCenter()const{return point.getCenter();} */
  /*   void getCenter(double *x, double* y)const{ point.getCenter(x,y); } */

  /*   double x()const{return point.x();} */
  /*   double y()const{return point.y();} */

  bool isCompatible(const CornerObservation*)const;
  double MatchAnglesMD2(const CornerObservation*, const RobotPose*
                      , double *saa)const;

};

//-------------------------------------------------------------
// CornerMappingProcess
//-------------------------------------------------------------
class CornerMappingProcess: public Landmark2dMappingProcess{
 public:
  //For Map Builder
  LaserGridMapper *gridMapper;

  MapEntryInterface * makeNewEntry(double *logProb,
                                   const ObservationStore &obs_store, 
                                   int ind,
                                   const RobotPose* robot_pose)const;
  
  void getInitialMapArea(MapAreaInterface **mapArea,
			 RobotPose* robot);

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

};


//-------------------------------------------------------------
// Corner Localisation
//-------------------------------------------------------------

class CornerLocGuts: public LocaliserParticleGuts{
 private:
  double PROB_NO_MATCH;
  double PROB_NO_MATCH_LOG;

 public:
  CornerLocGuts():PROB_NO_MATCH(0.1),PROB_NO_MATCH_LOG(log(0.1)){;}

  CornerLocGuts(int ID, const SimpleMap*m, LocaliserMonitor* mon):
    LocaliserParticleGuts(ID,m,mon)
    ,PROB_NO_MATCH(0.1),PROB_NO_MATCH_LOG(log(0.1)){;}

  CornerLocGuts(const CornerLocGuts & other):
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
    return new CornerLocGuts(*this);
  }
};

#endif
