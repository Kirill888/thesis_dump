#ifndef __LANDMARK3D_H__
#define __LANDMARK3D_H__

#include "geometry.h"
#include "DataStore.h"
#include "map.h"
#include "mappingProcess.h"

extern void robot2sensor(RobotPose* sensor, const RobotPose robot);

//---------------------------------------------------------------
// Landmark3dMappingProcess
//---------------------------------------------------------------
class Landmark3dMappingProcess: public MappingProcessInterface{
 public:
  double MAX_MD2;

  Landmark3dMappingProcess();

  //For Map Builder

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

};

//---------------------------------------------------------------
// Landmark3dObs
//---------------------------------------------------------------
class Landmark3dObs: public ObservationInterface{

 public:
  double range;
  double bearing;
  double azimuth;
  double cov[3][3];

  int id;

  Landmark3dObs():range(0), bearing(0),azimuth(0),id(-1){
    memset(cov,0, sizeof(double)*3*3);
  }

  Landmark3dObs(double r, double b, double a):
    range(r), bearing(b), azimuth(a),id(-1){
    memset(cov,0, sizeof(double)*3*3);
  }

  Landmark3dObs(double r, double b, double a, const double COV[3][3],int ID=-1):
    range(r), bearing(b), azimuth(a),id(ID){
 
   copy_cov(cov,COV);
  }

  ObservationInterface *clone()const{ 
    return new Landmark3dObs( range, bearing, azimuth, cov,id);
  }

  
  double logLikelihood(const ObservationInterface*)const;

  Gaussian3d toCartesian(const RobotPose *r)const;
  Gaussian3d toCartesian()const;

  int getObsType()const{return 1;}

  bool matlabDump(FILE *f)const{
    return fprintf(f,"%e %e %e %e %e %e %e %e %e %e %e %e"
		   ,range
		   ,bearing
		   ,azimuth
		   ,cov[0][0], cov[0][1] , cov[0][2]
		   ,cov[1][0], cov[1][1] , cov[1][2]
		   ,cov[2][0], cov[2][1] , cov[2][2]
    ) > 0;
  }

 private:

  static void copy_cov(double cov[3][3], const double COV[3][3]){
    cov[0][0] = COV[0][0];
    cov[0][1] = COV[0][1];
    cov[0][2] = COV[0][2];
    cov[1][0] = COV[1][0];
    cov[1][1] = COV[1][1];
    cov[1][2] = COV[1][2];
    cov[2][0] = COV[2][0];
    cov[2][1] = COV[2][1];
    cov[2][2] = COV[2][2];
  }


};

//---------------------------------------------------------------
// Landmark3d
//---------------------------------------------------------------
class Landmark3d: public MapEntryInterface{

 public:
  Gaussian3d point;

  Landmark3d(){;}

  Landmark3d(const Gaussian3d &p): point(p){;}

  Landmark3d(const Landmark3d *other):point(other->point){
    MapEntryInterface::set(other);
    obs = other->obs;
  }

  Landmark3d(const Landmark3d &other):point(other.point){
    MapEntryInterface::set(&other);
    obs = other.obs;
  }

  MapEntryInterface * clone()const{
    return new Landmark3d(this);
  }


 void translateMe(const RobotPose &pose){
   point.translateMe(pose);
 }

 void translateMe(double x, double y, double ca, double sa){
    point.translateMe(x,y,ca,sa);
  }

 Gaussian2d getCenter()const{return point;}
 void getCenter(double *x, double* y){ *x = point.x; *y = point.y;}


 double probMatch(const MapEntryInterface* )const;
 double probMatchLog(const MapEntryInterface*)const;
 bool matlabDump(FILE *f)const;

 void update(const ObservationStore &obs_store,
	     int obs_ind, 
	     const RobotPose *robot_pose);

 GenericStack obs;
 void addObs(int ind){ obs.add((void*)ind);}

};

void dumpDataAssociations(FILE*f, const SimpleMap* m);

//---------------------------------------------------------------
// Landmark3dSensorRange
//---------------------------------------------------------------

class Landmark3dSensorRange: public SensorRangeInterface{
  double minRange;
  double dRange;
  double minAngle1;
  double dAngle1;
  double minAngle2;
  double dAngle2;


 public:
  Landmark3dSensorRange():minRange(0),dRange(50.0)
                          ,minAngle1(-M_PI/6.0),dAngle1(M_PI/3.0)
                          ,minAngle2(-M_PI/6.0),dAngle2(M_PI/3.0){;}

  Landmark3dSensorRange(double minR, double maxR, 
			double minA, double maxA,
			double minB, double maxB
                        )
                            :minRange(minR) ,dRange(maxR - minR)
                            ,minAngle1(minA),dAngle1(maxA - minA)
                            ,minAngle2(minB),dAngle2(maxB - minB)
                            {;}

  bool isWithinRange(const ObservationInterface* o)const;

};


#endif
