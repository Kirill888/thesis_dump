#ifndef __POLE_LANDMARK_H__
#define __POLE_LANDMARK_H__

#include "landmark2d.h"
#include "mappingProcess.h"
#include "stereoPair.h"
#include "image.h"
#include "gridmap.h"

#define DEFAULT_MAX_MD2 4.0
//7 pixels 3 colours 1 byte each
#define EDGE_TEMPLATE_SIZE 21 

class EdgeMappingProcess: public MappingProcessInterface{
 private:
  BYTE* robotMask;
  int robotMask_w;
  int robotMask_h;

  void initRobotMask(double cell_sz);

  GridMap *region0;
  void initDefaultMapRegion(double cell_sz);

 public:
  double MAX_MD2;
  StereoPair *cam;


  EdgeMappingProcess();

  //For Map Builder

  MapEntryInterface * makeNewEntry(double *logProb,
                                   const ObservationStore &, int ind,
				   const RobotPose*)const;

  RangeInterface* getNeighbourhoodRange(const RobotPose*){return NULL;}

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
  SimpleMap* loadLocalMap(const MatlabVariable&){return NULL;}

  //For MultiSLAM
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

  bool isNeighbour(int nHops, const RobotPoseCov* mapRef,
		   const MapAreaInterface* region);


};

class EdgeObservation: public ObservationInterface{
 public:
  double u1;
  double u2;
  double h1;
  double h2;
  double corr;

  double cov[2][2];
  signed char Template[EDGE_TEMPLATE_SIZE];

  EdgeObservation():u1(0),u2(0),h1(0),h2(0),corr(0){
    matrix2_set0(cov);
  }

  EdgeObservation(double u1, double u2
                  ,double h1, double h2
                  , double corr, const double COV[2][2])
   :u1(u1),u2(u2),h1(h1),h2(h2), corr(corr){
    matrix2_set(cov,COV);
  }

  EdgeObservation(double u1, double u2
		  ,double h1, double h2
                  ,double corr, const double COV[2][2]
                  ,const signed char *t)
   :u1(u1),u2(u2),h1(h1),h2(h2), corr(corr){
    matrix2_set(cov,COV);
    memcpy(Template, t, EDGE_TEMPLATE_SIZE);
  }

  ObservationInterface *clone()const{ 
    return new EdgeObservation(u1,u2,h1,h2,corr, cov, Template);
  }

  bool matlabDump(FILE*f)const{
    fprintf(f,"%e %e %e %e %e %e %e %e"
	    ,u1 , u2
	    ,cov[0][0], cov[0][1]
	    ,cov[1][0], cov[1][1]
	    ,h1 , h2
	    );
    int i;
    for(i = 0; i < EDGE_TEMPLATE_SIZE; ++i){
      fprintf(f," %02d", (int)Template[i]);
    }
    return true;
  }

  void toCartesian(Gaussian2d *point, 
                   const StereoPair*, const RobotPose*)const;

  //Value is between -2^14 -> +2^14
  int correlate(const EdgeObservation* other)const;
};

class EdgeLandmark: public Landmark2d{
 public:
  GenericStack obs;

  EdgeLandmark(){;}

  EdgeLandmark(const EdgeLandmark &other):Landmark2d(other){
    obs = other.obs;
  }
  EdgeLandmark(const Gaussian2d p):Landmark2d(p){;}

  MapEntryInterface * clone()const{
    return new EdgeLandmark(*this);
  }


  void update(const ObservationStore &obs_store,
	      int obs_ind, 
	      const RobotPose *robot_pose);

  //  bool matlabDump(FILE* f)const;

  double projectToImagePlane(double h[2], 
                             const CameraModel *cam, 
                             const RobotPose *pose)const;

  //Returns true if within image plane
  bool projectToImagePlane(double u[2]         //out: u1,u2
			   , double cov[2][2]  //out: covariance of u1,u2
			   , double H[2][2]    //out: jacobian used to 
			                       //compute covariance
			   , const RobotPose* robot //in: pose of 1st camera
			   , const StereoPair *cam)const;//in: stero pair

  void addObs(int ind){ obs.add((void*)ind);}

};


bool Edge_storeLocalMap(FILE *f, const LocalMap *lmap);
void dumpDataAssociations(FILE*f, const SimpleMap* m);

#endif
