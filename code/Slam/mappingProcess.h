#ifndef __MAPPING_PROCESS_H__
#define __MAPPING_PROCESS_H__

#include "map.h"
#include "odoModel.h"
#include "matlab.h"

class MapMatcherInterface;
class LocaliserParticleGuts;

typedef void (*Robot2SensorFunctionPointer)(RobotPose*,const RobotPose*);

class MappingProcessInterface{
 protected:
  Robot2SensorFunctionPointer Robot2Sensor;

 public:

  const MotionModelInterface   * odoModel;
  const SensorRangeInterface   * sensorRange;
  const MapBuilderInterface    * mapBuilder;
  const LocaliserParticleGuts  * localiserGuts;

  MapMatcherInterface          * matcher;

  MappingProcessInterface():Robot2Sensor(NULL)
                          , odoModel(NULL),sensorRange(NULL)
                          , mapBuilder(NULL)
                          , localiserGuts(NULL), matcher(NULL){;}

  //For Particles
  void setRobot2SensorFunction(Robot2SensorFunctionPointer p){
    Robot2Sensor = p;
  }

  void robot2sensor(RobotPose *sensor, 
                     const RobotPose* robot)const{
    Robot2Sensor(sensor,robot);
  }

  //For Map Builder

  virtual MapEntryInterface * makeNewEntry(double *logProb,
                                   const ObservationStore &, int ind,
					   const RobotPose*)const{
    return NULL;
  }

  virtual RangeInterface* getNeighbourhoodRange(const RobotPose*){
    return NULL;
  }


  //Perform data association
  //   corr_out[i] -- should be set to index of the map element from
  //                  which observation i is originating,
  //                  -1 indicates no data association.
  //   obs,nObs    -- Array of observations and it's size
  //   robot       -- Pose of the robot
  //   map         -- Neighbourhood map
  // 
  // Return value: log(Prob) of the obs to map match.
  virtual double dataAssociate(int *corr_out
                               ,const ObservationInterface* const* obs, int nObs
			       ,const RobotPose* robot
			       ,const MapInterface* map){
    return 0.0;
  }

  //For Local Map
  virtual SimpleMap* loadLocalMap(const MatlabVariable&){return NULL;}
  //storeMap should also be here

  //For Global Map
  virtual bool isNeighbour(int nHops, const RobotPoseCov* mapRef,
			   const MapAreaInterface* region){return false;}

  //  virtual void getSuitableCoreLandmarks(int *core, const SimpleMap* m){;}


  //For Mapping Agent

  //   Compute Map Area and pose of the robot within it
  virtual void getInitialMapArea(MapAreaInterface **mapArea,
				 RobotPose* robot){;}
  virtual double computeRegionScore(const MapAreaInterface*, const RobotPose*)
    {return 0.0;}

  virtual double subtractFromRegion(MapAreaInterface* ,
                                    const MapAreaInterface *)
    {return 0.0;}

  virtual double subtractFromRegion(MapAreaInterface* ,
                                    const MapAreaInterface **,
                                    const RobotPose *, int n){
    return 0.0;
  }

  virtual void finaliseMapArea(MapAreaInterface*, const RobotPose&){
    ;
  }
  virtual void finaliseMapArea(MapAreaInterface*){
    ;
  }

  virtual void finaliseMapArea(MapAreaInterface*,const SimpleMap*){;}

  virtual void sampleTransition(const SimpleMap* m1, const SimpleMap* m2,
				const RobotPoseCov *m2in1,
				RobotPose *samples, double *weight
                                , int nsamples){;}

};

extern MappingProcessInterface * mappingProcess;
#endif
