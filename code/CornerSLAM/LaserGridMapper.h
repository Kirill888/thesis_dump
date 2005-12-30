#ifndef __LASER_GRID_MAPPER__
#define __LASER_GRID_MAPPER__

#ifndef DEPEND
#endif

#include "DataSource.h"
#include "gridmap.h"
#include "DROSGridMap.h"

extern void initDROSGridMap(double x0, double y0
                           ,double x1, double y1, double step);

class LaserGridMapper{
 private:
  class Reading{
  public:
    RobotPose   robot;
    LaserSource las;
  };

  double halfBeamWidth;
  double expansion;
  bool align;

  int szHistory;
  int tail;
  Reading *buffer;

 public:

  LaserGridMapper():halfBeamWidth(4.363323129985824e-03),expansion(0)
                   ,align(true), szHistory(5),tail(0){
    buffer = new Reading[szHistory];
  }

  LaserGridMapper(int nScans, double halfBeam, double expand, bool doAlign):
    halfBeamWidth(halfBeam), expansion(expand),align(doAlign),
    szHistory(nScans),tail(0){
    buffer = new Reading[szHistory];
  }

  void addReading(const LaserSource &las, const RobotPose *robot){
    buffer[tail].robot = *robot;
    buffer[tail].las   = las;

    //    printf("Add reading: %d/%d\n", tail, szHistory);

    tail = (tail + 1) % szHistory;
  }

  //Here robot pose will setto the current robot pose within the
  //coordinate frame of the grid map.
  GridMap* computeGridMap(RobotPose *robot)const;

  void debugDump(const char* fname = NULL)const;

 private:
  void processScan(const LaserSource &las, const RobotPose &robot)const;

};

#endif
