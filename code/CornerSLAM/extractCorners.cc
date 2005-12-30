#include "LaserCornerExtractor.h"
#include "DataSource.h"


extern RobotPose LASER_SENSOR_POSE;

void robot2sensor(RobotPose* sensor, const RobotPose *robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}


int main(int argc, char **argv){
  if(argc < 2){ 
    fprintf(stderr,"Usage: extractCorners las.log [out] [thresh]\n"); 
    return 1;
  }

  char *out = NULL;
  FILE *f = stdout;
  double thresh = 0.2;

  double maxLaser = 8.18;

  FileLaserSource las(argv[1], maxLaser);

  if(!las.isOk()){
    fprintf(stderr,"Failed to open: %s\n",argv[1]);
    return -1;
  }

  if(argc > 2){
    out = argv[2];
    f = fopen(out,"w");
    if(f == NULL){
      fprintf(stderr,"Failed to open: %s\n", out);
      return 1;
    }
  }

  if(argc > 3) thresh = atof(argv[3]);

#if 1
  //Kirill's version.


  LaserCornerDetector ce;

  int i;
  int nc;
  int scanId = 0;

  while(las.nextScan()){
    scanId += 1;

    nc = ce.ExtractCorners(las);

    for(i = 0; i < nc; ++i){
      fprintf(f,"%d %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %d\n"
	      , scanId
              , ce.X(i), ce.Y(i)
	      , ce.XTangentLeft(i) , ce.YTangentLeft(i)
	      , ce.XTangentRight(i), ce.YTangentRight(i)
	      , ce.CornerType(i) - 42000);

    }

  }


#else
  //DROS version.

  LaserCornerDetectorMain ce;

  double *x = new double[361];
  double *y = new double[361];
  int n;
  int i;
  int nc;
  int scanId = 0;

  while(las.nextScan()){
    scanId += 1;

    n = 0;

    for(i = 0; i < las.numPoints(); ++i){
      if(las.isValidReading(i) && !las.noReturn(i)){
	x[n] = las.x(i);
	y[n] = las.y(i);
	n += 1;
      }
    }

    ce.ClearCorners();

    nc = ce.ExtractCorners(x,y,n, thresh);

    for(i = 0; i < nc; ++i){
      fprintf(f,"%d %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %d\n"
	      , scanId
              , ce.X(i), ce.Y(i)
	      , ce.XTangentLeft(i) , ce.YTangentLeft(i)
	      , ce.XTangentRight(i), ce.YTangentRight(i)
	      , ce.CornerType(i) - 42000);

    }
  }
#endif



  return 0;
}


