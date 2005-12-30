#include "DataSource.h"
#include "geometry.h"
#include "icp.h"

LaserScan *getScan(const LaserSource& las){
  double *x = new double[las.numPoints()];
  double *y = new double[las.numPoints()];
  int i;
  unsigned int n = 0;

  for(i = 0; i < las.numPoints(); ++i){
    if(las.isValidReading(i) && !las.noReturn(i)){
      x[n] =  las.y(i);
      y[n] = -las.x(i);

      //SPAM:
      if(isnan(x[n]) || isnan(y[n])){ 
	//printf("Wrong data point: %d, %g,%g\n", n, x[n],y[n]);
      }else{
	n++;
      }
    }
  }

  return new LaserScan(x,y,n);
}

int ICP_Test(int argc, char **argv){
  if(argc != 3){
    fprintf(stderr,"Usage: %s las.log scan.out\n", argv[0]);
    return -1;
  }

  FileLaserSource las(argv[1],8.0);
  FILE *fscans = fopen(argv[2],"w");

  if(!las.isOk()){
    fprintf(stderr,"Failed to open: %s\n", argv[1]);
    return -1;
  }

  ICPScanMatch icp;
  LaserScan *scan     = NULL;
  LaserScan *prevScan = NULL;

  RobotPose odo(0,0,0);
  double step[3] = {0,0,0};

  while(las.nextScan()){
    scan = getScan(las);
    scan->computeCentroid();

    if(prevScan != NULL){
      icp.match(*scan, *prevScan, 0.01, 0.999, step, NULL);

      double ca = cos(odo.rot);
      double sa = sin(odo.rot);

      odo.x   += (ca*step[0] - sa*step[1]);
      odo.y   += (sa*step[0] + ca*step[1]);
      odo.rot += step[2];

      printf("%+10.3f %+10.3f %+10.3f \t\t %+10.3f %+10.3f %+10.3f\n"
            , odo.x, odo.y, odo.rot
            , step[0], step[1], step[2]);
      fflush(stdout);

#if 1
      ca = cos(odo.rot);
      sa = sin(odo.rot);
      unsigned int i;
      for(i = 0; i < scan->n; ++i){
	double x,y;
	x = scan->x[i]*ca - scan->y[i]*sa + odo.x;
	y = scan->x[i]*sa + scan->y[i]*ca + odo.y;
	fprintf(fscans,"%.3e %.3e\n", x,y);
      }
#endif

#if 0
      if(fabs(step[0]) > 0.03 || fabs(step[1]) > 0.03 
         || fabs(step[2]) > 0.175*2 ){

	prevScan->matlabDump(fscans, "scan0");
	scan->matlabDump(fscans,"scan1");
	fprintf(fscans,"odo = [ %e %e %e ];\n", step[0], step[1], step[2]);

	return -1;
      }
#endif

      delete prevScan;
    }

    scan->buildTree();
    prevScan = scan;
  }

  if(prevScan != NULL) delete prevScan;

  fclose(fscans);

  return 0;
}

int main(int argc, char *argv[]){
  return ICP_Test(argc,argv);
}
