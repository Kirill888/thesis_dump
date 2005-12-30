#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "icp.h"
#include "DataSource.h"

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
  }else{
    printf("%%Opened: %s \n", argv[1]);
  }

  ICPScanMatch icp;
  LaserScan *scan     = NULL;
  LaserScan *prevScan = NULL;

  double odo[3] =  {0,0,0};
  double step[3] = {0,0,0};
  int iScan = 0;

  while(las.nextScan() && iScan < 200){
    scan = getScan(las);
    scan->computeCentroid();

    if(prevScan != NULL){
      
      int np = icp.match(*scan, *prevScan, 0.01, 0.999, step, odo);

      double ca = cos(odo[2]);
      double sa = sin(odo[2]);

      odo[0] = step[0];
      odo[1] = step[1];
      odo[2] = step[2];

      printf("%+10.3f %+10.3f %+10.8f \t\t %+10.3f %+10.3f %+10.8f %d\n"
            , odo[0], odo[1], odo[2]
            , step[0], step[1], step[2], np);
      fflush(stdout);

#if 1
      ca = cos(odo[2]);
      sa = sin(odo[2]);
      unsigned int i;
      for(i = 0; i < scan->n; ++i){
	double x,y;
	x = scan->x[i]*ca - scan->y[i]*sa + odo[0];
	y = scan->x[i]*sa + scan->y[i]*ca + odo[1];
	fprintf(fscans,"%.3e %.3e %.3e %.3e %d %d\n", x,y, scan->x[i],scan->y[i], np, scan->n);
      }

#endif
      iScan += 1;
      delete scan;
    }else{
      prevScan = scan;
      prevScan->buildTree();
    }
  }


  if(prevScan != NULL) delete prevScan;

  fclose(fscans);

  return 0;
}

const double NaN = atof("+NAN"); //SPAM
const double Inf = HUGE_VAL;

int main(int argc, char *argv[]){

  return ICP_Test(argc,argv);
}
