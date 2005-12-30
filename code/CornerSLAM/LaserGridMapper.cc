#include "LaserGridMapper.h"
#include "DROSGridMap.h"
#include "GCEventLoop.h"
#include "hough.h"
#include "AngleTools.h"

#define PRINT_TIME_STATS 1
extern "C" double GetCurrentTime();

extern void robot2sensor(RobotPose* sensor, const RobotPose* robot);
DROSGridMap  *drosGridMap;

void initDROSGridMap(double x0, double y0
		     ,double x1, double y1, double step){

  drosGridMap = new DROSGridMap("grid",x0,y0,x1,y1, step);
  GCEventLoop loop("MainLoop");
  loop.AddToComponentList(drosGridMap);

  if (loop.EstablishConnections() < 0) {
    fprintf(stderr,"Failed to establish connections");
    return;
  }
}

//=============================================================================
double LaserFunc(double radiusfrac, double anglefrac, void *userdata) {

  double *r = (double*) userdata;
  double dr = (1 - radiusfrac)*(*r);

  if (dr > 0.07)
    return -0.2 / (1.0 + fabs(anglefrac));  //Empty space
  else
    return 0.4 / (1.0 + fabs(anglefrac));   //Occupied space
}

//=============================================================================

double LaserFunc2(double radiusfrac, double anglefrac, void *userdata) {
  double *r = (double*) userdata;
  double dr = (1 - radiusfrac)*(*r);

  if (dr > 0.07)
    return -0.2 / (1.0 + fabs(anglefrac));  //Empty space
  else
    return 0.0;                             //Occupied is not really occupied
}
//=============================================================================

void compute_hough(Hough* h, const LaserSource &las, int step){
  int i;

  if(step > 0){
    for(i = 0; i < las.numPoints() - step; ++i){
      if(las.isValidReading(i)        && !las.noReturn(i) && 
         las.isValidReading(i + step) && !las.noReturn(i + step) ){
	h->add(las.x(i), las.y(i), las.x(i+step), las.y(i+step));
      }
    }
  }else{
    for(i = 0; i < las.numPoints() - 1; ++i){
      if(las.isValidReading(i) && !las.noReturn(i)){
	int j;
	double x0 = las.x(i);   double y0 = las.y(i);
	for(j = i; j < las.numPoints(); ++j){
	  if(las.isValidReading(j) && !las.noReturn(j)) 
             h->add(x0,y0,las.x(j), las.y(j));
	}
      }
    }
  }
}
//=============================================================================
void applyFilter(LaserSource &las){
  const double* r0 = las.getR();
  int n = las.numPoints();
  double r[n];

  int WIN_SIZE   = 13;
  int WIN_SIDE   = WIN_SIZE/2;
  int WIN_CENTRE = WIN_SIDE + 1;
  int i,j;
  double window[WIN_SIZE];
  double MAX_ERR = 0.5;
  double MAX_READING = 10000;

  for(i = 0; i < WIN_SIDE; ++i)   r[i] = r0[i];
  for(i = n-WIN_SIDE; i < n; ++i) r[i] = r0[i];

  for(i = WIN_SIDE; i < n - WIN_SIDE; ++i){
    for(j = 0; j < WIN_SIZE; ++j){
      int ii = i + j - WIN_SIDE;
      if(!las.isValidReading(ii)){
	window[j] = -MAX_READING;
      }else if(las.noReturn(ii)){
	window[j] = MAX_READING;
      }else{
        window[j] = r0[ii];
      }
    }

    r[i] = window[WIN_CENTRE];

    double med = kth_smallest(window, WIN_SIZE, WIN_CENTRE);
    double err = fabs(r[i] - med);

    if(err > MAX_ERR){
      r[i] = med;
    }

    if(r[i] > las.getMaxRange()){
      r[i] = las.getMaxRange();
    }
  }

  double aa[n];
  int ii[n];

  for(i = 0; i < n; ++i){
    aa[i] = las.a(i);
    ii[i] = las.Intensity(i);
  }

  las.setData(r,aa,ii, n, las.TimeStamp());
}

//=============================================================================

void LaserGridMapper::processScan(const LaserSource &las, 
                                  const RobotPose & robot)const{
  int i;
  RobotPose sensor;
  robot2sensor(&sensor, &robot);

  printf("ProcessScan: %e %e %e\n",robot.x,robot.y,robot.rot*RADTODEG);

  for(i = 0; i < las.numPoints(); ++i){
    if(las.isValidReading(i)){
      double a = sensor.rot + las.a(i) - M_PI/2;
      double r = las.r(i) + expansion;

      if(las.noReturn(i)){
	drosGridMap->AddReading(sensor.x, sensor.y, 
				a, r, halfBeamWidth, LaserFunc2, &r);
      }else{
	drosGridMap->AddReading(sensor.x, sensor.y, 
				a, r, halfBeamWidth, LaserFunc, &r);
      }
    }
  }

  drosGridMap->CircleFill(robot.x, robot.y, 0.4, 0.0);
}

GridMap* LaserGridMapper::computeGridMap(RobotPose *robot)const{
  int i0 = tail - 1;  if(i0 < 0) i0 += szHistory;

  int i,j;
  double lasMaxRange = buffer[i0].las.getMaxRange();

  //Current robot pose is 0,0
  double x0,y0,a0,ca,sa;

#if PRINT_TIME_STATS
  double T0 = GetCurrentTime();
  double T00 = T0;
#endif


  if(align){
    //First find the peak line in the first scan
    Hough hough(0, lasMaxRange, 0.1, -M_PI, M_PI, 0.3*DEGTORAD);
    compute_hough(&hough, buffer[i0].las, 2);

    double a,r;
    hough.findmax(&a,&r);
    printf("HOUGH: %.2f (deg) %.2g (m)\n", a*RADTODEG, r); fflush(stdout);

    if(a < 0)      a += M_PI;
    if(a > M_PI/2) a -= M_PI/2;

    if(a < M_PI/4) a = -a;
    else           a = M_PI/2 - a;

    x0 = buffer[i0].robot.x;
    y0 = buffer[i0].robot.y;
    a0 = angleDiffRad(buffer[i0].robot.rot, a);
  }else{
    x0 = buffer[i0].robot.x;
    y0 = buffer[i0].robot.y;
    a0 = buffer[i0].robot.rot;
  }

#if PRINT_TIME_STATS
    printf("\t %.3fms\n",(GetCurrentTime()-T0)*1000);
    T0 = GetCurrentTime();
#endif


  ca = cos(a0);
  sa = sin(a0);



  //Compute Grid Map
  drosGridMap->Fill(0.5);

  for(i = 0; i < szHistory; ++i){
    double dx,dy;

    RobotPose odo = buffer[i].robot;
    dx = odo.x - x0;
    dy = odo.y - y0;

    odo.x =  dx*ca + dy*sa;
    odo.y = -dx*sa + dy*ca;
    odo.rot = angleDiffRad(odo.rot, a0);

    applyFilter(buffer[i].las);
    processScan(buffer[i].las, odo);

    if(i == i0) robot->rot = odo.rot;
  }

#if PRINT_TIME_STATS
    printf("\t %.3fms\n",(GetCurrentTime()-T0)*1000);
    T0 = GetCurrentTime();
#endif

  //Shrink Grid Map
  int poly[2][2] = {{drosGridMap->GetXSize(), drosGridMap->GetYSize()},{0,0}};

  for(i = 0; i < (int)drosGridMap->GetXSize(); ++i)
    for(j = 0; j < (int)drosGridMap->GetYSize(); ++j){
      double v = drosGridMap->Get(i,j);
      //Round off, one way only
      if(v > 0.49) v = 1;

      drosGridMap->Set(i,j,v);

      if(v < 0.3){
	//Update min bounds
	if(i < poly[0][0]) poly[0][0] = i;
	if(j < poly[0][1]) poly[0][1] = j;
	//Update max bounds
	if(i > poly[1][0]) poly[1][0] = i;
	if(j > poly[1][1]) poly[1][1] = j;
      }
    }

  printf("Bounding box: [%d %d] -> [%d %d]\n",
	 poly[0][0], poly[0][1],
	 poly[1][0], poly[1][1]);

  int nx,ny;
  double xl,yl,dx,dy;

  nx = poly[1][0] - poly[0][0] + 1;
  ny = poly[1][1] - poly[0][1] + 1;

  dx = drosGridMap->GetXMin() + drosGridMap->GetGridCellSize()*poly[0][0];
  dy = drosGridMap->GetYMin() + drosGridMap->GetGridCellSize()*poly[0][1];

  xl = drosGridMap->GetGridCellSize()*nx;
  yl = drosGridMap->GetGridCellSize()*ny;

  GridMap *gm = new GridMap(dx, xl, dy, yl,nx,ny);

  for(i = poly[0][0]; i <= poly[1][0]; ++i)
    for(j = poly[0][1]; j <= poly[1][1]; ++j)
      gm->set(i - poly[0][0], j - poly[0][1], 1 - drosGridMap->Get(i,j));

#if PRINT_TIME_STATS
    printf("\t %.3fms\n",(GetCurrentTime()-T0)*1000);
    printf("\t Total: %.3fms\n",(GetCurrentTime()-T00)*1000);
#endif

  return gm;
}

void LaserGridMapper::debugDump(const char* fname )const{
  static int id = 1;
  char buf[1024];
  int i;
  FILE * f;

  if(fname == NULL){

    sprintf(buf,"grid_data_%04d.m",id);
    id += 1;

    fname = buf;
  }

  f = fopen(fname,"w");
  if(f == NULL){
    fprintf(stderr,"Debug Log failed to open file: %s\n", fname);
    return;
  }

  fprintf(f,"settings = [%d %g %g]; %% sz, half_beam, expansion\n"
         , szHistory, halfBeamWidth, expansion);

  fprintf(f,"odo = [...\n");
  for(i = 0; i < szHistory; ++i){
    fprintf(f,"%g %g %g\n"
            , buffer[i].robot.x
            , buffer[i].robot.y
            , buffer[i].robot.rot);
  }
  fprintf(f,"];\n");

  fprintf(f,"las = [...\n");
  for(i = 0; i < szHistory; ++i){
    int j;
    const LaserSource &las = buffer[i].las;

    for(j = 0; j < las.numPoints(); ++j)
      fprintf(f," %g", las.r(j));

    fprintf(f, "\n");
  }
  fprintf(f,"];\n");

  fclose(f);

}

