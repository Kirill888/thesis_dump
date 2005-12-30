#ifndef DEPEND
#include <math.h>
#include <time.h>
#include <signal.h>
#endif

#include "TimeTools.h"
#include "DataSource.h"
#include "odoXR4000.h"
#include "gridmap.h"
#include "AngleTools.h"
#include "DROSGridMap.h"
#include "GCEventLoop.h"
#include "hough.h"
#include "LaserGridMapper.h"
#include "image.h"
#include "geometry.h"

#include "corner.h"

bool stop = false;
MotionModel robot;
RobotPose currentPose(0,0,0);

int scanId = 0;

LaserGridMapper *gridMapper;

const double HALF_BEAM_WIDTH = 0.25*DEGTORAD;
int fromScan = 0;
int toScan   = 10;

Hough hough(0,10,0.05, -M_PI, M_PI, 0.3*DEGTORAD);

//=============================================================================
extern RobotPose LASER_SENSOR_POSE;
void robot2sensor(RobotPose* sensor, const RobotPose* robot){
  sensor->set(LASER_SENSOR_POSE);
  sensor->translateMe(*robot);
}

void processScan(const LaserSource &las, const RobotPose & robot){
  printf("Adding Scan\n");
  gridMapper->addReading(las, &robot);
}

void lasEvent(const LaserSource &las){  
  scanId += 1;

  RobotPose odo   = currentPose;

  printf("LAS: %.2f %.2f %.2f [%4d]\n",odo.x, odo.y, odo.rot*RADTODEG
         ,scanId);

  if(scanId >= fromScan){
    processScan(las,odo);
  }

  if(scanId >= toScan) stop = true;

}

void odoEvent(const OdoSource &odo){
  RobotPoseOdo o((RobotPose)odo, odo.TimeStamp());

  robot.newOdometry(&o);
  printf("ODO: %.2f %.2f %.2f\n",odo.x(), odo.y(), odo.a()*RADTODEG);

  if(robot.hasMoved()){
    robot.getControlInput()->advance(&currentPose);
  }
}

extern FILO* m1;
extern FIFO* m2;

void permut(double w, int lvl = 0);

void permut_test(int argc, char** argv){
  int n1 = 5;
  int n2 = 7;
  int i;

  if(argc > 1){ n1 = atoi(argv[1]); }
  if(argc > 2){ n2 = atoi(argv[2]); }

  FILO m1(n1);
  FIFO m2(n2);

  for(i = n1-1;i >= 0; --i) m1.add(i);

  for(i = 0;i < n2; ++i) m2.add(i);

  printf("M1 = ");
  m1.dump();
  printf("\n");

  printf("M2 = ");
  m2.dump();
  printf("\n");

  ::m1 = &m1;
  ::m2 = &m2;
  permut(0.0 , 0);
}

void image_test(int argc, char **argv){
  BYTE mask[3][3] = {{0,1,0},
		     {1,1,1},
		     {0,1,0}};
  int nc,nr;

  if(argc < 3){
    printf("Args missing: nr,nc\n");
    return;
  }

  nr = atoi(argv[1]);
  nc = atoi(argv[2]);

  BYTE* im0 = new BYTE[nc*nr];
  BYTE* im  = new BYTE[nc*nr];
  int i;

  for(i = 0; i < nc*nr; ++i){
    int v = rand()%10;

    if(v < 7) im0[i] = 0;
    else      im0[i] = v - 7;
  }

  printf("%%Before\n");
  for(i = 0; i < nc*nr; ++i){
    printf(" %d", (int)im0[i]);

    if(((i+1) % nc) == 0) printf("\n");
  }

  image_dilate(im, im0, nr, nc, (BYTE*) mask, 3, 3);

  printf("%%After\n");
  for(i = 0; i < nc*nr; ++i){
    printf(" %d", (int)im[i]);

    if(((i+1) % nc) == 0) printf("\n");
  }

}

void gauss_test(int argc, char** argv){
  if(argc < 6){
    printf("Gauss test: x y sxx sxy syy [np]\n");
  }
  Gaussian2d g;
  int np = 100;
  int i;

  g.x = atof(argv[1]);
  g.y = atof(argv[2]);

  g.cov[0][0] = atof(argv[3]);
  g.cov[0][1] = g.cov[1][0] = atof(argv[4]);
  g.cov[1][1] = atof(argv[5]);

  if(argc > 6) np = atoi(argv[6]);

  Gaussian2dSampler sampler;
  sampler.set(g);

  for(i = 0; i < np; ++i){
    double x,y;

    sampler.sample(&x,&y);

    printf("%+10e %+10e\n",x,y);
  }

}

int gridMap_test(int argc, char** argv){
  char *file_odo = NULL;
  char *file_las = NULL;

  if(argc < 3){
    fprintf(stderr,"gridMapTest: odo las [from to]\n");
    return -1;
  }

  if(argc > 3){ fromScan = atoi(argv[3]);  }
  if(argc > 4){ toScan = atoi(argv[4]);  }

  file_odo = argv[1];
  file_las = argv[2];

  //Init Grid Mapper
  initDROSGridMap(-10,-10,10,10, 0.1);
  gridMapper = new LaserGridMapper(toScan - fromScan + 1, 
                                   HALF_BEAM_WIDTH,0.05, true);

  //Run Simulation
  FileOdoSource fodo(file_odo);
  FileLaserSource flas(file_las,8.1);

  run_simulation(fodo,odoEvent, flas, lasEvent, &stop);

  printf("Computing Grid Map:\n"); fflush(stdout);

  RobotPose robot;
  GridMap *gm = gridMapper->computeGridMap(&robot);

  gm->writePgm("grid0.pgm");
  printf("Pose: %.2f %.2f %.2f\n", robot.x, robot.y, robot.rot*RADTODEG);

  BYTE mask[3][3] = {{1,1,1},
		     {1,1,1},
		     {1,1,1}};

  image_dilate(gm->getData(),gm->getData(),gm->numY(), gm->numX()
              , (BYTE*) mask, 3, 3);

  gm->writePgm("grid1.pgm");

  delete gm;

  return 0;
}

int volume_test(int argc, char** argv){
  Gaussian2d g1, g2;

  if(argc < 2){
    fprintf(stderr,"VolumeTest: 'x1,y1,sxx1,sxy1,syy1' "
                    "'x2,y2,sxx2,sxy2,syy2'\n");    
    return -1;
  }

  sscanf(argv[1],"%lf,%lf,%lf,%lf,%lf", &g1.x,&g1.y
         ,&g1.cov[0][0], &g1.cov[0][1], &g1.cov[1][1]);
  g1.cov[1][0] = g1.cov[0][1];

  sscanf(argv[2],"%lf,%lf,%lf,%lf,%lf", &g2.x,&g2.y
         ,&g2.cov[0][0], &g2.cov[0][1], &g2.cov[1][1]);
  g2.cov[1][0] = g2.cov[0][1];

  double vol = volumeOfTheProduct(g1,g2);
  double logVol = logVolumeOfTheProduct(g1,g2);

  print_obs("g1", g1);
  print_obs("g2", g2);

  printf("Vol: %.8f %.8f\n", vol, logVol);

  return 0;
}

int rand_test(int argc, char**argv){
  double mean;
  double std;
  int np = 1000;
  int i;

  if(argc < 3){
    fprintf(stderr,"rand_test: mean std [np]\n");
    return 0;
  }

  mean = atof(argv[1]);
  std  = atof(argv[2]);

  if(argc > 3) np = atoi(argv[3]);

  GaussianRandomNumber g(mean,std);

  for(i = 0; i < np ; ++i){
    printf("%.12e\n", g.nextRandom());
  }

  return 0;
}

int lasMatch_test(int argc, char**argv){
  int n = 12;

  double angle[] = {-1.747,-1.337,-0.892,-0.722,-0.470,-0.027,
                     0.405, 0.627, 0.701, 0.761, 0.816, 0.892};

  double range[] = {2.089,2.133,2.842,1.975,1.693,2.681,
                    1.165,2.418,2.656,2.881,3.658,3.950};
  int    match[n];
  double wlog[n];

  RobotPose sensor(2,3, deg2rad(45));
  LineSegment line(4,3, 2,6);
  struct LineLaserMatchParams params;
  params.sigma_a  = deg2rad(0.3);
  params.sigma_a2 = POW2(deg2rad(0.3));
  params.sigma_r  = 0.05;
  params.sigma_r2 = POW2(0.05);

  params.maxRange = 8.131;
  params.maxDistToLine = params.sigma_r*3;

  int res = matchScanToLine(wlog, match, 
                            range, angle, n, 
                            &sensor, &line, &params);

  int i;

  printf("Returned: %d\n",res);

  if(res == 0) return 0;

  for(i = 0; i < n; ++i){
    printf("match[%d] = %d ", i+1, match[i]);

    if(match[i] == 0){
      printf("[no match] ");
    }else if( match[i] == 1){
      printf("[matched], log(w) = %g", wlog[i]);
    }else if( match[i] == 2){
      printf("[coflict] log{prob.Intersect} = %g", wlog[i] );
    }else{
      printf("[unknown]");
    }

    printf("\n");
  }

  return 0;
}

int sqrtTst(int argc, char** argv){
  int i;
  for(i = 1; i < argc; ++i){
    int x = atoi(argv[i]);
    int xs = (int) sqrt((unsigned int) x);

    printf(" sqrt(%d) = %d\n", x,xs);
  }
  return 0;
}

void print(const char*s, const UpperTriangularMatrix &m){
  printf("%s\n", s);
  int i,j;

  for(i = 0; i < m.N(); ++i){
    for(j = 0; j < i; ++j){
      printf("   ");
    }

    for(j = i; j < m.N(); ++j){
      printf(" %2d", m(i,j));
    }

    printf("\n");
  }

}

int UpperTriagMatrixTst(int argc, char **argv){
  const int n = 5;
  UpperTriangularMatrix m(n,0);

  int i,j;

  int k = 0;

  print("M0",m);

  for(i = 0; i < n; ++i)
    for(j = i; j < n; ++j)
      m(i,j) = k++;

  print("M1",m);
  UpperTriangularMatrix m2(m);

  m2.resize(6,-1);
  print("M2",m2);

  m2.removeRowCol(4);
  print("M2 - row5",m2);

  return 0;
}

void lookForInterestingPoints(const double *dist2, const Line* lines, 
                              int n, int nfit
                              ,const double *range
                              ,const double *ca
                              ,const double *sa
                              ,const double *x
                              ,const double *y){

  const double maxAvgDistPerPoint = 0.005;
  const double goodLineThresh = 
          maxAvgDistPerPoint*maxAvgDistPerPoint*nfit;

  const double maxConvexDist = 0.1;
  const double minJumpDist   = 0.05;
  const double minRayPenetration = 0.2;

  const double CONVEX_ANGLE_THRESH = cos(deg2rad(30));
  const double COLLINEAR_THRESH    = cos(deg2rad(10));

  int i, lastGood;
  int n_data = n + nfit - 1;

  i = 0;
  printf("%%Thresh: %g\n", goodLineThresh);

  //Find first good line
  while(i < n && (isnan(dist2[i]) || dist2[i] > goodLineThresh)){ ++i; }

  if(i >= n) return;

  //  printf("%%Good: %d\n",i+1);

  do{
    //Find first bad line
    while(i < n && !isnan(dist2[i]) && dist2[i] < goodLineThresh){++i;}

    if(i >= n) return;

    //    printf("%%Bad: %d\n", i+1);

    //The one before was good
    lastGood = i-1;

    //Now find good again
    while(i < n && (isnan(dist2[i]) || dist2[i] > goodLineThresh)){ ++i; }

    //    printf("%%Good: %d\n",i+1);

    bool found = false;

    const Line *l1, *l2;

    l1 = &lines[lastGood]; l2 = &lines[i];

    if(i - lastGood < 2*nfit){
      printf("%%Corner suspect  -- %03d %03d\n", lastGood+1, i + 1);
// 	  printf("   %%Lines: (%g,%g,%g) (%g,%g,%g)\n"
// 		 ,l1->A(), l1->B(), l1->C()
// 		 ,l2->A(), l2->B(), l2->C());

      double dot;
      double x0,y0;
      double d1,d2;
      int i1,i2;
      i1 = lastGood + nfit - 1;
      i2 = i;

      d1 = l2->distance(x[i1],y[i1]);
      d2 = l1->distance(x[i2],y[i2]);

      dot = l1->A()*l2->A() + l1->B()*l2->B();

      if(fabs(dot) > COLLINEAR_THRESH){
	//Collinear lines
	printf("   %%Suspect colliniear (jump)\n");

	if(fabs(d1) < minJumpDist || fabs(d2) < minJumpDist){
	  printf("    %%Jump failed: dist %g,%g\n",d1,d2);
	}else{
	  int i_jump;

	  if(fabs(l1->A()*x[i2] + l1->B()*y[i2]) > fabs(l1->C()) ){
	    i_jump = i1;
	  }else{
	    i_jump = i2;
	  }

	  printf("    %%Jump @ %d (%g,%g)\n", i_jump+1, d1, d2);
	  found = true;
	}

      }else if(fabs(dot) < CONVEX_ANGLE_THRESH){
        //Intersecting lines

	if(fabs(d1) > maxConvexDist ||
	   fabs(d2) > maxConvexDist){

	  printf("   %%Convex failed: Lines are far apart (%g,%g)\n",d1,d2);
	}else{
	  l1->intersect(&x0,&y0, *l2);
	  found = true;

	  printf("   %%Convex or Concave @ (%+.3f,%+.3f)\n",x0,y0);
	}
      }

    }

    if(!found){
      int i_ray;
      double r;

      i_ray = lastGood + nfit;
      printf("%%Suspect Convex Hidden (after)  %03d\n",lastGood + nfit);

      if(i_ray < n_data){
	r = -l1->C()/(l1->A()*ca[i_ray] + l1->B()*sa[i_ray]);

	if(range[i_ray] - r > minRayPenetration){
	  printf("   %%Convex Hidden @ %d  (%d, %g,%g)\n"
                ,lastGood + nfit, i_ray + 1, r, range[i_ray]);
	}else{
	  printf("   %% failed. (%d, %g, %g)\n",i_ray+1, r, range[i_ray]);
	}
      }

      printf("%%Suspect Convex Hidden (before) %03d\n", i + 1);
      i_ray = i - 1;

      if(i_ray >= 0){
	r = -l2->C()/(l2->A()*ca[i_ray] + l2->B()*sa[i_ray]);

	if(range[i_ray] - r > minRayPenetration){
	  printf("   %%Convex Hidden @ %d  (%d, %g,%g)\n"
                , i + 1,i_ray+1, r, range[i_ray]);
	}else{
	  printf("   %% failed. (%d, %g, %g)\n",i_ray+1, r, range[i_ray]);
	}
      }
    }

  }while(1);
}

#include "LeastSquares2D.h"

extern "C"
double FindAdjLineLeft2D(double *x, double *y, unsigned int start,
		     unsigned int nData, 
		     int debug, LineFittingParamType *p,
		     Line2DCartDataType *l);


int fitLineTst(int argc, char** argv){
  if(argc < 2){
    printf("fitLineTst: scan_file [points_per_line]\n");
    return -1;
  }

  int pointsPerLine = 7;
  const int MAX_SCAN_LENGTH = 361;
  const double MAX_SCAN_RANGE = 8.18;

  int i;

  if(argc > 2) pointsPerLine = atoi(argv[2]);

  FILE *f = fopen(argv[1],"r");
  if(f == NULL){
    fprintf(stderr,"Failed to open file: %s\n",argv[1]);
    return -1;
  }


  //Load Data.
  bool readOk = true;
  double *range = new double[MAX_SCAN_LENGTH];
  const double DA = deg2rad(0.5);
  const double A0 = 0;
  double a;
  double *ca = new double[MAX_SCAN_LENGTH];
  double *sa = new double[MAX_SCAN_LENGTH];


  for(i = 0, a = A0; readOk && i < MAX_SCAN_LENGTH; ++i, a += DA){
    readOk = (fscanf(f,"%lf", range + i) == 1);
    ca[i] = cos(a); sa[i] = sin(a);
  }
  fclose(f);

  int n_points = i;
  double *x  = new double[n_points];
  double *y  = new double[n_points];

  for(i = 0; i < n_points; ++i){
    if(range[i] < MAX_SCAN_RANGE){
      x[i] = range[i]*ca[i];
      y[i] = range[i]*sa[i];
    }else{
      x[i] = NaN;
      y[i] = NaN;
    }
  }

  int nl = n_points - pointsPerLine + 1;
  Line *lines = new Line[nl];
  double *dist2 = new double[nl];

  fitLinesToScan(lines, dist2, x, y, n_points, pointsPerLine);

  for(i = 0; i < nl; ++i){
    printf("%+g %+g %+g %+g\n"
	   , lines[i].A()
	   , lines[i].B()
	   , lines[i].C()
	   , dist2[i]);
  }

  //Find regions of interest
//   lookForInterestingPoints(dist2,lines, nl, pointsPerLine
//                            ,range,ca,sa,x,y);
  LineFittingParamType *params_;
  params_ = (LineFittingParamType *) malloc(sizeof(LineFittingParamType));
  params_->minPtsForLine = 10;
  params_->minDistForLine = 0.4;
  params_->maxProbScaleLen = 3.0;
  params_->maxDistForLine = 500.0; 
  params_->lineLenProbScale = 0.2;
  params_->ptErrorThresh = 0.01;
  params_->errorLenScale = 3.5; /* 
				 * Roughly speaking, want a 0.5m line
				 * with one erroroneous point 0.1m off
				 * to have a probability of 0.5 
				 */
  params_->maxPtSpacing = 0.5;
  params_->ptSpacingThresh = 0.1;
  params_->ptSpacingScale = 2.0;

  Line2DCartDataType l;
  double *x_rev = new double[n_points];
  double *y_rev = new double[n_points];
  for(i = 0; i < n_points; ++i){
    x_rev[i] = x[n_points - 1 -i];
    y_rev[i] = y[n_points - 1 -i];
  }

  for(i = 0; i < n_points; ++i){

    double p = FindAdjLine2D(x_rev,y_rev,n_points - i - 1, 
                             n_points, 0, params_, &l);

    printf("%03d %03d %15g", i+1, i - l.nPoints + 2, p);

    p = FindAdjLine2D(x,y,i,0, 0, params_, &l);
    printf("\t%03d %03d %15g\n", i+1, i - l.nPoints + 2, p);
  }



  delete[] range; delete[] ca; delete[] sa;
  delete[] x; delete[] y;
  delete[] x_rev; delete[] y_rev;

  return 0;
}

#include "MappingAgent.h"

int GP_test(int argc, char **argv){
  ParticleHistory* gp = new ParticleHistory;

  gp->setMap(0,1);
  gp->setMap(1,2);
  gp->setMap(10,10);

  gp->setTransition(1,1);
  gp->setTransition(2,2);
  gp->setTransition(10,10);

  ParticleHistory *gp2 = gp->clone();

  gp->debugDump(stdout);
  gp2->debugDump(stdout);

  gp2->setMap(4,5);
  gp2->setTransition(11,12);

  gp->debugDump(stdout);
  gp2->debugDump(stdout);

  delete gp;

  gp2->getMap(10);
  gp2->getMap(1);

  gp2->debugDump(stdout);

  delete gp2;

  return 0;
}

#include "kdtree.h"

int KDTree_Test(int argc, char** argv){
  const int n = 20;
  double *x = new double[n];
  double *y = new double[n];
  int i;
  UniformRandomNumber dice(0,n, time(NULL));

  printf("x = ");
  for(i = 0; i < n; ++i){
    x[i] = dice.nextRandom();
    y[i] = dice.nextRandom();
    printf(" (%.3f,%.3f)", x[i],y[i]);
  }
  printf("\n");

  KDTree kd(2);
  double *xy[2] = {x,y};

  kd.init(xy,n);

  for(i = 0; i < n; ++i){
    double dist2;
    double q[2] = {xy[0][i], xy[1][i]};

    int nn = kd.findNN(q, &dist2);

    printf("NN: %d => %d (dist2 %g, %g)\n",i,nn,dist2, sqrt(dist2));
  }

  return 0;
}

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
      icp.match(*scan, *prevScan, 0.01, 0.999, step);

      double ca = cos(odo.rot);
      double sa = sin(odo.rot);

      odo.x   += (ca*step[0] - sa*step[1]);
      odo.y   += (sa*step[0] + ca*step[1]);
      odo.rot += step[2];

      printf("%+10.3f %+10.3f %+10.3f \t\t %+10.3f %+10.3f %+10.3f\n"
            , odo.x, odo.y, odo.rot
            , step[0], step[1], step[2]);
      fflush(stdout);

#if 0
      ca = cos(odo.rot);
      sa = sin(odo.rot);
      unsigned int i;
      for(i = 0; i < scan->n; ++i){
	double x,y;
	x = scan->x[i]*ca - scan->y[i]*sa;
	y = scan->x[i]*sa + scan->y[i]*ca;
	fprintf(fscans,"%.3e %.3e\n", x,y);
      }
#endif

#if 1
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

int main(int argc, char** argv){
  return ICP_Test(argc,argv);
}


//====================================
// Permutation test
//====================================

FILO M1(100);
FILO M2(100);
FILO *m1;
FIFO *m2;

void compute_state(){
  int i;

  for(i = 0; i < M2.nElements(); ++i){
    printf(" %d<>%d", M1[i],M2[i]);
  }
  printf("\n");
}

void permut(double weight, int lvl){

  if(m1->isEmpty() || m2->isEmpty() ) return;

  int e1 = m1->remove();
  M1.add(e1);

  int e2;


  int e2stop = m2->getTail();
  e2 = m2->remove();

  while(true){
    double dw = 0;

    //Assign e1 => e2
    M2.add(e2);

//     for(int j = 0; j < lvl; ++j) printf("......");
//     printf("%d->%d\n", e1, e2);

    //Compute dw
    compute_state();    

    //Go into recursion
    permut(weight + dw,lvl+1);

    //Release e2, if it was the last -- end the loop
    m2->add(e2);
    M2.remove();

    if(e2 == e2stop) break;

    //Get next e2
    e2 = m2->remove();
  }

  //Assign e1 to nothing
  M1.remove();
  e2 = -1;

//   for(int j = 0; j < lvl; ++j) printf("......");
//   printf("%d->%d\n", e1, e2);

  //Go into recursion
  permut(weight, lvl+1);

  //Put back e1, return
  m1->add(e1);
}
