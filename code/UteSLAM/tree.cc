#ifndef DEPEND
#include <math.h>
#endif

#include "kalman2.h"
#include "tree.h"
#include "AngleTools.h"
#include "util.h"
#include "matlab.h"
#include "localmap.h"
#include "mapRegion.h"
#include "image.h"
#include "polygon.h"
#include "TimeTools.h"

//---------------------------------------------------
// Variables
//---------------------------------------------------
static RangeAndBearingSensorRange  sensorRange(0,120,-120*DEGTORAD,
                                                     120*DEGTORAD);
static Landmark2dMapMatcher        matcher;
static Landmark2dLocGuts           locGuts;

const double MAX_CORE_NORM  = POW2(0.5/3);
static Point rangeBounds[4];

static bool findFloodSeed(int* ix, int* iy, 
                   const RegionGridMap *reg,
		   const BYTE* data,
		   const RobotPose &r);

//----------------------------------------------------
// Mapping Process
//----------------------------------------------------
TreeMappingProcess::TreeMappingProcess():robotMask(NULL),region0(NULL){
  sensorRange     = &::sensorRange;
  matcher         = &::matcher;
  localiserGuts   = &::locGuts;
  MAX_MD2 = 9;

  //Init range bounds
  double R = 90;
  double margin = 0.3*R;

  rangeBounds[0].set( -margin , -R - margin);
  rangeBounds[1].set(R + margin , -R - margin);
  rangeBounds[2].set(R + margin ,  R + margin);
  rangeBounds[3].set( -margin ,  R + margin);

  //Init Map region
  const double cellSz = 0.5;

  initDefaultMapRegion(cellSz);
  initRobotMask(cellSz);

}

void TreeMappingProcess::initDefaultMapRegion(double cellSz){
  const double R = 40;

  int n = int(R/cellSz);
  const double fwd = 0.7;

  region0 = new GridMap((fwd-1)*R, 2*R, 
                        -R, 2*R, n,n);
  region0->setAll(1.0);
}

double TreeMappingProcess::computeRegionScore(const MapAreaInterface* area, 
                                              const RobotPose* robot){

  double score = 0;
  double   range[] = {5 , 10 , 15 , 18,  20 , 22 , 25 , 30 , 40};
  double w_range[] = {0.007 ,0.106 ,0.171 ,0.158 ,0.151 ,0.137 
                     ,0.125 ,0.101 ,0.044};

  const int n_range = 9;
  const double n_angle = 7;

  double da[] = {0.00000, 0.5236, -0.5236, 1.0472, -1.0472, 1.5708, -1.5708};

  int i,j;

  for(i = 0; score < 1.0 && i < n_angle; ++i){
    double a = robot->rot + da[i];
    double ca = cos(a);
    double sa = sin(a);

    for(j = 0; score < 1.0 && j < n_range; ++j){
      double x,y,w;
      x = range[j]*ca + robot->x;  y = range[j]*sa + robot->y;
      w = area->probIsInside(x,y)*w_range[j]*0.33333333;

      score += w;
    }
  }

  if(score > 1) score = 1.0;

  return score;
}


RangeInterface* TreeMappingProcess::getNeighbourhoodRange(const RobotPose* r){
  double R = 100;
  double xmin, xmax, ymin, ymax;

  xmin = r->x - R;
  xmax = r->x + R;
  ymin = r->y - R;
  ymax = r->y + R;

  return new RangeMinMax(xmin,ymin,xmax,ymax);
}

MapEntryInterface * 
TreeMappingProcess::makeNewEntry(double *logProb,
                                  const ObservationStore & obs_store, 
                                  int obs_ind,
                                  const RobotPose* robot)const{

  const TreeObservation *obs = (const TreeObservation*) obs_store[obs_ind];
 
  Gaussian2d point(obs->toCartesian(robot));

  *logProb = logVolumeOfTheProduct(point,point);

  return new TreeLandmark(point, 
                          obs->diameter, obs->sigma_d2);

}

//Perform data association
//   corr_out[i] -- should be set to index of the map element from
//                  which observation i is originating,
//                  -1 indicates no data association.
//   obs,nObs    -- Array of observations and it's size
//   map,nMap    -- Array of the expected observations and it's size
// 
// Return value: log(Prob) of the data association
double TreeMappingProcess::
dataAssociate(int *corr_out
	      ,const ObservationInterface*const* obs, int nObs
	      ,const RobotPose *robot
              ,const MapInterface *map){

  double p_log = 0.0;
  int i,j;

  for(i = 0; i < nObs; ++i){
    const Landmark2dObs* o = (const Landmark2dObs*) obs[i];
    Gaussian2d g(o->toCartesian(robot));

    double w_max = -100000;
    int ind = -1;

    for(j = 0; j < map->numElements(); ++j){
      const Landmark2d *m = (const Landmark2d*) map->get(j);

      double A[2][2];
      double A_inv[2][2];
      double b[2] = {g.x - m->point.x, 
                     g.y - m->point.y};

      matrix2_add(A, m->point.cov, g.cov);
      matrix2_inverse(A_inv, A);

      double md2 = matrix2_multiply2(A_inv, b);

      if(md2 < MAX_MD2){
	double w = -(log(matrix2_det(A)) + md2);

	if(ind < 0 || w > w_max){
	  w_max = w;
	  ind = j;
	}
      }
    }

    if(ind >= 0){
      const double LOGVOL2_FIXED_PARAM = -log(2*M_PI);

      p_log += (0.5*w_max + LOGVOL2_FIXED_PARAM);
    }

    corr_out[i] = ind;
  }

  return p_log;
}


SimpleMap* TreeMappingProcess::loadLocalMap(const MatlabVariable& m){
  if(m.numCols() != 9) return NULL;

  SimpleMap *map = new SimpleMap(m.numRows());
  int i;

  for(i = 0; i < m.numRows(); ++i){
    double x,y, d, sd2, cov[2][2];
    int st;
    x         = m.get(i,0);
    y         = m.get(i,1);
    cov[0][0] = m.get(i,2);
    cov[0][1] = m.get(i,3);
    cov[1][0] = m.get(i,4);
    cov[1][1] = m.get(i,5);
    d         = m.get(i,6);
    sd2       = m.get(i,7);

    st = (int) m.get(i,8);

    map->set(i, new TreeLandmark(Gaussian2d(x,y,cov),d,sd2,st));
  }

  return map;
}

void TreeMappingProcess::getInitialMapArea(MapAreaInterface **mapArea, 
                                           RobotPose* robot){
//   printf("TREE::getInitialMapArea\n");

  robot->x = 0.0;
  robot->y = 0.0;
  robot->rot = 0.0;

  *mapArea = new RegionGridMap(region0->clone());
}

double TreeMappingProcess::subtractFromRegion(
                          MapAreaInterface* a,
			  const MapAreaInterface *b){

        RegionGridMap* reg1 = (RegionGridMap*)a;
  const RegionGridMap* reg2 = (const RegionGridMap*)b;

  int npix = reg1->subtract(reg2);

  double area = npix*reg1->getGrid()->cellX()*reg1->getGrid()->cellY();

  return area;
}

double TreeMappingProcess::subtractFromRegion(MapAreaInterface* a,
					      const MapAreaInterface **b,
					      const RobotPose* odo
                                              ,int n){
  int i;
        RegionGridMap* reg1 = (RegionGridMap*)a;
  const RegionGridMap* reg2;
  int npix = 0;

  for(i = 0; i < n ; ++i){
    reg2 = (const RegionGridMap*) b[i];
    npix += reg1->subtract(reg2, odo[i]);
  }

  double area = npix*reg1->getGrid()->cellX()*reg1->getGrid()->cellY();

  return area;
}

void TreeMappingProcess::finaliseMapArea(MapAreaInterface* a, 
                                const RobotPose& r){
  RegionGridMap *reg = (RegionGridMap*) a;
  const GridMap* grid = reg->getGrid();

  BYTE *g1 = grid->getData();
  BYTE *g2 = new BYTE[grid->size()];

  int ix,iy;
  int ncols, nrows;

  ncols = grid->numX();
  nrows = grid->numY();

  //First run the robot mask
  image_mask(g2, g1, nrows, ncols
            ,robotMask, robotMask_h, robotMask_w);

  //Convert sensor pose to pixel value
  RobotPose sensor;
  robot2sensor(&sensor, &r);

  if(!reg->getPixel(&ix, &iy, sensor.x, sensor.y)){
    printf("Warning Initial point: %.2f %.2f %.2f(deg) -- outside the grid.\n",
           r.x, r.y, r.rot*RADTODEG);
    return;
  }

  if(!findFloodSeed(&ix,&iy, reg, g2, sensor)){
    //We don't have the seed point for the flood fill.
    printf("No seed for flooding (%g,%g)->(%d,%d) -- empty map\n",
            r.x,r.y,ix,iy);
    reg->set(new GridMap());
    return;
  }

  memset(g1, 0, sizeof(BYTE)*grid->size());
  image_getNonZeroBlob(g1,g2,nrows,ncols,iy,ix);

  delete[] g2;

  reg->shrink();
}

void TreeMappingProcess::finaliseMapArea(MapAreaInterface* a){
  RegionGridMap *reg = (RegionGridMap*) a;
  const GridMap* grid = reg->getGrid();

  BYTE *g = grid->getData();

  int ncols, nrows;

  ncols = grid->numX();
  nrows = grid->numY();

  //First run the robot mask
  image_mask(g, g, nrows, ncols
            ,robotMask, robotMask_h, robotMask_w);

  //Then shrink it
  reg->shrink();
}


void TreeMappingProcess::finaliseMapArea(MapAreaInterface* area
                                       ,const SimpleMap* map){
  int np = map->numMap();
  double x,y;
  int i;

  printf("FinaliseMapArea: map(%d)\n",np);
  double T0 = GetCurrentTime();

  if(np < 3) return;

  VECT2 *points = new VECT2[np+1];
  Polygon poly;

  for(i = 0; i < np; ++i){
    map->get(i)->getCenter(&x,&y);
    points[i][0] = x;
    points[i][1] = y;
  }

  points[np][0] = points[np][1] = 0;

  printf("Computing convex hull....");
  findConvexHull(&poly,points,np+1);
  printf("  %d points\n",poly.numPoints());

  poly.expand(1.3);


  printf("Computing area overlap...");


  RegionGridMap *reg = (RegionGridMap*) area;
  const GridMap* grid = reg->getGrid();

  BYTE *g = grid->getData();
  for(i = 0; i < grid->size(); ++i){
    if(g[i] != 0){
      grid->ind2point(&x,&y,i);
      if(!poly.isInside(x,y)){
	g[i] = 0;
      }
    }
  }

  //Then shrink it
  reg->shrink();
  printf("done (%d ms)\n",(int)((GetCurrentTime()-T0)*1000));


  delete[] points;
}


bool TreeMappingProcess::isNeighbour(int nHops, 
                                           const RobotPoseCov* mapRef,
					   const MapAreaInterface* region){
  const double MAX_NEIGHBOUR_NORM  = POW2(2.0/2);
  const double MAX_NEIGHBOUR_DIST2 = POW2(180);

  if(nHops == 1) return true;

  double norm = mapRef->norm();
  if(norm > MAX_NEIGHBOUR_NORM) return false;

  double dist2 = POW2(mapRef->x) + POW2(mapRef->y);
  if(dist2 > MAX_NEIGHBOUR_DIST2) return false;

  return true;
}

void TreeMappingProcess::getSuitableCoreLandmarks(int *core, 
                                    const SimpleMap* m){
  int i;

  for(i = 0; i < m->numElements(); ++i){
    if(core[i] != 0){
      const TreeLandmark *e = (const TreeLandmark*)m->get(i);

      if(e->norm() > MAX_CORE_NORM){
	printf("  Removed: %d -- norm %e\n",i,e->norm());
	core[i] = 0;
      }
    }
  }
}

void TreeMappingProcess::initRobotMask(double cell_sz){
  const double ROBOT_DIAMETER = 1.0;

  robotMask_w = int(ROBOT_DIAMETER/cell_sz);

  if(robotMask_w % 2 == 0) robotMask_w += 1;
  robotMask_h = robotMask_w;

  double half_w = robotMask_w/2 + 1;
  double half_h = robotMask_w/2 + 1;


  double r2 = 0.25*robotMask_w*robotMask_w;

  robotMask = new BYTE[robotMask_w*robotMask_h];

  int c,r,i;

  for(r = 0; r < robotMask_h; ++r){
    for(c = 0; c < robotMask_w; ++c){
      double dr = r - half_h;
      double dw = c - half_w;
      double d2 = dr*dr + dw*dw;

      i = r*robotMask_w + c;
      if(d2 > r2) robotMask[i] = 0;
      else        robotMask[i] = 1;
    }
  }
}

extern void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, double *weight, int nsamples,int);


void TreeMappingProcess::sampleTransition(const SimpleMap* m1 
					  ,const SimpleMap* m2
					  ,const RobotPoseCov *m2in1
					  ,RobotPose *samples
					  ,double *weight
					  ,int nsamples){

  ::sampleTransition(m1,m2,m2in1,samples, weight, nsamples,0);
}



//----------------------------------------------------
// Tree Observation
//----------------------------------------------------

double TreeObservation::logLikelihood(const ObservationInterface* o)const{
  //  const TreeObservation* tree = (const TreeObservation*) o;

  double res = Landmark2dObs::logLikelihood(o);

  //  res += md2(diameter, sigma_d2 + tree->sigma_d2, tree->diameter);

  return res;
}

//------------------------------------------------------
// Tree Landmark
//------------------------------------------------------
void TreeLandmark::update(const ObservationStore &obs_store,
			    int obs_ind, 
			    const RobotPose *robot_pose){

  Landmark2d::update(obs_store, obs_ind, robot_pose);

  const TreeObservation* o = (const TreeObservation*) obs_store[obs_ind];
  update(o->diameter, o->sigma_d2);
}

void TreeLandmark::update(double d, double sd2){
  double K = sigma_d2/(sigma_d2 + sd2);
  diameter = diameter + K*(d - diameter);
  sigma_d2 = (1 - K)*sigma_d2;
}

extern void toPolar(Gaussian2d *polar_out, const Gaussian2d *point
		    , const RobotPose *robot);

double TreeLandmark::probMatch(const MapEntryInterface* m)const{
  const TreeLandmark* other = (const TreeLandmark*) m;

  return Landmark2d::probMatch(m)*
         probMatchD(other->diameter, other->sigma_d2);
}

double TreeLandmark::probMatchLog(const MapEntryInterface* m)const{
  const TreeLandmark* other = (const TreeLandmark*) m;

  return Landmark2d::probMatchLog(m) + 
         probMatchDLog(other->diameter, other->sigma_d2);
}

double TreeLandmark::mahalanobis2(const MapEntryInterface* m)const{
  const TreeLandmark* other = (const TreeLandmark*) m;

  return point.mahalanobis2(other->point);
  //*mahalanobisD2(other->diameter, other->sigma_d2);

}

double TreeLandmark::probMatch(const ObservationInterface* obs,    
                             const RobotPose* robot)const{

  const TreeObservation *o = (const TreeObservation*) obs;
  Gaussian2d g(o->toCartesian(robot));
  double p = g.probMatch(point)*probMatchD(o->diameter,o->sigma_d2);

  return p;
}

double TreeLandmark::probMatchLog(const ObservationInterface* obs, 
                                const RobotPose* robot)const{
  const TreeObservation *o = (const TreeObservation*) obs;
  Gaussian2d g(o->toCartesian(robot));
  double p_log = g.probMatchLog(point) + 
                 probMatchDLog(o->diameter, o->sigma_d2);

  return p_log;
}

double TreeLandmark::mahalanobis2(const ObservationInterface* obs, 
                                const RobotPose* robot)const{
  const TreeObservation *o = (const TreeObservation*) obs;
  Gaussian2d g(o->toCartesian(robot));
  double MD2 = g.mahalanobis2(point) + 
               md2(diameter,sigma_d2+o->sigma_d2,o->diameter);
  return MD2;
}


double TreeLandmark::probMatchD(double d, double sd)const{
  double sd2 = sd + sigma_d2;
  double dx =  diameter - d;

  double p = 1.0/sqrt(2*M_PI*sd2)*exp(-0.5*dx*dx/sd2);

  return p;
}

double TreeLandmark::probMatchDLog(double d, double sd)const{
  double sd2 = sd + sigma_d2;
  double dx =  diameter - d;

  double p_log = -0.5*(dx*dx/sd2 + log(sd2) + log(2*M_PI));

  return p_log;
}

bool TreeLandmark::matlabDump(FILE* f)const{

  return  fprintf(f,"%e %e %e %e %e %e %e %e %d"
		  ,point.x
		  ,point.y
		  ,point.cov[0][0]
		  ,point.cov[0][1]
		  ,point.cov[1][0]
		  ,point.cov[1][1]
		  ,diameter
		  ,sigma_d2
                  ,state) > 0;
}

//---------------------------------------------------------------
// Support functions
//---------------------------------------------------------------

bool Tree_storeLocalMap(FILE *f, const LocalMap *lmap){
  bool res;
  int i;

  const SimpleMap *map = lmap->getMap();

  res = fprintf(f,"map = [ ...\n") > 0;

  for(i = 0 ; res && i < map->numMap() && res; ++i){ //For every map element
    res = map->get(i)->matlabDump(f);
    res &= fprintf(f,"\n");
  }

  res &= fprintf(f,"]; %%end map\n\n") > 0;

  const RegionGridMap *reg = (const RegionGridMap*) lmap->getRegion();

  res = reg->matlabDump(f,"region");

  return res;
}

static bool findFloodSeed(int* ix, int* iy, 
                   const RegionGridMap *reg,
		   const BYTE* data,
		   const RobotPose &r){

  double range[] = {5.0,7.5,10,12.5,15};
  double angle[] = {0.000, 0.524, -0.524, 1.047, -1.047, 1.571, -1.571};
  const int nr = 5;
  const int na = 7;
  double x,y;
  int ind;

  if(reg->getPixel(ix,iy, 0, 0)){
    ind = reg->getGrid()->ind(*ix,*iy);
    if(data[ind] != 0) return true;
  }

  int i,j;

  for(i = 0; i < na; ++i){
    double ca = cos(angle[i] + r.rot);
    double sa = sin(angle[i] + r.rot);

    for(j = 0; j < nr; ++j){
      x = r.x + ca*range[j];
      y = r.y + sa*range[j];

      if(reg->getPixel(ix,iy, x, y)){
	ind = reg->getGrid()->ind(*ix,*iy);
	if(data[ind] != 0) return true;
      }

    }
  }

  return false;
}
