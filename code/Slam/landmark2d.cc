#include "landmark2d.h"
#include "kalman2.h"
#include "matlab.h"
#include "util.h"
#include "mapRegion.h"
#include "localmap.h"
#include "image.h"

//extern double LAS_OFFSET;
static RangeAndBearingSensorRange  sensorRange;
static Landmark2dMapMatcher        matcher;
static Landmark2dLocGuts           locGuts;

const double MAX_CORE_NORM  = POW2(0.05/3);
const int MIN_NUM_OBS = 5;
const double MAX_NEIGHBOUR_NORM  = POW2(0.2/3);
const double MAX_NEIGHBOUR_DIST2 = POW2(20);


//----------------------------------------------------
// Landmark2dObservation
//----------------------------------------------------

double Landmark2dObs::logLikelihood(const ObservationInterface* o)const{
  const Landmark2dObs* obs = (const Landmark2dObs*) o;

  double COV[2][2];
  double x[2];
  double res = 0.0;

  matrix2_add(COV, cov, obs->cov);

  x[0] = range - obs->range;
  x[1] = angleDiffRad(bearing, obs->bearing);

  res = mahalanobis2(x,COV);

  return res;
}

Gaussian2d Landmark2dObs::toCartesian()const{
  Gaussian2d x;
  double ca = cos(bearing);
  double sa = sin(bearing);

  x.x = range*ca;
  x.y = range*sa;

  double ja[2][2] = {{ca, -x.y},
		     {sa,  x.x}};
  double ja_t[2][2];

  //x.cov = ja*cov*ja'
  matrix2_transpose(ja_t,ja);
  matrix2_multiply(x.cov, ja, cov);
  matrix2_multiply(x.cov, x.cov, ja_t);

  return x;
}

Gaussian2d Landmark2dObs::toCartesian(const RobotPose *robot)const{
  Gaussian2d x;
  double ca = cos(bearing + robot->rot);
  double sa = sin(bearing + robot->rot);

  x.x = range*ca;
  x.y = range*sa;

  double ja[2][2] = {{ca, -x.y},
		     {sa,  x.x}};
  double ja_t[2][2];

  //x.cov = ja*cov*ja'
  matrix2_transpose(ja_t,ja);
  matrix2_multiply(x.cov, ja, cov);
  matrix2_multiply(x.cov, x.cov, ja_t);

  x.x += robot->x;
  x.y += robot->y;

  return x;
}


//============================================================================
//  Landmark2d 
//============================================================================

void toPolar(Gaussian2d *polar_out, const Gaussian2d *point
           , const RobotPose *robot){

  double dx,dy;
  double range, bearing;
  double cov[2][2];

  dx = point->x - robot->x;
  dy = point->y - robot->y;

  double r2 = dx*dx + dy*dy;

  range = sqrt(r2);
  bearing = angleDiffRad(atan2(dy,dx), robot->rot);

  double jc[2][2] = {{dx/range, dy/range},
                     {-dy/r2,   dx/r2}};
  double jc_t[2][2];
  double tmp[2][2];

  matrix2_transpose(jc_t,jc);
  matrix2_multiply(tmp, jc, point->cov);
  matrix2_multiply(cov, tmp, jc_t);

  polar_out->set(range,bearing,cov);
}

void Landmark2d::update(const ObservationStore &obs_store,
                        int obs_ind, 
		        const RobotPose *robot){
  const Landmark2dObs * obs0 = (const Landmark2dObs*) obs_store[obs_ind];

  ekf_update(&point, obs0->range, obs0->bearing, obs0->cov, robot);

  numObs_ += 1;
  lastScanId = obs_store.getScanId(obs_ind);

  if(isNew() && numObs_ > MIN_NUM_OBS){
    setMature();
  }
}

double Landmark2d::probMatch(const MapEntryInterface* m)const{
  const Landmark2d *other = (const Landmark2d*) m;
  return point.probMatch(other->point);
}

double Landmark2d::probMatchLog(const MapEntryInterface* m)const{
  const Landmark2d *other = (const Landmark2d*) m;
  return point.probMatchLog(other->point);
}

double Landmark2d::mahalanobis2(const MapEntryInterface* m)const{
  const Landmark2d *other = (const Landmark2d*) m;
  return point.mahalanobis2(other->point);
}

double Landmark2d::probMatch(const ObservationInterface* obs,    
                             const RobotPose* robot)const{

  const Landmark2dObs *o = (const Landmark2dObs*) obs;
  Gaussian2d g(o->toCartesian(robot));
  return g.probMatch(point);
}

double Landmark2d::probMatchLog(const ObservationInterface* obs, 
                                const RobotPose* robot)const{
  const Landmark2dObs *o = (const Landmark2dObs*) obs;
  Gaussian2d g(o->toCartesian(robot));
  return g.probMatchLog(point);
}

double Landmark2d::mahalanobis2(const ObservationInterface* obs, 
                                const RobotPose* robot)const{
  const Landmark2dObs *o = (const Landmark2dObs*) obs;
  Gaussian2d g(o->toCartesian(robot));
  return g.mahalanobis2(point);
}



bool Landmark2d::matlabDump(FILE* f)const{
  bool res = fprintf(f,"%16.9e %16.9e %16.9e %16.9e %16.9e %16.9e %d"
             ,point.x,point.y
             ,point.cov[0][0]
             ,point.cov[0][1]
             ,point.cov[1][0]
             ,point.cov[1][1], state) > 0;

  return res;
}

//---------------------------------------------------------------
// RangeAndBearingSensorRange
//---------------------------------------------------------------

bool RangeAndBearingSensorRange::isWithinRange(const ObservationInterface* o)
const{
  const Landmark2dObs* lndm = (const Landmark2dObs* ) o;

  double r = lndm->range - minRange;
  if(r > dRange) return false;

  double a = angleDiffRad(lndm->bearing, minAngle);
  if(a < 0) a = M_PI - a;

  return a < dAngle;
}

bool RangeAndBearingSensorRange::isWithinRange(double x, double  y)const{
  double r2 = x*x + y*y;

  if(r2 > maxRange2) return false;
  if(r2 < minRange2) return false;

  double a = angleDiffRad(atan2(y,x), minAngle);
  if(a < 0) a = M_PI - a;

  return a < dAngle;
}

bool RangeAndBearingSensorRange::isWithinRange(double x, double  y, double a0)const{
  double r2 = x*x + y*y;

  if(r2 > maxRange2) return false;
  if(r2 < minRange2) return false;

  double a = angleDiffRad(atan2(y,x), a0);
  a = angleDiffRad(a,minAngle);

  if(a < 0) a = M_PI - a;

  return a < dAngle;
}



//============================================================================
// MappingProcess
//============================================================================


Landmark2dMappingProcess::Landmark2dMappingProcess(){
  sensorRange     = &::sensorRange;
  matcher         = &::matcher;
  localiserGuts   = &::locGuts;
  MAX_MD2         = 16;
  useNegativeInfo = true;

}


RangeInterface* Landmark2dMappingProcess::getNeighbourhoodRange(const RobotPose* r){
  double R = 10;
  double xmin, xmax, ymin, ymax;

  xmin = r->x - R;
  xmax = r->x + R;
  ymin = r->y - R;
  ymax = r->y + R;

  return new RangeMinMax(xmin,ymin,xmax,ymax);
}

const double LOGVOL2_FIXED_PARAM = -log(2*M_PI);
const double VOL2_FIXED_PARAM    = 1.0/(2*M_PI);

RangeAndBearingSensorRange negInfoRange(0,7,-M_PI/2,M_PI/2);

double logProbNoMatch(int d2){
  //SPAM: assumes cell size is 5cm
  const
  int X2[] = { 36,100,196,324,484,676,900,1156,1444,1764,2116,2500,2916
              ,3364,3844,4356,4900,5476,6084,6724,7396,8100,8836,9604,10404
              ,11236,12100,12996,13924,14884,15876,16900,17956,19044,20164
              ,21316,22500,23716,24964,26244};

  //Y(i) = log(Prob. Not observing a landmark at X(i))
  const
  double Y[] = {        0.000, -0.007, -0.047, -0.152, -0.354, -0.683, 
               -0.993, -1.174, -1.272, -1.125, -0.974, -0.843, -0.726, 
               -0.626, -0.504, -0.367, -0.279, -0.270, -0.202, -0.160, 
               -0.152, -0.150, -0.132, -0.123, -0.121, -0.107, -0.109, 
               -0.095, -0.083, -0.074, -0.062, -0.043, -0.039, -0.035, 
               -0.010, -0.007, -0.007, -0.006, -0.002,  0.000};
  const int ndata = 40;

  if(d2 < X2[0] || d2 > X2[ndata-1]) return Y[0];

  int i = binarySearch(d2, X2, ndata);

  //  printf("BSearch: %d -> %d\n",d2,i);

  return Y[i];
}

double logProbNoMatch(double r2){
  //Perform table lookup (with linear interpolation)
  const
  double X[] = {     0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9, 
                2.1, 2.3, 2.5, 2.7, 2.9, 3.1, 3.3, 3.5, 3.7, 3.9, 
                4.1, 4.3, 4.5, 4.7, 4.9, 5.1, 5.3, 5.5, 5.7, 5.9, 
                6.1, 6.3, 6.5, 6.7, 6.9, 7.1, 7.3, 7.5, 7.7, 7.9, 8.1};
  const
  double Y[] = {        0.000, -0.007, -0.047, -0.152, -0.354, -0.683, 
               -0.993, -1.174, -1.272, -1.125, -0.974, -0.843, -0.726, 
               -0.626, -0.504, -0.367, -0.279, -0.270, -0.202, -0.160, 
               -0.152, -0.150, -0.132, -0.123, -0.121, -0.107, -0.109, 
               -0.095, -0.083, -0.074, -0.062, -0.043, -0.039, -0.035, 
               -0.010, -0.007, -0.007, -0.006, -0.002,  0.000};

  //Y(i) = log(Prob. Not observing a landmark at X(i))

  const int ndata = 40;
  const double step = 0.2;
  double r = sqrt(r2);

  double w_log;


  int ix = (int)((r - X[0])/step);
  if( ix < 0 || ix >= ( ndata -1 ) ) return Y[ndata-1];

  double dx = r - X[ix];

  w_log = Y[ix] + dx*(Y[ix+1] - Y[ix])/step;

  return w_log;
}

double logProbNoMatch(double x, double y, double a0){
  double maxRange2 = 8*8;         //Range < 8m
  double minAngle = deg2rad(-85); //from -85 to +85
  double dAngle   = deg2rad(170);

  double r2 = x*x + y*y;

  if(r2 > maxRange2) return 0.0; //outside the range (distance)

  double a = angleDiffRad(atan2(y,x), a0);
  a = angleDiffRad(a,minAngle);

  if(a < 0) a = M_PI - a;

  if(a > dAngle) return 0.0;   //outside range (angle)

  return logProbNoMatch(r2);
}

double 
Landmark2dMappingProcess::computeNegativeInfo(const int *corr
                                              ,const ObservationInterface*const* obs
                                              ,int nObs
					      ,const RobotPose *robot
					      ,const MapInterface *map){
  double w_log = 0;
  int i;

  for(i = 0; i < map->numElements(); ++i){
    if(corr[i] < 0){
      double x,y;
      map->get(i)->getCenter(&x,&y);
      x -= robot->x;
      y -= robot->y;

      w_log += logProbNoMatch(x,y,robot->rot);

    }
  }

  return w_log;
}



//Perform data association
//   corr_out[i] -- should be set to index of the map element from
//                  which observation i is originating,
//                  -1 indicates no data association.
//   obs,nObs    -- Array of observations and it's size
//   map,nMap    -- Array of the expected observations and it's size
// 
// Return value: log(Prob) of the data association
double Landmark2dMappingProcess::
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

//     double d_max = 10*sqrt(max(g.cov[0][0], g.cov[1][1]) 
//                             + fabs(g.cov[0][1]));

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
      p_log += (0.5*w_max + LOGVOL2_FIXED_PARAM);
    }

    corr_out[i] = ind;
  }
  
  if(useNegativeInfo) 
    p_log += computeNegativeInfo(corr_out,obs,nObs,robot,map);

  return p_log;
}

//   Compute Map Area and pose of the robot within it
void Landmark2dMappingProcess::getInitialMapArea(
          MapAreaInterface **mapArea, RobotPose* robot){
  //TODO:
  *mapArea = NULL;
  robot->set(0,0,0);
}

double Landmark2dMappingProcess::subtractFromRegion(
                          MapAreaInterface* a,
			  const MapAreaInterface *b){

        RegionGridMap* reg1 = (RegionGridMap*)a;
  const RegionGridMap* reg2 = (const RegionGridMap*)b;

  int npix = reg1->subtract(reg2);

  double area = npix*reg1->getGrid()->cellX()*reg1->getGrid()->cellY();

  return area;
}

double Landmark2dMappingProcess::subtractFromRegion(MapAreaInterface* a,
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

bool findFloodSeed(int* ix, int* iy, 
                   const RegionGridMap *reg,
		   const BYTE* data,
		   const RobotPose &r){

  double range[] = {0.5,0.75,1.0,1.25,1.5};
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

void Landmark2dMappingProcess::finaliseMapArea(MapAreaInterface* a, 
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

  if(!findFloodSeed(&ix,&iy, reg, g2, sensor)){
    //We don't have the seed point for the flood fill.
    printf("No seed for flooding -- empty map\n");
    reg->set(new GridMap());

    delete[] g2;

    return;
  }

  memset(g1, 0, sizeof(BYTE)*grid->size());
  image_getNonZeroBlob(g1,g2,nrows,ncols,iy,ix);

  delete[] g2;

  reg->shrink();
}

void Landmark2dMappingProcess::finaliseMapArea(MapAreaInterface* a){
  RegionGridMap *reg = (RegionGridMap*) a;
  const GridMap* grid = reg->getGrid();

  BYTE *g = grid->getData();

  //First run the robot mask
  image_mask(g, g, grid->numY(), grid->numX()
            ,robotMask, robotMask_h, robotMask_w);

  //Then shrink it
  reg->shrink();
}


void Landmark2dMappingProcess::finaliseMapArea(MapAreaInterface* a,
                                               const SimpleMap *m){
  int n_landmarks = m->numElements();

  int *ix = new int[n_landmarks];
  int *iy = new int[n_landmarks];
  RegionGridMap * reg = (RegionGridMap*) a;
  int i,j;

//   printf("finaliseMapArea: %d \n",n_landmarks);

  //Convert map in to pixel coordinates
  for(i = 0; i < n_landmarks; ++i){
    double X, Y;
    m->get(i)->getCenter(&X,&Y);

    reg->getPixel(ix+i,iy+i,X,Y);

//     printf("Landmark: %d %d\n",ix[i],iy[i]);
  }

  //Compute prob of observing any landmark in pixel coordinates
  const GridMap* grid = reg->getGrid();
  BYTE *g = grid->getData();
  int x,y;
  double logW;
  const double LOG_PROB_NOMATCH = -1.2; // ~= 30%

  x = 0; y = 0;

  for(i = 0; i < grid->size(); ++i){
    if(g[i] > 0){ //If occupied.
      grid->ind2point(&x,&y, i); //Convert to image coords.

//       printf("%d -> %d,%d\n",i,x,y);

      //For every landmark
      logW = 0;
      for(j = 0; j < n_landmarks && logW > LOG_PROB_NOMATCH; ++j){
	int dx,dy;
	dx = x - ix[j];
	dy = y - iy[j];

	int d2 = dx*dx + dy*dy;

	if(d2 < 400){ //Anything within 20 pixel radius to a landmark
	              // automaticaly quialifies
	  logW = LOG_PROB_NOMATCH - 100;
	}else{
	  logW += logProbNoMatch(d2);
	}
      }

      if(logW > LOG_PROB_NOMATCH){ //Likely that sees nothing from here
	//	printf("Removing: %d,%d %g\n",x,y,logW);
 	g[i] = 0;
      }
    }
  }

  reg->shrink();

  //clean up;
  delete[] ix; delete[] iy;
}

double Landmark2dMappingProcess::
computeRegionScore(const MapAreaInterface* area, 
		   const RobotPose* robot){
  double score = 0;
  double   range[] = {0.5 , 1.0 , 1.5 , 1.8,  2.0  , 2.2 , 2.5 , 3.0 , 4.0};
  double w_range[] = //{0.08, 0.13, 0.15, 0.15, 0.14, 0.13, 0.11, 0.08, 0.03};
             {0.007 ,0.106 ,0.171 ,0.158 ,0.151 ,0.137 ,0.125 ,0.101 ,0.044};

  const int n_range = 9;
  double da[] = {0.00000, 0.5236, -0.5236, 1.0472, -1.0472, 1.5708, -1.5708};
  const double n_angle = 7;

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


//For Local Map
SimpleMap* Landmark2dMappingProcess::loadLocalMap(const MatlabVariable& m){
  if(m.numCols() != 7) return NULL;

  SimpleMap *map = new SimpleMap(m.numRows());
  int i;

  for(i = 0; i < m.numRows(); ++i){
    double x,y, cov[2][2];
    int st;
    x = m.get(i,0);
    y = m.get(i,1);
    cov[0][0] = m.get(i,2);
    cov[0][1] = m.get(i,3);
    cov[1][0] = m.get(i,4);
    cov[1][1] = m.get(i,5);

    st = (int) m.get(i,6);

    map->set(i, new Landmark2d(Gaussian2d(x,y,cov),st));
  }

  return map;
}


//For Global Map
bool Landmark2dMappingProcess::isNeighbour(int nHops, 
                                           const RobotPoseCov* mapRef,
					   const MapAreaInterface* region){

  if(nHops == 1) return true;

  double dist2 = POW2(mapRef->x) + POW2(mapRef->y);
  if(dist2 > MAX_NEIGHBOUR_DIST2) return false;

  double norm = mapRef->norm();

  //  printf("NORM: %g\n", norm);

  if(norm > MAX_NEIGHBOUR_NORM) return false;


  return true;
}

void Landmark2dMappingProcess::getSuitableCoreLandmarks(int *core, 
                                    const SimpleMap* m){
  int i;

  for(i = 0; i < m->numElements(); ++i){
    if(core[i] != 0){
      const Landmark2d *e = (const Landmark2d*)m->get(i);

      if(e->norm() > MAX_CORE_NORM){
	printf("  Removed: %d -- norm %e\n",i,e->norm());
	core[i] = 0;
      }
    }
  }
}


MapEntryInterface * 
Landmark2dMappingProcess::makeNewEntry(double *logProb,
				const ObservationStore &obs_store, 
				int ind,
				const RobotPose* robot)const{

  const Landmark2dObs *obs = (const Landmark2dObs *)obs_store[ind];
  Gaussian2d point(obs->toCartesian(robot));

  *logProb = logVolumeOfTheProduct(point,point) - 4;

  if(isnan(*logProb)){
    printf("makeNewEntry::isNaN\n");
    print_robot("robot", robot);
    print_obs("point", point);
    printf("obs = [");
    obs->matlabDump(stdout);
    printf("];\n");
  }

  return new Landmark2d(point,0);
}

void Landmark2dMappingProcess::initRobotMask(double robotD, double cell_sz){
  robotMask_w = int(robotD/cell_sz);

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

//---------------------------------------------------------------
// Support functions
//---------------------------------------------------------------
bool Landmark2d_storeLocalMap(FILE *f, const LocalMap *lmap){
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



void computeMidPoint(double *x, double *y, double *a,
		     const Point p[2]){
  double dy = p[0].y - p[1].y;
  double dx = p[0].x - p[1].x;
  *a = atan2(dy,dx);
  *x = 0.5*(p[0].x + p[1].x);
  *y = 0.5*(p[0].y + p[1].y);
}

void computeTransition(RobotPose* r, const Point p1[2],
		       const Point p2[2]){
  double x1,y1,a1;
  double x2,y2,a2;

  computeMidPoint(&x1,&y1,&a1, p1);
  computeMidPoint(&x2,&y2,&a2, p2);
  double a = angleDiffRad(a1,a2);
  double ca,sa;
  ca = cos(a); sa = sin(a);

  r->x   = x1 - x2*ca + y2*sa;
  r->y   = y1 - x2*sa - y2*ca;
  r->rot = a;
}

inline double computeWeight(const RobotPose* r2in1,
                            const SimpleMap *m1, const SimpleMap *m2){
  SimpleMap m2_(*m2);
  m2_.translateMe(*r2in1);
  double w_log = m1->probMatchLog(m2_);
  return exp(w_log);
}

extern void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, double *weight, 
                      int nsamples, int nTrials);


void Landmark2dMappingProcess::sampleTransition(const SimpleMap* m1 
						,const SimpleMap* m2
						,const RobotPoseCov *m2in1
						,RobotPose *samples_out
						,double *w_out
						,int nsamples){
  RobotPoseSampler sampler(*m2in1);
  RobotPoseCov est(*m2in1);

  int i;
  double sumW;
  int trial = 0;

  int n2 = nsamples*2;

  int *ind = new int[nsamples];
  double *w = new double[n2];
  RobotPose* smpl = new RobotPose[n2];

  const int Ninit = min(300,n2);
  const int nTrials = 2;


  while(trial < nTrials){
    trial += 1;
    sumW = 0.0;

    for(i = 0; i < Ninit; ++i){
      sampler.sample(smpl + i);
      w[i] = computeWeight(smpl+i, m1,m2);
      sumW += w[i];
    }

    if(sumW <= 0.0){
      printf("Failed to sample transition. (SumW <= 0.0)\n");
      trial = nTrials + 1;
    }else{
      //Normalise the weights
      register double sumW_inverse = 1.0/sumW;
      for(i = 0; i < Ninit; ++i) w[i] *= sumW_inverse;

      //Compute covariance
      est.cov[0][0] = 0.00;  est.cov[0][1] = 0.00;  est.cov[0][2] = 0.00;
      est.cov[1][0] = 0.00;  est.cov[1][1] = 0.00;  est.cov[1][2] = 0.00;
      est.cov[2][0] = 0.00;  est.cov[2][1] = 0.00;  est.cov[2][2] = 0.00;

      for(i = 0; i < Ninit; ++i){
	register double dx,dy,da;
	register double w_ = w[i];

	dx = smpl[i].x   - m2in1->x;
	dy = smpl[i].y   - m2in1->y;
	da = angleDiffRad(smpl[i].rot,m2in1->rot);

	est.cov[0][0] += dx*dx*w_;
	est.cov[0][1] += dx*dy*w_;
	est.cov[0][2] += dx*da*w_;
	est.cov[1][1] += dy*dy*w_;
	est.cov[1][2] += dy*da*w_;
	est.cov[2][2] += da*da*w_;
      }

      est.cov[1][0] = est.cov[0][1];
      est.cov[2][0] = est.cov[0][2];
      est.cov[2][1] = est.cov[1][2];

      sampler.set(est);
    }
  }

  for(i = 0; i < n2; ++i){
    sampler.sample(smpl + i);
    w[i]  = computeWeight(smpl + i, m1,m2);
  }
  normalise(w,w,nsamples);

  sample(ind,w,n2, nsamples);

  double w0 = 1.0/nsamples;

  for(i = 0; i < nsamples; ++i){
    w_out[i] = w0;
    samples_out[i] = smpl[ind[i]];
  }

}


//============================================================================
// Landmark2dMapMatcher
//============================================================================
void Landmark2dMapMatcher::match(const SimpleMap &m1, const SimpleMap &m2){
  Gaussian2d map1[m1.numElements()];
  Gaussian2d map2[m2.numElements()];
  int i;

  for(i = 0; i < m1.numElements(); ++i){
    map1[i] = m1.get(i)->getCenter();
  }

  for(i = 0; i < m2.numElements(); ++i){
    map2[i] = m2.get(i)->getCenter();
  }

  g1.set(map1, m1.numElements());  
  g2.set(map2, m2.numElements());

  matcher.match(g1,g2);
}


void Landmark2dMapMatcher::match(const SimpleMap &m1, const SimpleMap &m2,
                                 const RobotPoseCov &prior){
  Gaussian2d map1[m1.numElements()];
  Gaussian2d map2[m2.numElements()];
  int i;

  for(i = 0; i < m1.numElements(); ++i){
    map1[i] = m1.get(i)->getCenter();
  }

  for(i = 0; i < m2.numElements(); ++i){
    map2[i] = m2.get(i)->getCenter();
  }

  g1.set(map1, m1.numElements());  
  g2.set(map2, m2.numElements());

  matcher.match(g1,g2,prior);
}


RobotPoseCov Landmark2dMapMatcher::getTranslation()const{
  RobotPoseCov xyr;
  int n_matches = matcher.numMatch();

  int ii1[n_matches];
  int ii2[n_matches];

  matcher.commonElements1(ii1);
  matcher.commonElements2(ii2);

  allign_graphs(&xyr, g1, ii1, g2, ii2, n_matches, true);

  return xyr;
}


//============================================================================
// Landmark2dLocGuts
//============================================================================

int Landmark2dLocGuts::associate(int *corr_out, double *w_out,
                                 const RobotPose *robot,
				 const ObservationStore &obs_store,
				 const int obs_ind){

  const Landmark2dObs* obs = (const Landmark2dObs*) obs_store[obs_ind];

  Gaussian2d g(obs->toCartesian(robot));

  int i;
  int nMatch = 0;

  for(i = 0; i < map->numElements(); ++i){
    const Landmark2d *m = (const Landmark2d *) map->get(i);

    double A[2][2];
    double A_inv[2][2];
    double b[2] = {g.x - m->point.x, 
		   g.y - m->point.y};

    matrix2_add(A, m->point.cov, g.cov);
    matrix2_inverse(A_inv, A);

    double md2 = matrix2_multiply2(A_inv, b);

    if(md2 < MD2_MAX){

      double w = VOL2_FIXED_PARAM*sqrt(1.0/matrix2_det(A))*exp(-0.5*md2);
      w_out[i] = w;
      corr_out[nMatch] = i;

      //printf("Matched: %d->%d (%g)\n",obs_ind,i+1,w);

      nMatch += 1;
    }
  }

  return nMatch;
}
