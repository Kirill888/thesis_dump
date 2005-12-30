#include "edgeLandmark.h"
#include "mapRegion.h"
#include "localmap.h"
#include "matrix.h"
#include "polygon.h"
#include "TimeTools.h"

const double LOGVOL2_FIXED_PARAM = -log(2*M_PI);
const double VOL2_FIXED_PARAM    = 1.0/(2*M_PI);

#define MIN_NUM_OBS 7

//min corr set to 0.5
#define MIN_CORR 16348*1/2


const double MAX_NEIGHBOUR_NORM  = POW2(0.2/3);
const double MAX_NEIGHBOUR_DIST2 = POW2(30);
const double MIN_LANDMARK_NORM   = POW2(0.3);

static bool findFloodSeed(int* ix, int* iy, 
                   const RegionGridMap *reg,
		   const BYTE* data,
			  const RobotPose &r);

static Landmark2dMapMatcher        matcher;

//----------------------------------------------------------------
// EdgeMappingProcess
//----------------------------------------------------------------

EdgeMappingProcess::EdgeMappingProcess():MAX_MD2(DEFAULT_MAX_MD2),cam(NULL){
  //Init Map region
  const double cellSz = 0.1;

  initDefaultMapRegion(cellSz);
  initRobotMask(cellSz);
  matcher         = &::matcher;
} 

MapEntryInterface * 
EdgeMappingProcess::makeNewEntry(double *logProb,
				 const ObservationStore &obs_store, int ind,
				 const RobotPose* sensor)const{
  const EdgeObservation* obs = (const EdgeObservation*)obs_store.get(ind);

  Gaussian2d point;
  obs->toCartesian(&point,cam, sensor);

  *logProb = 0;//logVolumeOfTheProduct(point,point) - 4;

  EdgeLandmark *lndm = new EdgeLandmark(point);
  lndm->addObs(ind);

  return lndm;
}


bool EdgeMappingProcess::isNeighbour(int nHops, const RobotPoseCov* mapRef,
			       const MapAreaInterface* region){
  if(nHops == 1) return true;

//   double dist2 = POW2(mapRef->x) + POW2(mapRef->y);

//   if(dist2 > MAX_NEIGHBOUR_DIST2) return false;

//   double norm = mapRef->norm();

//   //  printf("NORM: %g\n", norm);

//   if(norm < MAX_NEIGHBOUR_NORM) return true;


  return false;

}


int match(const EdgeObservation* o, const EdgeLandmark* m, int minCorr
          , const ObservationStore &obs_store){
  int corr, obs_id;
  GenericStack::Enumeration e = m->obs.getElements();

  do{
    obs_id = (int) e.next(); // landmark must have at least one obs!
    corr = o->correlate((const EdgeObservation*)obs_store[obs_id]);
  }while(e.hasMoreElements() && corr < minCorr);

  //  printf("Corr: %d\n", corr);

  return corr; 
}

//Perform data association
//   corr_out[i] -- should be set to index of the map element from
//                  which observation i is originating,
//                  -1 indicates no data association.
//   obs,nObs    -- Array of observations and it's size
//   robot       -- Robot pose
//   map         -- Map
// 
// Return value: log(Prob obs->map match);

#if 1
//DODGE
extern ObservationStore obs0;

//x,y based
double EdgeMappingProcess::
dataAssociate(int *corr_out
	      ,const ObservationInterface*const* obs, int nObs
	      ,const RobotPose *robot
	      ,const MapInterface *map){
  double p_log = 0.0;
  int i,j;
  Gaussian2d g;

  for(i = 0; i < nObs; ++i){
    const EdgeObservation* o = (const EdgeObservation*) obs[i];
    o->toCartesian(&g,cam,robot);

    double w_max = -100000;
    int ind = -1;

    for(j = 0; j < map->numElements(); ++j){
      const EdgeLandmark *m = (const EdgeLandmark*) map->get(j);

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
	  int corr = match(o,m,MIN_CORR, obs0);

	  if(corr >= MIN_CORR){
// 	    printf("  accepting: %d\n", corr);
	    w_max = w;
	    ind = j;
 	  }else{
// 	    printf("  rejecting: %d\n", corr);
 	  }
	}
      }
    }

    if(ind >= 0 && map->get(ind)->isMature()){
      p_log += (0.5*w_max + LOGVOL2_FIXED_PARAM);
    }

    corr_out[i] = ind;
  }
  
  return p_log;
}


#else
//Pixel based
double EdgeMappingProcess::
dataAssociate(int *corr_out
	      ,const ObservationInterface*const* obs, int nObs
	      ,const RobotPose *robot
	      ,const MapInterface *map){

  int nmap0 = map->numElements();

  Gaussian2d *m = new Gaussian2d[nmap0];
  int *mi = new int[nmap0];

  int i,j;
  int nmap = 0;
  double u[2];
  double p_log = 0;
  double H[2][2];

  const EdgeLandmark *landmark;
  const EdgeObservation* o ;

  //1st. Project landmarks into image plane
  //     discard landmarks outside the plane

  for(i = 0; i < nmap0; ++i){
    landmark = (const EdgeLandmark*) map->get(i);

    if(landmark->projectToImagePlane(u,m[nmap].cov,H,robot, cam)){
      if(m[nmap].cov[0][0] < 100 && m[nmap].cov[1][1] < 100){
	m[nmap].x = u[0];
	m[nmap].y = u[1];
	mi[nmap]  = i;

	nmap += 1;
      }
    }else{
    }
  }

  //2.nd Perform data association
  for(i = 0; i < nObs; ++i){
    o = (const EdgeObservation*) obs[i];
    double w_max = -100000;
    int ind      = -1;

    for(j = 0; j < nmap; ++j){
      double A[2][2];
      double A_inv[2][2];
      double b[2] = {o->u1 - m[j].x, 
                     o->u2 - m[j].y};

      matrix2_add(A, m[j].cov, o->cov);
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
      corr_out[i] = mi[ind];
    }else{
      corr_out[i] = -1;
    }
  }

  //3rd -- Clean up.
  delete[] m;
  delete[] mi;

  return p_log;
}
#endif


#include "obs.sample"

//MultiSLAM stuff dealing with Map area

double EdgeMappingProcess::computeRegionScore(const MapAreaInterface* area, 
                                              const RobotPose* robot){

#if 1
  int i;
  double score = 0;

  for(i = 0; i < n_obs_sample; ++i){
    double a = angleDiffRad(robot->rot, - obs_sample[i][0]);
    double r= obs_sample[i][1];
    double ca = cos(a);
    double sa = sin(a);
    double x,y;

    x = r*ca + robot->x;
    y = r*sa + robot->y;

    score += area->probIsInside(x,y);
  }

  score = 3*score/n_obs_sample;

  if(score > 1.0) score = 1.0;

  return score;

#else
  double score = 0;
  double   range[] = { 3, 5 , 7, 10, 12, 15};
  double w_range[] = {0.125,0.125,0.250,0.250,0.125,0.125};

  const int n_range = 6;
  const int n_angle = 7;
  double da[] = {-0.35,-0.175, -0.08727,0.00000,0.08727,0.175, 0.35};

  int i,j;

  for(i = 0; score < 1.0 && i < n_angle; ++i){
    double a = robot->rot + da[i];
    double ca = cos(a);
    double sa = sin(a);

    for(j = 0; score < 1.0 && j < n_range; ++j){
      double x,y,w;
      x = range[j]*ca + robot->x;  
      y = range[j]*sa + robot->y;

      w = area->probIsInside(x,y)*w_range[j]*0.33333333;

      score += w;
    }
  }

  if(score > 1) score = 1.0;

  return score;
#endif
}

void EdgeMappingProcess::getInitialMapArea(MapAreaInterface **mapArea, 
                                           RobotPose* robot){
  robot->x   = 0.0;
  robot->y   = 0.0;
  robot->rot = 0.0;

  *mapArea = new RegionGridMap(region0->clone());
}

double EdgeMappingProcess::subtractFromRegion(
                          MapAreaInterface* a,
			  const MapAreaInterface *b){

        RegionGridMap* reg1 = (RegionGridMap*)a;
  const RegionGridMap* reg2 = (const RegionGridMap*)b;

  int npix = reg1->subtract(reg2);

  double area = npix*reg1->getGrid()->cellX()*reg1->getGrid()->cellY();

  return area;
}

double EdgeMappingProcess::subtractFromRegion(MapAreaInterface* a,
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

void EdgeMappingProcess::finaliseMapArea(MapAreaInterface* a, 
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

void EdgeMappingProcess::finaliseMapArea(MapAreaInterface* a){
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

void EdgeMappingProcess::finaliseMapArea(MapAreaInterface* area
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

  printf("Computing area overlap...");
  poly.expand(1.2);

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


void EdgeMappingProcess::initRobotMask(double cell_sz){
  const double ROBOT_DIAMETER = 0.8;

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

void EdgeMappingProcess::initDefaultMapRegion(double cellSz){
  const double R = 10;

  int n = int(R/cellSz);
  const double fwd = 0.7;
  const double v2h = 0.5;

  region0 = new GridMap((fwd-1)*R, 2*R, 
                        -R*v2h, 2*R*v2h, n,n);
  region0->setAll(1.0);
}

extern void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, double *weight, int nsamples,int);


void EdgeMappingProcess::sampleTransition(const SimpleMap* m1 
					  ,const SimpleMap* m2
					  ,const RobotPoseCov *m2in1
					  ,RobotPose *samples
					  ,double *weight
					  ,int nsamples){

  ::sampleTransition(m1,m2,m2in1,samples, weight, nsamples,0);
}


//----------------------------------------------------------------
// EdgeObservation
//----------------------------------------------------------------
/*


>> pretty(J)     
 
        [                       2
        [%1 x2 - y2      (1 + %1 ) (x2 u1 - y2)        %1         1
        [----------- , - ---------------------- , - -------- , -------- ,
        [          2               2        2       -%1 + u1   -%1 + u1
        [(-%1 + u1)      (-%1 + u1)  (1 + u2 )

                 2              ]
          (1 + %1 ) (x2 u1 - y2)]
        - ----------------------]
                         2      ]
               (-%1 + u1)       ]

        [                              2
        [(%1 x2 - y2) %1     u1 (1 + %1 ) (x2 u1 - y2)      u1 %1        u1
        [--------------- , - ------------------------- , - -------- , --------
        [            2                   2        2        -%1 + u1   -%1 + u1
        [  (-%1 + u1)          (-%1 + u1)  (1 + u2 )

                      2              ]
            u1 (1 + %1 ) (x2 u1 - y2)]
        , - -------------------------]
                             2       ]
                   (-%1 + u1)        ]


>> pretty([x;y]);
 
                              [    y2 - %1 x2   ]
                              [    ----------   ]
                              [     -%1 + u1    ]
                              [                 ]
                              [  u1(y2 - %1 x2) ]
                              [  ---------------]
                              [     -%1 + u1    ]

                             %1 := tan(atan(u2) + a2)

%  u1,u2 -- normalised

*/

void EdgeObservation::toCartesian(Gaussian2d *point
         , const StereoPair* cam, const RobotPose* sensor)const{

  double x,y;
  double den,aux1;
  double J1[2][2];
  double J2[3][3];
  double tmp[2][2];

  double x2,y2;

  x2 = cam->X2();
  y2 = cam->Y2();

  aux1 = tan(atan(u2) + cam->A2());
  den = 1.0/(-aux1 + u1);

  x = (y2 - aux1*x2)*den;
  y =  u1*x;

  J2[0][1] = u1*den;
  J2[0][0] = -J2[0][1]*aux1;
  J2[0][2] = -u1*( 1 + POW2(aux1) ) * ( x2*u1 - y2 )*den*den;
  J2[1][0] = u1*J2[0][0];
  J2[1][1] = u1*J2[0][1];
  J2[1][2] = u1*J2[0][2];

  J1[0][0] = -x*den;
  J1[1][0] = J1[0][0]*aux1;
  J1[0][1] = J2[0][2]/(1 + u2*u2);
  J1[1][1] = u1*J1[0][1];


  //point->x = x;
  point->x = x*cam->tilt_cos;
  point->y = y;

  //Compute uncertainty.
  //  point->cov = J2*cam->pose2in1_cov*J2' + J1*cov*J1'

  //uncertainty from relative camera pose.
  double J2_t[3][2];
  double A[2][3];
  matrix_transpose(J2_t,J2,2,3);
  matrix_multiply(A,J2,cam->pose2in1_cov,2,3,3);
  matrix_multiply(point->cov,A,J2_t,2,3,2);


  //uncertainty from pixel errors
  matrix2_multiply2(tmp, cov, J1);

  //Add them up.
  matrix2_add(point->cov,point->cov,tmp); 

  point->translateMe(*sensor);
}

int EdgeObservation::correlate(const EdgeObservation* other)const{
  int i;
  int s = 0;

  for(i = 0; i < EDGE_TEMPLATE_SIZE; ++i){
    s += Template[i]*other->Template[i];
  }

  return s;
}

//----------------------------------------------------------------
// EdgeLandmark
//----------------------------------------------------------------

double EdgeLandmark::projectToImagePlane(double h[2], 
                                         const CameraModel *cam, 
                                         const RobotPose *pose
	)const{
  double ca,sa;
  double u;
  double x,y,tmp;
  double R[2][2];

  //Translate point into camera coords
  ca = cos(pose->rot);
  sa = sin(pose->rot);
  x = point.x - pose->x;
  y = point.y - pose->y;

  tmp = x*ca + y*sa;
  y  = -x*sa + y*ca; 
  x  = tmp;

  u    =  y/x;
  h[0] = -u/x;
  h[1] =  1/x;

  R[0][0] =  ca; R[0][1] = sa;
  R[1][0] = -sa; R[1][1] = ca;

  matrix2_multiply(h,h,R);

  return u;
}

bool EdgeLandmark::projectToImagePlane(double u[2], double Szz[2][2]
                                     , double H[2][2]
                                     , const RobotPose* robot

                                     , const StereoPair* cam)const{

  u[0] = projectToImagePlane(H[0], &(cam->camera1), robot);
  RobotPose cam2(cam->pose2in1);
  cam2.translateMe(*robot);

  u[1] = projectToImagePlane(H[1], &(cam->camera2), &cam2);

  matrix2_multiply2(Szz, point.cov, H); //Szz = H*point.cov*H'
  return true;
}

#if 0
//x,y based
//kalman2.h
void kalman_update(Gaussian2d *x, const Gaussian2d *o);

void EdgeLandmark::update(const ObservationStore &obs_store,
			  int obs_ind, 
			  const RobotPose *robot){

  const StereoPair *cam = ((const EdgeMappingProcess*)mappingProcess)->cam;
  const EdgeObservation* o = (const EdgeObservation*) obs_store[obs_ind];
  Gaussian2d z;

  o->toCartesian(&z, cam, robot);
  kalman_update(&point, &z);

  //Update number of observations
  numObs_ += 1;
  lastScanId = obs_store.getScanId(obs_ind);

  if(isNew() && numObs_ > MIN_NUM_OBS){
    setMature();
  }
}

#else
// pixel based
void EdgeLandmark::update(const ObservationStore &obs_store,
			  int obs_ind, 
			  const RobotPose *robot){

  const StereoPair *cam = ((const EdgeMappingProcess*)mappingProcess)->cam;
  const EdgeObservation* o = (const EdgeObservation*) obs_store[obs_ind];

  double H[2][2]; //Jacobian x,y -> u1,u2 
  double K[2][2]; //Kalman gain

  double Ht[2][2]; //Transpose of H
  double Szz[2][2];//H*P*H'  -- covariance of the projection to pixel plane
  double aux[2][2];

  double u[2], dz[2],dx[2];


  if(!projectToImagePlane(u,Szz,H, robot,cam)){
    //    fprintf(stderr, "landmark::update -- Projection failed!\n");
    //    return;
  }

  //Compute Kalman Gain 
  // K = P*H'*inv(H*P*H' + Z_obs)
  matrix2_transpose(Ht,H);
  matrix2_add(K,Szz, o->cov);         //K   =          H*P*H' + Z_obs
  matrix2_inverse(K,K);               //K   =      inv(H*P*H' + Z_obs)
  matrix2_multiply(K, Ht, K);         //K   =   H'*inv(H*P*H' + Z_obs)
  matrix2_multiply(K, point.cov, K);  //K   = P*H'*inv(H*P*H' + Z_obs)

  dz[0] = o->u1 - u[0];
  dz[1] = o->u2 - u[1];

#if 0
  printf("%%BakProject: xx = [%.8e,%.8e,%.8e,%.8e,%.8e,%.8e]\n"
  	 ,u[0],u[1]
	 ,Szz[0][0], Szz[0][1]
	 ,Szz[1][0], Szz[1][1]);

  printf("x = [%.8e;%.8e]\n", point.x, point.y);
  printf("z = [%.8e;%.8e]\n", o->u1, o->u2);
  printf("z_obs = [%.8e,%.8e;%.8e,%.8e]\n"
   , o->cov[0][0],o->cov[0][1],o->cov[1][0],o->cov[1][1]);
  printf("P = [%.8e,%.8e; %.8e,%.8e]\n"
	 , point.cov[0][0],point.cov[0][1]
	 , point.cov[1][0],point.cov[1][1]);

  printf("H = [%.8e,%.8e; %.8e,%.8e]\n", H[0][0],H[0][1],H[1][0],H[1][1]);

  printf("Szz = [%.8e,%.8e; %.8e,%.8e]\n", Szz[0][0],Szz[0][1],Szz[1][0],Szz[1][1]);
  printf("K = [%.8e,%.8e;%.8e,%.8e]\n", K[0][0],K[0][1], K[1][0],K[1][1]); 
  printf("dz = [%.8e ; %.8e]\n",dz[0],dz[1]);
#endif



  //Update state
  matrix2_multiply(dx,K,dz);
  point.x += dx[0];
  point.y += dx[1];

  //Update state covariance
  // P = (I - K*H)*P
  matrix2_multiply(aux,K,H);
  matrix2_subtract(aux,I_2x2, aux);
  matrix2_multiply(point.cov, aux, point.cov);

  //Update number of observations
  numObs_ += 1;
  lastScanId = obs_store.getScanId(obs_ind);
  addObs(obs_ind);

  if(isNew() && numObs_ > MIN_NUM_OBS){
    if(norm() < MIN_LANDMARK_NORM ){
       setMature();
    }
  }

}

#endif



//-------------------------------------------------------------
// SUPPORT FUNCTIONS
//-------------------------------------------------------------
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

bool Edge_storeLocalMap(FILE *f, const LocalMap *lmap){
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

  dumpDataAssociations(f,map);

  return res;
}

void dumpDataAssociations(FILE*f, const SimpleMap* m){
  int i;
  int j;

  int *ind;
  fprintf(f,"map2obs = [\n");
  for(i = 0; i < m->numElements(); ++i){
    const GenericStack &obs = ((const EdgeLandmark*)m->get(i))->obs;

    //    printf("Dump: %d, %d\n",i,obs.numElements());

    ind = new int[obs.numElements()];
    obs.getAll((void**)ind);

    for(j = 0; j < obs.numElements(); ++j){
      fprintf(f,"%03d %04d\n", i+1, ind[j]+1);
    }

    delete[] ind;
  }
  fprintf(f,"];%%map2obs\n");
}
