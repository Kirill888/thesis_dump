#include "AngleTools.h"
#include "matrix3.h"
#include "landmark3d.h"
#include "matrix.h"
const int MIN_NUM_OBS = 5;

#ifndef DEPEND
#include <stdlib.h>
#endif

#include "TemplateStore.h"


TemplateStore *templateStore = NULL;

//---------------------------------------------------------------
// Landmark3dMappingProcess
//---------------------------------------------------------------
static Landmark3dSensorRange       sensorRange;

Landmark3dMappingProcess::Landmark3dMappingProcess(){
  sensorRange     = &::sensorRange;

  MAX_MD2 = 16;
}

MapEntryInterface * Landmark3dMappingProcess::
makeNewEntry(double *logProb,
	     const ObservationStore &obs_store, 
	     int ind,
	     const RobotPose* robot)const{

  const Landmark3dObs *obs = (const Landmark3dObs *)obs_store[ind];

  Gaussian3d point(obs->toCartesian(robot));

  *logProb = logVolumeOfTheProduct(point,point) - 5;

  Landmark3d *lndm = new Landmark3d(point);
  lndm->addObs(ind);

  return lndm;
}


const double LOGVOL3_FIXED_PARAM = -1.5*log(2*M_PI);
const double VOL3_FIXED_PARAM    = 1.0/(2*M_PI*sqrt(2*M_PI));

RangeInterface* Landmark3dMappingProcess::
getNeighbourhoodRange(const RobotPose* r){
  return NULL;
}

double match(const Landmark3dObs* o, const Landmark3d* m
             , double minCorr){

  if(templateStore != NULL){
    GenericStack::Enumeration e = m->obs.getElements();
    double corr = -1;

    while(e.hasMoreElements() && corr < minCorr){
      int map_id = (int) e.next();
      corr = templateStore->match(o->id, map_id);
      //      printf("Corr: %d->%d %f\n",o->id, map_id,corr);
    }
    return corr;
  }else{
    return 1;
  }
}

double Landmark3dMappingProcess::
dataAssociate(int *corr_out
	      ,const ObservationInterface*const* obs, int nObs
	      ,const RobotPose *robot
              ,const MapInterface *map){

  double p_log = 0.0;
  int i,j;
  const double minCorr = 0.4;

  for(i = 0; i < nObs; ++i){
    const Landmark3dObs* o = (const Landmark3dObs*) obs[i];
    Gaussian3d g(o->toCartesian(robot));

    double w_max = -100000;
    int ind = -1;

    for(j = 0; j < map->numElements(); ++j){
      const Landmark3d *m = (const Landmark3d*) map->get(j);

      double A[3][3];
      double A_inv[3][3];
      double b[3] = {g.x - m->point.x, 
                     g.y - m->point.y,
                     g.z - m->point.z};

      matrix3_add(A, m->point.cov, g.cov);
      matrix3_inverse(A_inv, A);

      double md2 = matrix3_multiply2(A_inv, b);

      if(md2 < MAX_MD2){
	double w = -(log(matrix3_det(A)) + md2);

	if(ind < 0 || w > w_max){
	  double corr = match(o,m, minCorr);

	  if(corr >= minCorr){
  	     w_max = w;
	     ind = j;
	  }else{
	    //printf("Match failed: %d->%d (%f)\n",i,j,corr);
	  }
	}
      }
    }

    if(ind >= 0){
      p_log += (0.5*w_max + LOGVOL3_FIXED_PARAM);
    }

    corr_out[i] = ind;
  }

  return p_log;
}

//For Local Map
SimpleMap* Landmark3dMappingProcess::loadLocalMap(const MatlabVariable&){
  return NULL;
}



//-------------------------------------------------------------------------
// Landmark3dObs
//-------------------------------------------------------------------------
double Landmark3dObs::logLikelihood(const ObservationInterface* o)const{
  const Landmark3dObs *obs = (const Landmark3dObs *)o;

  double S[3][3];
  double dx[3];
  double aux[3];
  double md2[1];

  dx[0] = range - obs->range;
  dx[1] = angleDiffRad(bearing, obs->bearing);
  dx[2] = angleDiffRad(azimuth, obs->azimuth);

  matrix3_add(S, cov, obs->cov);
  matrix3_inverse(S,S);
  matrix_multiply(aux,dx,S,1,3,3);
  matrix_multiply(md2, aux, dx, 1,3,1);

  return md2[0];
}


Gaussian3d Landmark3dObs::toCartesian(const RobotPose *r)const{
  double x,y,z;
  double COV[3][3];

  double a  = bearing + r->rot;
  double ca = cos(a);
  double sa = sin(a);
  double cb = cos(azimuth);
  double sb = sin(azimuth);

  x = r->x + range*cb*ca;
  y = r->y + range*cb*sa;
  z = range*sb;

  double  F[3][3] = {{cb*ca, -range*cb*sa, -range*sb*ca},
		     {cb*sa,  range*cb*ca, -range*sb*sa},
	             {sb   ,    0        ,  range*cb}};
  double aux[3][3];

  //COV = F*cov*F'
  matrix3_transpose(aux,F);
  matrix3_multiply(aux,cov,aux);
  matrix3_multiply(COV,F,aux);

  return Gaussian3d(x,y,z,COV);
}

Gaussian3d Landmark3dObs::toCartesian()const{
  double x,y,z;
  double COV[3][3];

  double ca = cos(bearing);
  double sa = sin(bearing);
  double cb = cos(azimuth);
  double sb = sin(azimuth);

  x = range*cb*ca;
  y = range*cb*sa;
  z = range*sb;

  double  F[3][3] = {{cb*ca, -range*cb*sa, -range*sb*ca},
		     {cb*sa,  range*cb*ca, -range*sb*sa},
	             {sb   ,    0        ,  range*cb}};
  double aux[3][3];

  //COV = F*cov*F'
  matrix3_transpose(aux,F);
  matrix3_multiply(aux,cov,aux);
  matrix3_multiply(COV,F,aux);

  return Gaussian3d(x,y,z,COV);
}

//---------------------------------------------------------------
// Landmark3d
//---------------------------------------------------------------
void toPolar(Gaussian3d *polar, const Gaussian3d* p
           , const RobotPose* robot){

  double dx,dy,dz;
  dx = p->x - robot->x;
  dy = p->y - robot->y;
  dz = p->z;

  double R,R2, r, r2, R2r_inv, R_inv, r2_inv,b,a;

  r2 = dx*dx + dy*dy;
  R2 = r2 + dz*dz;
  r = sqrt(r2);
  R = sqrt(R2);
  r2_inv  = 1.0/r2;
  R2r_inv = 1.0/(R2*r);
  R_inv   = 1.0/R;

  double H[3][3] = {{ dx*R_inv     ,  dy*R_inv     , dz*R_inv  },
                    {-dy*r2_inv    ,  dx*r2_inv    ,     0     },
                    {-dz*dx*R2r_inv, -dz*dy*R2r_inv, r2*R2r_inv}
                   };

  double Ht[3][3];
  double Sxx[3][3], //Covariance of the map entry in polar coords
         aux1[3][3];

  matrix3_transpose(Ht,H);
  matrix3_multiply(aux1, H, p->cov);
  matrix3_multiply(Sxx, aux1, Ht);   //Sxx = H*cov*H'

  b = angleDiffRad(atan2(dy,dx), robot->rot);
  a = asin(dz*R_inv);

  polar->set(R,b,a,Sxx);
}

void ekf_update(Gaussian3d *p, const Landmark3dObs *obs0, 
                const RobotPose *robot){
  double dx,dy,dz;
  double I[3][3] = {{1.0, 0.0, 0.0} ,
		    {0.0, 1.0, 0.0} ,
		    {0.0, 0.0, 1.0} };

  dx = p->x - robot->x;
  dy = p->y - robot->y;
  dz = p->z;

  double R,R2, r, r2, R2r_inv, R_inv, r2_inv;

  r2 = dx*dx + dy*dy;
  R2 = r2 + dz*dz;
  r = sqrt(r2);
  R = sqrt(R2);
  r2_inv  = 1.0/r2;
  R2r_inv = 1.0/(R2*r);
  R_inv   = 1.0/R;

  double H[3][3] = {{ dx*R_inv     ,  dy*R_inv     , dz*R_inv  },
                    {-dy*r2_inv    ,  dx*r2_inv    ,     0     },
                    {-dz*dx*R2r_inv, -dz*dy*R2r_inv, r2*R2r_inv}
                   };

  double Ht[3][3];
  double K[3][3];
  double Sxx_[3][3], //Covariance of the map entry in polar coords
         aux1[3][3];
  //K = Sxx*H'*inv(H*Sxx*H' + Szz)
  matrix3_transpose(Ht,H);

  matrix3_multiply(aux1, H, p->cov);
  matrix3_multiply(Sxx_, aux1, Ht);   //Sxx_ = H*Sxx*H'
  matrix3_add(aux1, Sxx_, obs0->cov); 
  matrix3_inverse(aux1, aux1);
  matrix3_multiply(aux1, Ht, aux1);
  matrix3_multiply(K, p->cov, aux1);

  double dr = obs0->range - R;
  double b_expected = angleDiffRad(atan2(dy,dx), robot->rot);
  double db = angleDiffRad(obs0->bearing, b_expected);
  double a_expected = asin(dz*R_inv);
  double da = angleDiffRad(obs0->azimuth, a_expected);

  //Update point estimate
  p->x += (K[0][0]*dr + K[0][1]*db + K[0][2]*da);
  p->y += (K[1][0]*dr + K[1][1]*db + K[1][2]*da);
  p->z += (K[2][0]*dr + K[2][1]*db + K[2][2]*da);

  //Update Covariance
  //Sxx = (eye(3) - K*H)*Sxx        % Update covariance
  matrix3_multiply(aux1,K,H);
  matrix3_subtract(aux1, I, aux1);
  matrix3_multiply(p->cov, aux1, p->cov);
}

void Landmark3d::update(const ObservationStore &obs_store,
                        int obs_ind, 
		        const RobotPose *robot){
  const Landmark3dObs * obs0 = (const Landmark3dObs*) obs_store[obs_ind];

  ekf_update(&point, obs0, robot);

  numObs_ += 1;
  lastScanId = obs_store.getScanId(obs_ind);

  if(numObs_ > MIN_NUM_OBS){
    setMature();
  }

  addObs(obs_ind);
}



double Landmark3d::probMatch(const MapEntryInterface* other)const{
  const Landmark3d *m = (const Landmark3d*) other;
  return point.probMatch(m->point);
}

double Landmark3d::probMatchLog(const MapEntryInterface* other)const{
  const Landmark3d *m = (const Landmark3d*) other;
  return point.probMatchLog(m->point);
}

bool Landmark3d::matlabDump(FILE *f)const{
  return fprintf(f,"%e %e %e %e %e %e %e %e %e %e %e %e"
		 ,point.x,point.y,point.z
		 ,point.cov[0][0],point.cov[0][1],point.cov[0][2]
		 ,point.cov[1][0],point.cov[1][1],point.cov[1][2]
		 ,point.cov[2][0],point.cov[2][1],point.cov[2][2]
         ) > 0;
}

void dumpDataAssociations(FILE*f, const SimpleMap* m){
  int i;
  int j;

  int *ind;
  fprintf(f,"map2obs = [\n");
  for(i = 0; i < m->numElements(); ++i){
    const GenericStack &obs = ((const Landmark3d*)m->get(i))->obs;

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


//-------------------------------------------------------------------------
// Landmark3dSensorRange
//-------------------------------------------------------------------------
bool Landmark3dSensorRange::isWithinRange(const ObservationInterface* o)
const{
  const Landmark3dObs* obs = (const Landmark3dObs* ) o;

  double r = obs->range - minRange;
  if(r > dRange) return false;

  double a = angleDiffRad(obs->bearing, minAngle1);
  if(a < 0) a = M_PI - a;

  if( a > dAngle1) return false;

  a = angleDiffRad(obs->azimuth, minAngle2);
  if(a < 0) a = M_PI - a;

  return a < dAngle2;
}
