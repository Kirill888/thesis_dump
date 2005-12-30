#ifndef DEPEND
#include <math.h>
#include <stdio.h>
#endif

#include "corner.h"
#include "util.h"
#include "image.h"
#include "gridmap.h"
#include "mapRegion.h"

double probMatchAngleLog(double a1, double sa1, 
			 double a2, double sa2);

double probMatchAngle(double a1, double sa1, 
		      double a2, double sa2);

const double LOGVOL2_FIXED_PARAM = -log(2*M_PI);
const double VOL2_FIXED_PARAM    = 1.0/(2*M_PI);

const double ROBOT_DIAMETER = 0.75;
//-------------------------------------------------------------
// CornerObservation
//-------------------------------------------------------------
double CornerObservation::logLikelihood(const ObservationInterface* obs)const{
  double p_log;

  const CornerObservation* o = (const CornerObservation*) obs;

  p_log = Landmark2dObs::logLikelihood(obs);

  p_log += md2_angle(a1,sa1+o->sa1,o->a1);
  p_log += md2_angle(a2,sa2+o->sa2,o->a2);

  return p_log;
}

bool CornerObservation::matlabDump(FILE *f)const{
  return fprintf(f,"%.8e %.8e %.8e %.8e %.8e %.8e   %.8e %.8e %d  %.8e %.8e %d"
                 ,range, bearing
                 , cov[0][0], cov[0][1]
                 , cov[1][0], cov[1][1]
		 , a1, sa1, dir1
		 , a2, sa2, dir2) > 0;
}


//-------------------------------------------------------------
// CornerLandmark::
//-------------------------------------------------------------

void CornerLandmark::update(const ObservationStore &obs_store,
			    int obs_ind, 
			    const RobotPose *r){

 Landmark2d::update(obs_store, obs_ind, r);

 const CornerObservation* o = (const CornerObservation*) obs_store[obs_ind];

 double da1, da2;
 double A1 = angleDiffRad(o->a1, -r->rot);
 double A2 = angleDiffRad(o->a2, -r->rot);

 if(type == o->type || 
    (   type == JUMP && o->type == CONVEX_HIDDEN) ||
    (o->type == JUMP &&    type == CONVEX_HIDDEN)){
   updateA1(A1, o->sa1);

   if(type <= CONVEX){ //Convex and concave
     updateA2(A2, o->sa2);
   }

 }else if(type == CONVEX && o->type == CONVEX_HIDDEN){
   //Update only one side
   da1 = angleDiffRad(a1, A1);
   da2 = angleDiffRad(a2, A1);

   if(fabs(da1) < fabs(da2)){
     updateA1(A1, o->sa1);
   }else{
     updateA2(A1, o->sa1);
   }

 }else if(type == CONVEX_HIDDEN && o->type == CONVEX){
   da1 = angleDiffRad(a1, A1);
   da2 = angleDiffRad(a1, A2);

   if(fabs(da1) < fabs(da2)){
     //Match 1->1

     //Update a1
     updateA1(A1, o->sa1);

     //Set a2
     a2  = A2;
     sa2 = o->sa2;
   }else{
     //Match 1->2
     //Set a2 to updated a1
     updateA1(A2, o->sa2);
     a2 = a1;

     //Set a1 to a2 of observation
     a1 = A2; sa1 = o->sa2;
   }

   //Set to convex now
   type = CONVEX;
 }

  numObs_ += 1;
  lastScanId = obs_store.getScanId(obs_ind);

  if(isNew() && numObs_ > 5){
    setMature();
  }
}

void CornerLandmark::updateA1(double a, double saa){
  double da = angleDiffRad(a1,a);
  double K = sa1/(sa1 + saa);

//   printf("UpdateA1: %g -- %g (diff = %g, %g)\n"
//     , rad2deg(a1), rad2deg(a), rad2deg(da), saa);

  a1 = angleDiffRad(a1, K*da);
  sa1 = (1-K)*sa1;
}

void CornerLandmark::updateA2(double a, double saa){
  double da = angleDiffRad(a, a2);
  double K  = sa2/(sa2 + saa);

  a2  = angleDiffRad(a2, -K*da);
  sa2 = (1-K)*sa2;
}

double CornerLandmark::probMatch(const MapEntryInterface* m)const{
  const CornerLandmark *other = (const CornerLandmark*) m;
  double p = Landmark2d::probMatch(m);

  return p;

  double pa1, pa2;

  if(type == other->type|| (type == JUMP && other->type == CONVEX_HIDDEN) ||
     (other->type == JUMP &&    type == CONVEX_HIDDEN)){

    pa1 = probMatchAngle(a1,sa1,other->a1,other->sa1);
    if(type <= CONVEX){
      pa2 = probMatchAngle(a2,sa2,other->a2,other->sa2);
    }else{
      pa2 = 1;
    }
  }else{
    if(type == CONVEX && other->type == CONVEX_HIDDEN){
      pa1 = probMatchAngle(a1,sa1,other->a1,other->sa1);
      pa2 = probMatchAngle(a2,sa2,other->a1,other->sa1);

      pa1 = max(pa1,pa2);
      pa2 = 1;

    }else if(type == CONVEX_HIDDEN && other->type == CONVEX){
      pa1 = probMatchAngle(a1,sa1,other->a1,other->sa1);
      pa2 = probMatchAngle(a1,sa1,other->a2,other->sa2);

      pa1 = max(pa1,pa2);
      pa2 = 1;

    }else{ return 0; }
  }

  return p*pa1*pa2;
}

double CornerLandmark::probMatchLog(const MapEntryInterface* m)const{
  const CornerLandmark *other = (const CornerLandmark*) m;
  double p = Landmark2d::probMatchLog(m);

  return p;

  double pa1, pa2;
  //TODO: take care of visible side (dir1,dir2)

  if(type == other->type ||   
     (   type == JUMP     && other->type == CONVEX_HIDDEN) ||
     (other->type == JUMP &&        type == CONVEX_HIDDEN)){
    pa1 = probMatchAngleLog(a1,sa1,other->a1,other->sa1);
    if(type <= CONVEX){
      pa2 = probMatchAngleLog(a2,sa2,other->a2,other->sa2);
    }else{
      pa2 = 0;
    }
  }else{
    if(type == CONVEX && other->type == CONVEX_HIDDEN){
      pa1 = probMatchAngleLog(a1,sa1,other->a1,other->sa1);
      pa2 = probMatchAngleLog(a2,sa2,other->a1,other->sa1);

      pa1 = max(pa1,pa2);
      pa2 = 0;

    }else if(type == CONVEX_HIDDEN && other->type == CONVEX){
      pa1 = probMatchAngleLog(a1,sa1,other->a1,other->sa1);
      pa2 = probMatchAngleLog(a1,sa1,other->a2,other->sa2);

      pa1 = max(pa1,pa2);
      pa2 = 0;

    }else{ return -Inf; }
  }

  return p+pa1+pa2;
}

double CornerLandmark::mahalanobis2(const MapEntryInterface* m)const{
  //  const CornerLandmark *other = (const CornerLandmark*) m;
  ABORT("CornerLandmark::mahalanobis2 -- not implemented\n");
  return 0;
}

bool CornerLandmark::isCompatible(const CornerObservation*  o)const{
  if(type == o->type ||
    (   type == JUMP && o->type == CONVEX_HIDDEN) ||
    (o->type == JUMP &&    type == CONVEX_HIDDEN)) return true;

  return ((type == CONVEX        && o->type == CONVEX_HIDDEN)||
          (type == CONVEX_HIDDEN && o->type == CONVEX)); 
}

double CornerLandmark::MatchAnglesMD2(const CornerObservation* o, 
                                      const RobotPose* r, double *saa)const{
 
  double pa1, pa2;
  //TODO: take care of visible side (dir1,dir2)
  double A1 = angleDiffRad(o->a1, -r->rot);
  double A2 = angleDiffRad(o->a2, -r->rot);

  if(type == o->type){
    pa1 = md2_angle(a1,sa1 + o->sa1,A1);
    *saa = sa1 + o->sa1;

    if(type > CONVEX)  return pa1;

    pa2 = md2_angle(a2,sa2 + o->sa2,A2);

    (*saa) *= (sa2 + o->sa2);

  }else{
    if(type == CONVEX && o->type == CONVEX_HIDDEN){
      pa1 = md2_angle(a1, sa1 + o->sa1, A1);
      pa2 = md2_angle(a2, sa2 + o->sa1, A1);

      if(pa1 < pa2){
	*saa = sa1 + o->sa1;
	return pa1;
      }else{
	*saa = sa2 + o->sa1;
	return pa2;
      }

    }else if(type == CONVEX_HIDDEN && o->type == CONVEX){
      pa1 = md2_angle(a1,sa1 + o->sa1, A1);
      pa2 = md2_angle(a1,sa1 + o->sa2, A2);

      if(pa1 < pa2){
	*saa = sa1 + o->sa1;
	return pa1;
      }else{
	*saa = sa1 + o->sa2;
	return pa2;
      }

    }else{ return +Inf; }
  }

  return pa1 + pa2;
}


/*****************************************************************
//This was used for data assocciation but is no longer needed
//
double CornerLandmark::probMatch(const ObservationInterface*,    
                                 const RobotPose*)const{
}

double CornerLandmark::probMatchLog(const ObservationInterface*, 
                                    const RobotPose*)const{
}
double CornerLandmark::mahalanobis2(const ObservationInterface*, 
                                   const RobotPose*)const{
}
*/


bool CornerLandmark::matlabDump(FILE* f)const{
  return fprintf(f,"%e %e %e %e %e %e  %d %e %e %e %e %d %d %d"
		  ,point.x
		  ,point.y
		  ,point.cov[0][0]
		  ,point.cov[0][1]
		  ,point.cov[1][0]
		  ,point.cov[1][1]
		  ,type
		  ,a1, a2, sa1, sa2, dir1, dir2
                  ,state) > 0;
}

void CornerLandmark::translateMe(const RobotPose &r){
  point.translateMe(r);
  a1 = angleDiffRad(a1, -r.rot);
  if(type <= CONVEX) a2 = angleDiffRad(a2, -r.rot);
}

//Translate with adding uncertainty
void CornerLandmark::translateMe(const RobotPoseCov &r){
  point.translateMe(r);

  a1   = angleDiffRad(a1, -r.rot);
  sa1 += r.cov[2][2];

  if(type <= CONVEX){
    a2 = angleDiffRad(a2, -r.rot);
    sa2 += r.cov[2][2];
  }
}



//-------------------------------------------------------------
// Corner Mapping Process
//-------------------------------------------------------------


MapEntryInterface * CornerMappingProcess::
makeNewEntry(double *logProb,
	     const ObservationStore &obs_store, 
	     int ind,
	     const RobotPose* robot)const{

  const CornerObservation *obs = (const CornerObservation *)obs_store[ind];
      
  Gaussian2d point(obs->toCartesian(robot));

  *logProb = logVolumeOfTheProduct(point,point) - 4;

  double A1,A2;

  A1 = angleDiffRad(obs->a1, - robot->rot);

  if(obs->type <= CONVEX) A2 = angleDiffRad(obs->a2, -robot->rot);
  else                    A2 = 0;

  return new CornerLandmark(point,obs->type
                            , A1
                            , obs->sa1, obs->dir1
			    , A2
                            , obs->sa2, obs->dir2);
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
double CornerMappingProcess::
dataAssociate(int *corr_out
	      ,const ObservationInterface*const* obs, int nObs
	      ,const RobotPose *robot
	      ,const MapInterface *map){
  double p_log = 0.0;
  int i,j;

  for(i = 0; i < nObs; ++i){
    const CornerObservation* o = (const CornerObservation*) obs[i];
    Gaussian2d g(o->toCartesian(robot));

    double w_max = -100000;
    int ind = -1;

//     double d_max = 10*sqrt(max(g.cov[0][0], g.cov[1][1]) 
//                             + fabs(g.cov[0][1]));

    for(j = 0; j < map->numElements(); ++j){
      const CornerLandmark *m = (const CornerLandmark*) map->get(j);

//       printf("Matching %d (%d)->%d(%d)\n",i+1,o->type, j+1,m->type);

      if(m->isCompatible(o)){
	double angle_md2;
	double angle_sigma2;

	angle_md2 = m->MatchAnglesMD2(o, robot, &angle_sigma2);

//  	printf("   Compatible types: angle_md2 = %g (%g,%g,%g)\n"
//             , angle_md2, rad2deg(m->a1), rad2deg(o->a1), rad2deg(robot->rot));


	if(angle_md2 < MAX_MD2){
	  double A[2][2];
	  double A_inv[2][2];
	  double b[2] = {g.x - m->point.x, 
			 g.y - m->point.y};

	  matrix2_add(A, m->point.cov, g.cov);
	  matrix2_inverse(A_inv, A);

	  double md2 = matrix2_multiply2(A_inv, b);

	  if(md2 < MAX_MD2){
	    double w = -(log(matrix2_det(A)*angle_sigma2) 
                         + md2 + angle_md2);

	    if(ind < 0 || w > w_max){
	      w_max = w;
	      ind = j;
	    }
	  }
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

//For Local Map
SimpleMap* CornerMappingProcess::loadLocalMap(const MatlabVariable& m){
  printf("Corner::loadLocalMap\n");

  if(m.numCols() != 14){
    printf("Bad Matlab Matrix cols = %d (need 14)\n",m.numCols());
    return NULL;
  }

  SimpleMap *map = new SimpleMap(m.numRows());
  int i;

  for(i = 0; i < m.numRows(); ++i){
    double x,y, cov[2][2];
    CornerType type;

    double a1, sa1;
    double a2, sa2;
    int dir1,dir2;

    int st;
    x = m.get(i,0);
    y = m.get(i,1);
    cov[0][0] = m.get(i,2);
    cov[0][1] = m.get(i,3);
    cov[1][0] = m.get(i,4);
    cov[1][1] = m.get(i,5);

    type = Int2CornerType(int(m.get(i,6)));
    a1 = m.get(i,7);
    a2 = m.get(i,8);
    sa1 = m.get(i,9);
    sa2 = m.get(i,10);
    dir1 = int(m.get(i,11));
    dir2 = int(m.get(i,12));
    st = (int) m.get(i,13);

    CornerLandmark *landmark = 
      new CornerLandmark(Gaussian2d(x,y,cov),type
			 ,a1,sa1,dir1
                         ,a2,sa2,dir2);

    landmark->setState(st);

    map->set(i, landmark);
  }

  return map;
}

CornerType Int2CornerType(int i){
  switch(i){
  case CONVEX:
    return CONVEX;
  case CONVEX_HIDDEN:
    return CONVEX_HIDDEN;
  case JUMP:
    return JUMP;
  case CONCAVE:
    return CONCAVE;
  default:
    return CORNER_UNKNOWN;
  }
}

void CornerMappingProcess::getInitialMapArea(
          MapAreaInterface **mapArea, RobotPose* robot){

  GridMap * map = gridMapper->computeGridMap(robot);

  printf("LNDM2D::getInitialMapArea\n");
  printf("   odo = [%e, %e, %e]; %% %.2f\n"
         ,robot->x, robot->y, robot->rot, robot->rot*RADTODEG);

  BYTE mask[3][3] = {{1,1,1},
		     {1,1,1},
		     {1,1,1}};

  image_dilate(map->getData(),map->getData(),map->numY(), map->numX()
              , (BYTE*) mask, 3, 3);

  if(robotMask == 0){
    initRobotMask(ROBOT_DIAMETER, map->cellX());
  }

  image_mask(map->getData(),map->getData(),map->numY(), map->numX()
              ,robotMask, robotMask_h, robotMask_w);

  image_dilate(map->getData(),map->getData(),map->numY(), map->numX()
              , (BYTE*) mask, 3, 3);

  *mapArea = new RegionGridMap(map);
}

//-------------------------------------------------------------
// Corner Localisation
//-------------------------------------------------------------

int CornerLocGuts::associate(int *corr_out, double *w_out,
			     const RobotPose *p,
			     const ObservationStore &obs_store,
			     const int obs_ind){

  const CornerObservation* o = (const CornerObservation*) obs_store[obs_ind];
  Gaussian2d g(o->toCartesian(p));
  int i;
  int nMatch = 0;

  for(i = 0; i < map->numElements(); ++i){
    const CornerLandmark *m = (const CornerLandmark *) map->get(i);
    if(m->isCompatible(o)){
      double angle_md2;
      double angle_sigma2;

      angle_md2 = m->MatchAnglesMD2(o, p, &angle_sigma2);

      //  	printf("   Compatible types: angle_md2 = %g (%g,%g,%g)\n"
      //  , angle_md2, rad2deg(m->a1), rad2deg(o->a1), rad2deg(robot->rot));


      if(angle_md2 < MD2_MAX){
	double A[2][2];
	double A_inv[2][2];
	double b[2] = {g.x - m->point.x, 
		       g.y - m->point.y};

	matrix2_add(A, m->point.cov, g.cov);
	matrix2_inverse(A_inv, A);

	double md2 = matrix2_multiply2(A_inv, b);

	if(md2 < MD2_MAX){
	  double w = VOL2_FIXED_PARAM/sqrt(matrix2_det(A)*angle_sigma2)
	    *exp(-0.5*(md2 + angle_md2));
	  w_out[i] = w;
	  corr_out[nMatch] = i;
	  nMatch += 1;
	}
      }
    }
  }

  return nMatch;
}


//-------------------------------------------------------------
// Support Functions
//-------------------------------------------------------------
double probMatchAngle(double a1, double sa1, 
		      double a2, double sa2){

  double sigma2 = sa1 + sa2;
  double dx =  angleDiffRad(a1,a2);

  double p = 1.0/sqrt(2*M_PI*sigma2)*exp(-0.5*dx*dx/sigma2);

  return p;
}

double probMatchAngleLog(double a1, double sa1, 
		      double a2, double sa2){

  double sigma2 = sa1 + sa2;
  double dx =  angleDiffRad(a1,a2);

  const double CONST_PARAM = -0.5*log(2*M_PI);
  double p_log = -0.5*(dx*dx/sigma2 + log(sigma2)) + CONST_PARAM;

  return p_log;
}


