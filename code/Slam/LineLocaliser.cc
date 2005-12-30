#include "LineLocaliser.h"


extern void robot2sensor(RobotPose*,const RobotPose*);

bool LineMap::load(FILE* f){
  int n;
  double x1,y1,x2,y2;
  int i;


  bool res = fscanf(f,"%d",&n)==1;

  if(!res || n <= 0) return false;

  DESTROY_ARRAY(lines);
  lines = new LineSegment[n];
  n_observed = new int[n];
  n_conflict = new int[n];

  for(i = 0; res && i < n; ++i){
    res = fscanf(f,"%lf %lf %lf %lf",&x1, &y1, &x2, &y2) == 4;

    if(res){
      lines[i].set(x1,y1,x2,y2);
    }
  }

  if(!res){
    DESTROY_ARRAY(lines);
    nlines = 0;
  }else{
    nlines = n;
  }

  return res;
}


void LineLocaliserParticle::evaluate(double dt,
                                     const ObservationStore &obs_store){
  const LineObservation *obs = (const LineObservation*) obs_store.getLast();

  RobotPose robot(odo);
  if(u != NULL){
    u->advance(&robot,dt);
  }

  RobotPose sensor;
  robot2sensor(&sensor, &robot);

#if 1
  int    *match    = new    int[obs->n];
  int    *state    = new    int[obs->n];
  double *wlog     = new double[obs->n];
  double *wlog_max = new double[obs->n];

  int res,i,j;
  double sumWlog = 0;

  const double logProbConflict = -0.1;

  memset(match, 0, obs->n*sizeof(int));
  memset(state, 0, obs->n*sizeof(int));

  for(i = 0; i < map->numLines(); ++i){
    res = matchScanToLine(wlog, match, obs->range, obs->angle, obs->n,
			   &sensor, &(map->line(i)), params);

    if(res > 0){
      for(j = 0; j < obs->n; ++j){
	if(state[j] <= 1){
	  switch(match[j]){
	  case 1://Match
	    if(state[j] == 0){
	      wlog_max[j] = wlog[j];
	      state[j] = 1;
	    }else if(wlog_max[j] < wlog[j]){
	      wlog_max[j] = wlog[j];
	    }

	    map->observe(i);
	    break;
	  case 2://Conflict
	    state[j] = 2;
	    wlog_max[j] = logProbConflict;

	    map->conflict(i);
	    break;
	  case 0://No match
	    break;
	  default:
	    printf("Invalid state: %d\n", match[j]);
	  }
	}
      }
    }
  }

  for(i = 0; i < obs->n; ++i){
    if(state[i] > 0){
      sumWlog = wlog_max[i];
    }
  }

  weight += sumWlog;

  //Clean up.
  delete[] match;
  delete[] state;
  delete[] wlog;
  delete[] wlog_max;

#else
  int n_left = obs->n;
  double *range = new double[obs->n];
  double *angle = new double[obs->n];
  int    *match = new    int[obs->n];
  double *wlog  = new double[obs->n];
  int res,i,j;
  double sumWlog = 0;

  const double logProbNoMatch  = -0.00;
  const double logProbConflict = -0.15;

  memset(match, 0, obs->n*sizeof(int));
  memcpy(range, obs->range, obs->n*sizeof(double));
  memcpy(angle, obs->angle, obs->n*sizeof(double));

  for(i = 0; n_left > 0 && i < map->numLines(); ++i){
    res = matchScanToLine(wlog, match, range, angle,n_left, 
                          &sensor, &(map->line(i)), params);
//     if(i == 1 && res <= 0){
//         printf("Line 2: %d\n", res);
//     }

    if(res > 0){
      int k = 0;
      int n_removed = 0;

      while(k < n_left && match[k] == 0){ k++; }

      for(j = k; j < n_left; ++j){
	if(match[j] == 0){
	  range[k] = range[j];
	  angle[k] = angle[j];
	  k += 1;
	}else{
	  n_removed += 1;

	  if(match[j] == 1){            //Matched to line i
	    sumWlog += wlog[j];
	    map->observe(i);
	  }else if(match[j] == 2){      //Conflict
	    sumWlog += logProbConflict;
	    map->conflict(i);
	  }else{
	    ABORT("evaluate: wrong state -- %d\n",match[j]);
	  }
	}
      }

      n_left -= n_removed;
    }
  }

  sumWlog += n_left*logProbNoMatch;
  weight += sumWlog;

  //Clean up
  delete[] range;
  delete[] angle;
  delete[] match;
  delete[] wlog;
#endif

}


void LineLocaliser::init(int np, const MotionModelInterface *m,
			 LineMap *map,
                         const  struct LineLaserMatchParams* params){
  ParticleFilter::init(m,NULL,0);
  if(np <= 0) return;

  resize(np);

  int i;
  for(i = 0; i < nParticles; ++i){
    LineLocaliserParticle *lp = new LineLocaliserParticle();
    lp->setMap(map);
    lp->setLineParams(params);

    particles[i] = lp;
  }

}

void LineLocaliser::getPose(RobotPoseCov* out)const{

  //Ignore weights of particles
  double w = 1.0/nParticles;
  int i;
  double x,y,a,ax,ay;

  x = 0;  y = 0;  ax = 0;  ay = 0;

  //Compute Mean
  for(i = 0; i < nParticles; ++i){
    x += particles[i]->getPose().x; 
    y += particles[i]->getPose().y; 

    double ca = cos(particles[i]->getPose().rot);    
    double sa = sin(particles[i]->getPose().rot);

    ax += ca;    ay += sa;
  }
  x *= w; y *= w; ax *= w; ay *= w;
  a = atan2(ay,ax);

  //Compute Covariance
  out->cov[0][0] = 0;  out->cov[0][1] = 0;  out->cov[0][2] = 0;
  out->cov[1][0] = 0;  out->cov[1][1] = 0;  out->cov[1][2] = 0;
  out->cov[2][0] = 0;  out->cov[2][1] = 0;  out->cov[2][2] = 0;

  for(i = 0; i < nParticles; ++i){
    register double dx,dy,da;

    dx = particles[i]->getPose().x   - x;
    dy = particles[i]->getPose().y   - y;
    da = angleDiffRad(particles[i]->getPose().rot,a);

    out->cov[0][0] += dx*dx;
    out->cov[0][1] += dx*dy;
    out->cov[0][2] += dx*da;
    out->cov[1][1] += dy*dy;
    out->cov[1][2] += dy*da;
    out->cov[2][2] += da*da;

  }
  out->cov[0][0] *= w;
  out->cov[0][1] *= w;
  out->cov[0][2] *= w;
  out->cov[1][1] *= w;
  out->cov[1][2] *= w;
  out->cov[2][2] *= w;

  out->cov[1][0] = out->cov[0][1];
  out->cov[2][0] = out->cov[0][2];
  out->cov[2][1] = out->cov[1][2];

  out->x = x;
  out->y = y;
  out->rot = a;
}

void LineLocaliser::evaluate(double TimeStamp, 
			     const ObservationStore &obs_store){

  register int i;
  register double maxLog = -Inf;
  register double dt  = odo->getDT(TimeStamp);

  for(i = 0; i < nParticles; ++i){
    particles[i]->evaluate(dt, obs_store);
    double w = particles[i]->getWeight();

    if(w > maxLog){
      maxLog = w;
    }
  }

  for(i = 0; i < nParticles; ++i){
    particles[i]->setWeight(particles[i]->getWeight()-maxLog);
  }

  moved = false;
}

void LineLocaliser::computeCumSum(){
  int i;
  double sum = 0.0;

  for(i = 0; i < nParticles; ++i){
    sum += exp(particles[i]->getWeight());
    cumSumW[i] = sum;
  }
}

void LineLocaliser::resample(){
  computeCumSum();

  if(cumSumW[nParticles - 1] <= 0.0000000001){
    printf("RESAMPLE: Error ZERO SUM %e\n", cumSumW[nParticles-1]);
  }else{
    ParticleFilter::resample();
  }
}

void LineLocaliser::resample(int newSz){
  computeCumSum();
  if(cumSumW[nParticles - 1] <= 0.0000000001){
    printf("RESAMPLE: Error ZERO SUM %e\n", cumSumW[nParticles-1]);
  }else{
    ParticleFilter::resample(newSz);
  }
}
