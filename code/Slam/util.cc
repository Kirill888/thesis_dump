#ifndef DEPEND
#include <math.h>
#endif

#include "util.h"
#include "random.h"
//#include "bisearch.h"
#include "AngleTools.h"
#include "slam.h"

/////////////////////////////////////////////////////////////////
// UpperTriangular Matrix
////////////////////////////////////////////////////////////////
void UpperTriangularMatrix::resize(int n){
  if(n <= 0){
    destroy();
  }else{
    this->n = n;
    sz_dat = n*(n+1)/2;

    dat = (int*) realloc(dat, sz_dat*sizeof(int));
    matrix = (int**) realloc(matrix,n*sizeof(int*));

    matrix[0] = dat;
    int i;
    int offset = 1;

    for(i = 1; i < n; ++i){
      matrix[i] = matrix[i-1] + offset;
      offset += 1;
    }
  }
}

void UpperTriangularMatrix::resize(int n, int v){
  int sz_old = sz_dat;

  resize(n);
  int i;

  for(i = sz_old; i < sz_dat; ++i){
    dat[i] = v;
  }
}

void UpperTriangularMatrix::set(const UpperTriangularMatrix &other){
  destroy();
  if(other.n > 0){
    resize(other.n);
    memcpy(dat,other.dat, sz_dat*sizeof(int));
  }
}

void UpperTriangularMatrix::removeRowCol(int r){
  UpperTriangularMatrix m(n-1, -1);
  int i,j;

  for(i = 0; i < r; ++i){
    for(j = i; j < r; ++j)
      m(i,j) = get(i,j);

    for(j = r+1; j < n; ++j)
      m(i,j-1) = get(i,j);
  }

  for(i = r+1; i < n; ++i){
    for(j = i; j < n; ++j){
      m(i-1,j-1) = get(i,j);
    }
  }

  set(m);
}

//////////////////////////////////////////////////////////////////
// Misc.
//////////////////////////////////////////////////////////////////

unsigned int sqrt(unsigned int a){
  unsigned int rem = 0;
  unsigned int root = 0;
  for(int i=0; i < 16; i++){
    root <<= 1;
    rem = ((rem<<2) + (a >> 30));
    a<<= 2;
    root++;
    if(root <= rem){
      rem -= root;
      root++;
    }else{
      root--;
    }
  }
  return (root >> 1);
}

int binarySearch(int key, const int* data, int n){
  int low,high,i;

  for ( low=(-1), high=n;  high-low > 1;  )
    {
      i = (high+low) / 2;
      if ( key <= data[i])  high = i;
      else                  low  = i;
    }

  return high;
}


void sample_cumsum(int *sample_out 
                  ,const double *cumsum, int np
   	          ,int nsample){

  int i;
  double sum = cumsum[np-1];
  UniformRandomNumber dice(0.0, sum);
  double pos  = dice.nextDoubleRandom();
  double step = sum/np;
  int ind     = 0;

  for(i = 0; i < nsample; ++i){
    pos += step;
    if(pos > sum){
      pos = pos - sum;
      ind = 0;
    }

    while(pos > cumsum[ind]){
      ind += 1;
    }

    sample_out[i] = ind;
  }
}

void sample(int *sample_out, 
            const double *w, int np,
   	    int nsample){

  double *cumsum = new double[np];
  int i;


  //  printf("Sampling, max_log %.9e\n", log_max);
  cumsum[0] = w[0];

  for(i = 1; i < np; ++i){
    cumsum[i] = cumsum[i-1] + w[i];
  }

  sample_cumsum(sample_out, cumsum, np, nsample);

  delete[] cumsum;
}

void sample_log(int *sample_out, 
                const double *w_log, int np,
   	        int nsample){

  double *cumsum = new double[np];
  int i;
  double log_max = w_log[0];

  for(i = 1; i < np; ++i){
    if(log_max < w_log[i]) log_max = w_log[i];
  }

  cumsum[0] = exp(w_log[0] - log_max);
  for(i = 1; i < np; ++i){
    cumsum[i] = cumsum[i-1] + exp(w_log[i] - log_max);
  }

  sample_cumsum(sample_out, cumsum, np, nsample);

  delete[] cumsum;
}

static const double *QSORT_data;
static int QSORT_compare_asc(const void* v1, const void* v2){
  int i1 = *((int*)v1);
  int i2 = *((int*)v2);

  double a = QSORT_data[i1];
  double b = QSORT_data[i2];

  if(a > b)      return +1;
  else if(a < b) return -1;
  else           return 0;
}

static int QSORT_compare_dsc(const void* v1, const void* v2){
  int i1 = *((int*)v1);
  int i2 = *((int*)v2);

  double a = QSORT_data[i1];
  double b = QSORT_data[i2];

  if(a > b)      return -1;
  else if(a < b) return +1;
  else           return 0;
}

void sortAscending(int *ind_out, const double* data, int n){
  QSORT_data = data;
  int i;

  for(i = 0; i < n; ++i){ ind_out[i] = i; }

  qsort(ind_out, n, sizeof(int), QSORT_compare_asc);
}

void sortDescending(int *ind_out, const double* data, int n){
  QSORT_data = data;
  int i;

  for(i = 0; i < n; ++i){ ind_out[i] = i; }

  qsort(ind_out, n, sizeof(int), QSORT_compare_dsc);
}


void shuffle_ind(int *deck, int np, int nSwaps){
  int i;

  for(i = 0; i < nSwaps; ++i){
    int k1,k2;
    k1 = rand()%np;
    k2 = rand()%np;

    //Swap k1 and k2
    int tmp = deck[k1];
    deck[k1] = deck[k2];
    deck[k2] = tmp;
  }
}


void normalise(double *w_out, const double *w, int np){
  register int i;
  register double sum = 0.0;

  for(i = 0; i < np; ++i) sum += w[i];

  if(sum > 0){
    register double sum_inv = 1.0/sum;
    for(i = 0; i < np; ++i) w_out[i] = w[i]*sum_inv;
  }
}

void weightedMean(RobotPoseCov *out, 
                  const RobotPose *p, const double *w, int np){

  int i;

  double x,y,a, ax,ay;
  x = 0;
  y = 0;
  ax = 0;
  ay = 0;

  for(i = 0; i < np; ++i){
   //Compute weighted mean.
    x += p[i].x*w[i]; 
    y += p[i].y*w[i]; 

    double ca = cos(p[i].rot);    double sa = sin(p[i].rot);

    ax += w[i]*ca;    ay += w[i]*sa;
  }

  a = atan2(ay,ax);

  out->x   = x;
  out->y   = y;
  out->rot = a;

  //Compute covariance matrix
  out->cov[0][0] = 0.00;  out->cov[0][1] = 0.00;  out->cov[0][2] = 0.00;
  out->cov[1][0] = 0.00;  out->cov[1][1] = 0.00;  out->cov[1][2] = 0.00;
  out->cov[2][0] = 0.00;  out->cov[2][1] = 0.00;  out->cov[2][2] = 0.00;

  for(i = 0; i < np; ++i){
    register double dx,dy,da;
    register double w_ = w[i];

    dx = p[i].x   - x;
    dy = p[i].y   - y;
    da = angleDiffRad(p[i].rot,a);

    out->cov[0][0] += dx*dx*w_;
    out->cov[0][1] += dx*dy*w_;
    out->cov[0][2] += dx*da*w_;
    out->cov[1][1] += dy*dy*w_;
    out->cov[1][2] += dy*da*w_;
    out->cov[2][2] += da*da*w_;
  }

  out->cov[1][0] = out->cov[0][1];
  out->cov[2][0] = out->cov[0][2];
  out->cov[2][1] = out->cov[1][2];
}

void robotPoseMean(RobotPoseCov *out,
		   const RobotPose *p, int np){
  out->cov[0][0] = 0.00;  out->cov[0][1] = 0.00;  out->cov[0][2] = 0.00;
  out->cov[1][0] = 0.00;  out->cov[1][1] = 0.00;  out->cov[1][2] = 0.00;
  out->cov[2][0] = 0.00;  out->cov[2][1] = 0.00;  out->cov[2][2] = 0.00;

  int i;

  double x,y,a, ax,ay;
  x = 0;
  y = 0;
  ax = 0;
  ay = 0;

  double w = 1.0/double(np);

  for(i = 0; i < np; ++i){
   //Compute mean.
    x += p[i].x; 
    y += p[i].y; 

    double ca = cos(p[i].rot);    double sa = sin(p[i].rot);

    ax += ca;    ay += sa;
  }

  x *= w; y *= w; ax *= w; ay *= w;

  a = atan2(ay,ax);

  //Compute covariance matrix
  for(i = 0; i < np; ++i){
    register double dx,dy,da;

    dx = p[i].x   - x;
    dy = p[i].y   - y;
    da = angleDiffRad(p[i].rot,a);

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

bool matlabDump_odo(FILE* f, const char *var_name
		    , const SLAMParticle *const *p
		    , int n_p){
  bool res;
  int i;

  res = fprintf(f,"%s = [ ...\n", var_name) > 0;

  for(i = 0 ; i < n_p && res; ++i){ //For every particle
    const OdometryStoreNode * odo = p[i]->getOdo().getLastNode();
    while(odo != NULL){
      const RobotPose & p = odo->getPose();
      res = fprintf(f,"%d %.9e %.9e %.9e\n"
		    , i
		    , p.x
		    , p.y
		    , p.rot) > 0;

      odo = odo->getParent();
    }
  }

  res &= fprintf(f,"]; %%end %s\n\n", var_name) > 0;


  return res;
}

bool matlabDump_map(FILE *f, const char* var_name
		   , const SLAMParticle *const *p, int n_p){
  bool res;
  int i;
  int j;

  res = fprintf(f,"%s = [ ...\n",var_name) > 0;

  RangeMature range;

  for(i = 0 ; i < n_p && res; ++i){ //For every particle
    SimpleMap* map = p[i]->getMap()->getSubMap(range);

    for(j = 0 ; j < map->numMap() && res; ++j){ //For every map element
      res  = fprintf(f,"%d ",i) > 0;
      res &= map->get(j)->matlabDump(f);
      res &= fprintf(f,"\n");
    }
    delete map;
  }

  res &= fprintf(f,"]; %%end %s\n\n", var_name) > 0;

  return res;
}

void location_destructor(GenericStackType t){
  Location *l = (Location *) t;
  delete l;
}

void locationCov_destructor(GenericStackType t){
  LocationCov *l = (LocationCov *) t;
  delete l;
}

/*
 * Algorithm from N. Wirth's book, implementation by N. Devillard.
 * This code in public domain.
 */

/*---------------------------------------------------------------------------
   Function :   kth_smallest()
   In       :   array of elements, # of elements in the array, rank k
   Out      :   one element
   Job      :   find the kth smallest element in the array
   Notice   :   use the median() macro defined below to get the median. 

                Reference:

                  Author: Wirth, Niklaus 
                   Title: Algorithms + data structures = programs 
               Publisher: Englewood Cliffs: Prentice-Hall, 1976 
    Physical description: 366 p. 
                  Series: Prentice-Hall Series in Automatic Computation 

 ---------------------------------------------------------------------------*/

#define ELEM_SWAP(a,b) { register double t=(a);(a)=(b);(b)=t; }

double kth_smallest(double *a, int n, int k)
{
    register int i,j,l,m ;
    register double x ;

    l=0 ; m=n-1 ;
    while (l<m) {
        x=a[k] ;
        i=l ;
        j=m ;
        do {
            while (a[i]<x) i++ ;
            while (x<a[j]) j-- ;
            if (i<=j) {
                ELEM_SWAP(a[i],a[j]) ;
                i++ ; j-- ;
            }
        } while (i<=j) ;
        if (j<k) l=i ;
        if (k<i) m=j ;
    }
    return a[k] ;
}

