#ifndef __UTIL_H_
#define __UTIL_H_

#ifndef DEPEND
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#endif

#include "geometry.h"
#include "random.h"
#include "DataStore.h"

#define ABORT(msg,args...) {fprintf(stderr,msg,##args); exit(1);}

#define DESTROY_ARRAY(a) if((a) != NULL){ delete[] (a); (a) = NULL;}

#define DESTROY_ARRAY2(a,sz) if((a)!=NULL){\
                                for(int i = 0; i < (sz); ++i) delete[] (a)[i];\
                                delete[] (a); (a) = NULL;}

#define DESTROY(a) if((a) != NULL){ delete (a); (a) = NULL;}

//Destroy array of pointers to the single object.
#define DESTROY_ARRAYP(a,sz) if((a)!=NULL){\
                                for(int i = 0; i < (sz); ++i){ \
                                   if((a)[i] != NULL) delete (a)[i];}\
                                delete[] (a); (a) = NULL;}

#define NULL_ARRAY(a) (a = NULL)

#define ARRAY_COPY(to, from, n) {int i; for(i=0;i<(n);++i) to[i] = from[i];}

#define CLONE(a) ((a)==NULL ? NULL:(a)->clone())

#define ARRAY_RESIZE(a, CLASS, NDATA, SZ) if((a) == NULL){(a)=new CLASS[SZ];}\
else{ CLASS* tmp = new CLASS[SZ];\
      memcpy(tmp,(a),NDATA*sizeof(CLASS)); delete[] a; a = tmp;}

#define DUMP_INT_ARRAY(name, a,n) {printf("%s = [",name);\
for(int ii=0; ii < (n); ++ii){ printf("%d ",a[ii]);}\
printf("];\n");}

typedef int DiagMatrixType;
inline int min(int a, int b){return a<b?a:b;}
inline int max(int a, int b){return a>b?a:b;}
inline double min(double a, double b){ if(a<b) return a; else return b;}
inline double max(double a, double b){ if(a>b) return a; else return b;}

unsigned int sqrt(unsigned int a);

void print_rcov(FILE *f, const char* s, const RobotPoseCov &rcov);
void print_rcov(const char* s, const RobotPoseCov &rcov);

void print_obs(FILE *f, const char* s, const Gaussian2d &mmm);
void print_obs(const char* s, const Gaussian2d &mmm);


void sample_cumsum(int *sample_out, 
                   const double *cumsum, int np,
   	           int nsample);

void sample(int *sample_out, 
            const double *w, int np,
   	    int nsample);

void shuffle_ind(int *deck, int np, int nSwaps);

void sample_log(int *sample_out, 
                const double *w_log, int np,
   	        int nsample);


void normalise(double *w_out, const double *w, int np);

void weightedMean(RobotPoseCov *out,
                  const RobotPose *, const double *w, int np);

void robotPoseMean(RobotPoseCov *out,
                  const RobotPose *, int np);

double kth_smallest(double *a, int n, int k);
#define median(a,n) kth_smallest(a,n,(((n)&1)?((n)/2):(((n)/2)-1)))

void sortAscending(int *ind_out, const double* data, int n);
void sortDescending(int *ind_out, const double* data, int n);

int binarySearch(int key, const int* data, int n);

inline void print_robot(const char*s, const RobotPose* p){
  if(s != NULL)
    printf("%s = [%e %e %e];\n",s,p->x,p->y,p->rot);
  else
    printf("%e %e %e\n",p->x,p->y,p->rot);
}

inline double md2(double x, double sigma2, double xobs){
  double dx = x - xobs;
  return dx*dx/sigma2;
}

inline double md2_angle(double x, double sigma2, double xobs){
  double dx = angleDiffRad(x,xobs);
  return dx*dx/sigma2;
}

class Location{
public:
  int mapId;
  RobotPose pose;

  Location():mapId(-1){;}
  Location(int mi, const RobotPose &p):mapId(mi),pose(p){;}
  Location(const Location &other):mapId(other.mapId),pose(other.pose){;}
};
extern void location_destructor(GenericStackType t);

class LocationCov{
public:
  int mapId;
  RobotPoseCov pose;

  LocationCov():mapId(-1){;}
  LocationCov(int mi, const RobotPoseCov &p):mapId(mi),pose(p){;}
  LocationCov(const LocationCov &other):mapId(other.mapId),pose(other.pose){;}
};

extern void locationCov_destructor(GenericStackType t);


class UpperTriangularMatrix{
 private:
  int *dat;
  int **matrix;
  int sz_dat;
  int n;

 public:
  UpperTriangularMatrix():dat(NULL),matrix(NULL),sz_dat(0),n(0){;}

  UpperTriangularMatrix(int N):
              dat(NULL),matrix(NULL),sz_dat(0),n(0){
    resize(N);
  }

  UpperTriangularMatrix(int N, int v):
              dat(NULL),matrix(NULL),sz_dat(0),n(0){
    resize(N,v);
  }

  UpperTriangularMatrix(const UpperTriangularMatrix& o):
              dat(NULL),matrix(NULL),sz_dat(0),n(0){
    set(o);
  }

  const UpperTriangularMatrix & operator=(const UpperTriangularMatrix& o){
    if(this != &o){
      set(o);
    }
    return *this;
  }

  ~UpperTriangularMatrix(){ destroy(); }

  int& operator()(int r, int c)const{
    if(c > r)
      return matrix[c][r];
    else
      return matrix[r][c];
  }

  int get(int r, int c)const{
    if(c > r)
      return matrix[c][r];
    else
      return matrix[r][c];
  }

  int N()const{ return n;}

  void resize(int n);
  void resize(int n, int v);
  void set(const UpperTriangularMatrix &other);

  void removeRowCol(int r);

  void destroy(){
    if(sz_dat > 0){
      free(dat);    dat    = NULL;
      free(matrix); matrix = NULL;
      sz_dat = n = 0;
    }
  }
};

class DiagonalMatrix{

 public:
  DiagonalMatrix(){ 
    n = d_sz = 0; dat = NULL; col2dat = NULL; 
  }

  DiagonalMatrix(int sz){
    n = d_sz = 0; dat = NULL; col2dat = NULL; 
    resize(sz);
  }

  DiagonalMatrix(int sz, DiagMatrixType v){
    n = d_sz = 0; dat = NULL; col2dat = NULL; 
    resize(sz,v);
  }

  DiagonalMatrix(const DiagonalMatrix &other){
    set(other);
  }

  const DiagonalMatrix operator=(const DiagonalMatrix &other){
    if(this != &other){
      resize(0);
      set(other);
    }
    return *this;
  }

  ~DiagonalMatrix(){ resize(0);}

  void resize(int sz);
  //Resize to new size "sz" if expanding,
  //set new elements to "v"
  void resize(int sz, DiagMatrixType v);

  DiagMatrixType & operator()(int r, int c){
    if(c > r)
      return dat[col2dat[c] + r];
    else
      return dat[col2dat[r] + c];
  }

  const DiagMatrixType & get(int r, int c) const{
    if(c > r)
      return dat[col2dat[c] + r];
    else
      return dat[col2dat[r] + c];
  }

  void set(DiagMatrixType v){
    for(int i=0;i < d_sz; ++i) dat[i] = v;
  }

 private:
  DiagMatrixType *dat;
  int n;
  int d_sz;
  int *col2dat;

  void set(const DiagonalMatrix &other);
};

class FIFO{

 public:
  FIFO(){
    dat = NULL;
    sz = 0;
    head = tail = 0;
    full = false;
  }

  FIFO(int SZ){
    sz = SZ;
    dat = new int[sz];
    head = tail = 0;
    full = false;
  }

  ~FIFO(){ destroy();}

  void add(int v){
    dat[tail] = v;
    tail = (tail + 1) % sz;
    full = (tail == head);
  }

  void putBack(int v){
    head -= 1;
    if(head < 0) head += sz;
    dat[head] = v;
    full = (tail == head);
  }

  int remove(){
    int res = dat[head];

    head = (head + 1) % sz;

    full = false;
    return res;
  }

  int getHead()const{ return dat[head];}

  int getTail()const{ 
    int i = tail-1; if(i < 0) i += sz;
    return dat[i]; 
  }

  bool isEmpty()const{ if(full) return false; else return head == tail;}
  bool isFull()const{ return full;}

  void dump(FILE* f=stdout)const{
    int i;
    i = head;
    fprintf(f,"%d", dat[i]);

    for(i = (head+1)%sz; i != tail; i = (i+1)%sz ){
      fprintf(f," %d", dat[i]);
    }
  }

  void reset(){
    head = tail = 0;
    full = false;
  }

 private:
  int *dat;
  int sz;
  int head;
  int tail;
  bool full;

  void destroy(){ if(dat != NULL) delete[] dat;}

};

class FILO{
 private:
  int *dat;
  int sz;
  int ind;

 public:
  FILO():dat(NULL),sz(0),ind(-1){;}

  FILO(int SZ):sz(SZ),ind(-1){
    dat = new int[sz];
  }

  FILO(const FILO &o):sz(o.sz),ind(o.ind){
    if(sz > 0){
      dat = new int[sz];
      memcpy(dat, o.dat, sz*sizeof(int));
    }
  }

  const FILO & operator=(const FILO& o){
    if(this != &o){
      if(dat != NULL) delete[] dat;
      sz = o.sz;
      ind = o.sz;

      if(sz > 0){
	dat = new int[sz];
	memcpy(dat, o.dat, sz*sizeof(int));
      }
    }
    return *this;
  }

  ~FILO(){ if(dat != NULL) delete[] dat;}

  void add(int v){
    ind += 1;
    //    printf("Add: [%d] = %d  (sz %d)\n", ind, v, sz);

    dat[ind] = v;
  }

  int pop(){
    return dat[ind];
  }

  int remove(){
    int v = dat[ind];
    ind -= 1;
    return v;
  }

  int operator[](int i)const{ return dat[i];}

  void getAll(int* out)const{
    int i;
    for(i = 0; i <= ind; ++i) out[i] = dat[i];
  }

  int nElements()const{ return ind + 1; }

  bool isFull()const{ return ind >= sz; }
  bool isEmpty()const{ return ind < 0;}

  void dump(FILE* f = stdout)const{
    int i;
    for(i = 0; i <= ind; ++i) fprintf(f," %d",dat[i]);
  }

  void reset(){ ind = -1; }
};

class RunningAverage{
 private:
  double *data;
  int sz;
  int tail;

 public:
  RunningAverage():data(NULL),sz(0),tail(0){;}
  RunningAverage(int n):sz(n), tail(0){
    if(sz > 0){
      data = new double[sz];
      memset(data, 0, sz*sizeof(double));
    }else{
      data = NULL;
    }
  }

  RunningAverage(const RunningAverage &other){
    set(&other);
  }

  const RunningAverage & operator=(const RunningAverage &other){
    if(this != &other){
      DESTROY(data);
      set(&other);
    }
    return *this;
  }

  ~RunningAverage(){DESTROY_ARRAY(data);}

  void set(const RunningAverage*other){
    sz = other->sz;
    tail = other->tail;
    if(sz > 0){
      data = new double[sz];
      memcpy(data, other->data, sz*sizeof(double));
    }else{
      data = NULL;
    }
  }

  void add(double v){
    data[tail] = v;
    tail = (tail + 1)%sz;
  }

  void reset(){
    if(data != NULL){ 
      memset(data, 0, sz*sizeof(double));
      tail = 0;
    }
  }

  double average()const{
    double sum = 0;
    int i;
    for(i =0; i < sz; ++i){
      sum += data[i];
    }
    return sum/sz;
  }

  double average(const double *w)const{
    double sum = 0;
    int i;

    for(i = 0; i < sz; ++i){
      int k = (tail + i)%sz;
      sum += data[k]*w[i];
    }
    return sum;
  }

};

//            1                         1
// y = ---------------- % Default: -------------
//     1 + exp(b - a*x)             1 + exp(-x)
//
class Sigmoid{
 private:
  double a,b;

 public:
  Sigmoid():a(1),b(0){;}

  Sigmoid(double x1, double y1, double x2, double y2){
    set(x1,y1,x2,y2);
  }

  void set(double x1, double y1, double x2, double y2){
    a = log(y1*(y2-1)/(y2*(y1-1)))/(x1-x2);
    b = a*x1 + log((1-y1)/y1);
  }

  double operator()(double x)const{
    return 1/(1 + exp(b - a*x));
  }
};

// y = a*exp(b*x + c) % Defaults to y = exp(x)
class Exponent{
 private:
  double a,b,c;

 public:
  Exponent():a(1),b(1),c(0){;}
  Exponent(double aa, double bb, double cc):a(aa),b(bb),c(cc){;}

  void set(double aa, double bb, double cc){
    a = aa; b  = bb; c = cc; }

  double operator()(double x)const{ return a*exp(b*x + c);}
};

/////////////////////////////////////////////////////////////////
// Inlines
////////////////////////////////////////////////////////////////
inline void DiagonalMatrix::set(const DiagonalMatrix &other){
  n = other.n;
  d_sz = other.d_sz;

  if( n == 0){
    col2dat = NULL;
    dat     = NULL;
  }else{
    col2dat = (int*) malloc(n*sizeof(int));
    memcpy(col2dat, other.col2dat, n*sizeof(int));

    dat = (DiagMatrixType*) malloc(d_sz*sizeof(DiagMatrixType));
    memcpy(dat, other.dat, d_sz*sizeof(DiagMatrixType));
  }
}

inline void DiagonalMatrix::resize(int sz){
  if(sz <= 0){
    free(dat); 
    free(col2dat);
    n = 0; dat = NULL; col2dat = NULL;
  }else{
  
    col2dat = (int*) realloc(col2dat, sz*sizeof(int));

    int i;

    col2dat[0] = -1;

    for(i = max(n,1); i < sz; ++i){
      col2dat[i] = col2dat[i-1] + i;
      //      printf("col2dat[%d]: %d\n",i,col2dat[i]);
    }

    n = sz;

    d_sz = col2dat[n-1] + n;
    dat  = (DiagMatrixType*)realloc(dat, d_sz*sizeof(DiagMatrixType));
  }
}

inline void DiagonalMatrix::resize(int sz, DiagMatrixType v){
  if(sz <= 0){
    free(dat); 
    free(col2dat);
    n = 0; dat = NULL; col2dat = NULL;
  }else{
  
    col2dat = (int*) realloc(col2dat, sz*sizeof(int));

    int i;

    col2dat[0] = -1;

    for(i = max(n,1); i < sz; ++i){
      col2dat[i] = col2dat[i-1] + i;
      //      printf("col2dat[%d]: %d\n",i,col2dat[i]);
    }

    n = sz;

    int d_sz_old = d_sz;
    d_sz = col2dat[n-1] + n;
    dat  = (DiagMatrixType*)realloc(dat, d_sz*sizeof(DiagMatrixType));

    //Set new elements to v
    for(i = d_sz_old; i < d_sz; ++i) dat[i] = v;
  }
}


inline int find_max(const double *a, int n){
  register double max = a[0];
  register int imax = 0;
  register int i;

  for(i = 1; i < n; ++i){
    if(a[i] > max){
      imax = i;
      max = a[i];
    }
  }

  return imax;
}

inline void print_rcov(FILE *f, const char* s, const RobotPoseCov &rcov){
  if(s != NULL)
  fprintf(f,"%s = [%.5e,%.5e,%.5e"
         ",%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e];\n"
	 ,s
	 ,rcov.x, rcov.y, rcov.rot
	 ,rcov.cov[0][0] 
	 ,rcov.cov[0][1] 
	 ,rcov.cov[0][2] 
	 ,rcov.cov[1][0] 
	 ,rcov.cov[1][1] 
	 ,rcov.cov[1][2] 
	 ,rcov.cov[2][0] 
	 ,rcov.cov[2][1] 
	 ,rcov.cov[2][2] );
  else
  fprintf(f,"%.5e,%.5e,%.5e"
         ",%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e\n"
	 ,rcov.x, rcov.y, rcov.rot
	 ,rcov.cov[0][0] 
	 ,rcov.cov[0][1] 
	 ,rcov.cov[0][2] 
	 ,rcov.cov[1][0] 
	 ,rcov.cov[1][1] 
	 ,rcov.cov[1][2] 
	 ,rcov.cov[2][0] 
	 ,rcov.cov[2][1] 
	 ,rcov.cov[2][2] );
}

inline void print_rcov(const char* s, const RobotPoseCov &rcov){
  print_rcov(stdout,s,rcov);
}

inline void print_obs(FILE *f, const char* s, const Gaussian2d &mmm){
  fprintf(f,"%s = [%.5e,%.5e,%.5e,%.5e,%.5e,%.5e];\n"
	 ,s
	 ,mmm.x, mmm.y
	 ,mmm.cov[0][0]
	 ,mmm.cov[0][1]
	 ,mmm.cov[1][0]
	 ,mmm.cov[1][1]);
}

inline void print_obs(const char* s, const Gaussian2d &mmm){
  print_obs(stdout,s,mmm);
}

class SLAMParticle;

bool matlabDump_odo(FILE* f, const char *var_prefix
		    , const SLAMParticle *const *p
		    , int n_p);
bool matlabDump_map(FILE *f, const char* var_name
		    , const SLAMParticle *const *p, int n_p);

inline double fuzzyOR(double a, double b){
  return a + b - a*b;
}
inline double fuzzyOR(double a, double b, double c){
  return  c + b + a - b*c - a*c - a*b + a*b*c;
}

inline double fuzzyRound(double a, double b=0.5){ return (a > b ? 1 : 0);}

#endif




