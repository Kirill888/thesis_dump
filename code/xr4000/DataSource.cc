#ifndef DEPEND
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#endif

#include "DataSource.h"
#include "debug.h"

void run_simulation(OdoSource &odo
                   , void (*odoEvent)(const OdoSource &odo)
                   , LaserSource &las
                   , void (*lasEvent)(const LaserSource &odo)
                   , const bool *STOP){
   
  bool bMoreOdo;
  bool bMoreLas;

  bMoreOdo = odo.next();
  bMoreLas = las.nextScan();

  while(!(*STOP) && bMoreOdo && bMoreLas){

    while(bMoreOdo && odo.TimeStamp() < las.TimeStamp()){
      odoEvent(odo);
      bMoreOdo = odo.next();
    }

    while(bMoreLas && las.TimeStamp() < odo.TimeStamp()){
      lasEvent(las);
      bMoreLas = las.nextScan();
    }
  }

  if(!(*STOP) && bMoreOdo){
    odoEvent(odo);
    while(odo.next()) odoEvent(odo);
  }

  if(!(*STOP) && bMoreLas){
    lasEvent(las);
    while(las.nextScan()) lasEvent(las);
  }

}


//==========================================================
// LaserSource
//==========================================================
void LaserSource::set(const LaserSource* other){
  if( other->npoints  != npoints ){
    destroy();

    npoints = other->npoints;

    if( npoints == 0){
      xx = yy = rr = aa = NULL;      
      ii = NULL;
      return;
    }else{
      rr = new double[npoints];
      aa = new double[npoints];
      yy = new double[npoints];
      xx = new double[npoints];

      ii = new int[npoints];
    }
  }

  if(npoints == 0) return;

  memcpy(rr, other->rr, npoints*sizeof(double) );
  memcpy(aa, other->aa, npoints*sizeof(double) );
  memcpy(xx, other->xx, npoints*sizeof(double) );
  memcpy(yy, other->yy, npoints*sizeof(double) );

  memcpy(ii, other->ii, npoints*sizeof(int) );

  tt = other->tt;
  maxRange = other->maxRange;
}

void LaserSource::setData(double *r,double *a, int* intensity,
                          int n, double t){
  int i;

  tt = t;

  if( n != npoints){
    destroy();

    npoints = n;

    if( n == 0){
      xx = yy = rr = aa = NULL;
      ii = NULL;
      npoints = 0;
      return;
    }else{
      rr = new double[npoints];
      aa = new double[npoints];
      yy = new double[npoints];
      xx = new double[npoints];

      ii = new int[npoints];
    }
  }

  if(n == 0) return;

  memcpy(rr,r,n*sizeof(double));
  memcpy(aa,a,n*sizeof(double));

  if( intensity != NULL) {
    memcpy(ii,intensity,n*sizeof(int));
  }else{
    memset(ii,0,n*sizeof(int));
  }


  //Compute x,y coords
  for(i = 0; i < n; ++i){
    double ca = cos(aa[i]);
    double sa = sin(aa[i]);

    xx[i] = rr[i]*ca;
    yy[i] = rr[i]*sa;
  }
}

//==========================================================
// FileLaserSource
//==========================================================
const int NUM_POINTS_PER_SCAN = 361;
const double ANGLE_STEP = 0.5*M_PI/180.0;
const int LASER_FILE_LINE_LENGTH = 4*1024;

FileLaserSource::FileLaserSource(char *fileName, double maxR, bool no_eol){
  f = fopen(fileName,"r");

  maxRange = maxR;

  npoints = NUM_POINTS_PER_SCAN;

  rr = new double[npoints];
  aa = new double[npoints];
  yy = new double[npoints];
  xx = new double[npoints];

  cos_aa = new double[npoints];
  sin_aa = new double[npoints];

  ii = new int[npoints];
  memset(ii,0,npoints*sizeof(int));

  for(int i=0;i < npoints; ++i){
    aa[i] = i*ANGLE_STEP;

    cos_aa[i] = cos(aa[i]);
    sin_aa[i] = sin(aa[i]);
  }

  noEOL = no_eol;
}

FileLaserSource::~FileLaserSource(){
  if(f != NULL) fclose(f);

  DESTROY_ARRAY(cos_aa);
  DESTROY_ARRAY(sin_aa);
}

static const double LAS_MIN_RANGE  = 0.001;
static const double LAS_NOT_AVALUE = NaN;

bool FileLaserSource::nextScan_noEOL(){
  int i;
  if(fscanf(f,"%lf",&tt) != 1) return false;

  for(i = 0; i < npoints; ++i){
    if(fscanf(f,"%lf", rr + i) != 1) return false;

    xx[i] = rr[i]*cos_aa[i];
    yy[i] = rr[i]*sin_aa[i];
  }

  return true;
}

bool FileLaserSource::nextScan(){

  if(noEOL) return nextScan_noEOL();

  int i;

  if( f == NULL) return false;

  char buf[LASER_FILE_LINE_LENGTH];
  char *token;

  if(fgets(buf,LASER_FILE_LINE_LENGTH,f) != NULL){
    token = strtok(buf,"\t ");

    if(token == NULL) return false;
      
    tt = atof(token);

    for(i = 0, token = strtok(NULL,"\t ");
	i < npoints && token != NULL;
	++i, token = strtok(NULL,"\t ")){

      rr[i] = atof(token);

      if(rr[i] >= LAS_MIN_RANGE){
	if(rr[i] < maxRange){
	  xx[i] = rr[i]*cos_aa[i];
	  yy[i] = rr[i]*sin_aa[i];
	}else{
	  xx[i] = NaN;
	  yy[i] = NaN;
	}
      }else{
	rr[i] = LAS_NOT_AVALUE;
        xx[i] = NaN;
	yy[i] = NaN;
      }

    }

    if( i != npoints){
      DEBUG_error("Incomplete Laser Range Data");
      return false;
    }

    for(i = 0;
	i < npoints && token != NULL;
	++i, token = strtok(NULL,"\t\n ")){
 
      ii[i] = atoi(token);
    }

    if(i != npoints && i != 0){
      DEBUG_error("Incomplete Laser Intensity Data: %d\n",i);
      return false;
    }

    return true;

  }else{
    return false;
  }
}

//============================================================================
// FileOdoSource
//============================================================================

FileOdoSource::FileOdoSource(char *fileName){
  f = fopen(fileName,"r");
}

FileOdoSource::~FileOdoSource(){
  if(f != NULL) fclose(f);
}

bool FileOdoSource::next(){
  if(f == NULL) return false;

  return (fscanf(f,"%lf%lf%lf%lf",&tt,&xx,&yy,&aa) == 4);
}
