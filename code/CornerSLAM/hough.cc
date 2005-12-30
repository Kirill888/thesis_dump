#ifndef DEPEND
#include <math.h>
#include <stdio.h>
#endif /* !DEPEND */

#include "hough.h"


#ifdef HOUGH_SMALL_TABLE
const double DEFAULT_MINTHETA = -180 * DEGTORAD;
const double DEFAULT_MAXTHETA = +180 * DEGTORAD;
const double DEFAULT_DTHETA   = + 30 * DEGTORAD;

const double DEFAULT_MINDIST = 0.0;
const double DEFAULT_MAXDIST = 4.0;
const double DEFAULT_DDIST   = 0.5;
#else
const double DEFAULT_MINTHETA = -180 * DEGTORAD;
const double DEFAULT_MAXTHETA = +180 * DEGTORAD;
const double DEFAULT_DTHETA   = +  1 * DEGTORAD;

const double DEFAULT_MINDIST = 0.0;
const double DEFAULT_MAXDIST = 6.0;
const double DEFAULT_DDIST   = 0.01;
#endif


Hough::Hough(){
  m_table = NULL;
  setBounds(DEFAULT_MINDIST , DEFAULT_MAXDIST , DEFAULT_DDIST,
	    DEFAULT_MINTHETA, DEFAULT_MAXTHETA, DEFAULT_DTHETA);
}

Hough::Hough(double dmin, double dmax, double dstep,
             double tmin, double tmax, double tstep){
  m_table = NULL;
  setBounds(dmin,dmax,dstep,
	    tmin,tmax,tstep);
}


void Hough::setBounds(double dmin, double dmax, double dstep,
		      double tmin, double tmax, double tstep){
  MINDIST = dmin;
  MAXDIST = dmax;
  DDIST   = dstep;

  MINTHETA = tmin;
  MAXTHETA = tmax;
  DTHETA   = tstep;

  destroyTable();
  initTable();
}

void Hough::destroyTable(){
  int i;

  if(m_table == NULL) return;

  for(i=0;i<m_rows;++i)
    delete[] m_table[i];

  delete[] m_table;
}

void Hough::initTable(){
  int i,j;


  m_rows = (int)((MAXDIST-MINDIST)  / DDIST)  + 1;
  m_cols = (int)((MAXTHETA-MINTHETA)/ DTHETA) + 1;


  m_table = new int*[m_rows];
  for(i=0;i<m_rows;++i){
    m_table[i] = new int[m_cols];

    for(j=0;j<m_cols;++j) m_table[i][j] = 0;
  }

}

void Hough::reset(){
  int i,j;

  for(i=0;i<m_rows;++i)
    for(j=0;j<m_cols;++j) m_table[i][j] = 0;

}

Point* Hough::prune(const Point* points, int &size){
  int i;
  int n_pruned = 0;

  Point *pruned;
  pruned = new Point[size];

  for(i=0;i<size;++i){
    if(WITHIN_NN(-MAXDIST,points[i].x,MAXDIST) &&
       WITHIN_NN(-MAXDIST,points[i].y,MAXDIST)){
      pruned[n_pruned++] = points[i];
    }
  }

  size = n_pruned;
  return pruned;
}

void Hough::compute(const Point *points,int size){
  int i,j;
#ifdef HOUGH_PRUNE_POINTS
  points = prune(points,size);
#endif

  for(i=0;i<size-1;++i){
    Point p1;
    p1 = points[i];

    for(j=i+1;j<size;++j){
      add(p1.x,p1.y,points[j].x,points[j].y);
    }
  }

#ifdef HOUGH_PRUNE_POINTS
  //If we pruned the points, delete the copy we created.
  delete[] points;
#endif
}

void Hough::add(double x1, double y1, double x2, double y2){
  double dx = x2 - x1;
  double dy = y2 - y1;
  double a;
  double r;

  if( dx == 0){
    if(x1 < 0){      r = -x1;      a = M_PI;
    }else{           r = x1;       a = 0;    }

  }else if(dy == 0){
    if(y1 < 0){      r = -y1;      a =  M_PI/2;
    }else{           r =  y1;      a = -M_PI/2;    }

  }else{
    a = atan(dy/dx) - M_PI/2;
    r = (dx*y1 - dy*x1)/(dx*sin(a) - dy*cos(a));
    if(r < 0){
      r = -r;
      a = a + M_PI;
    }
  }

  int ri,ai;
  ri = int( (r - MINDIST)/DDIST);
  ai = int( (a - MINTHETA)/DTHETA);

  if(WITHIN_EN(0,ri,m_rows) && WITHIN_EN(0,ai,m_cols)){
    m_table[ri][ai] += 1;
  }

}

int Hough::findmax(double *angle, double *dist)const{
  int c,r;
  int c_max = -1;
  int r_max = -1;
  int hits_max = 0;

  for(r = 0; r < m_rows; ++r)
    for(c = 0; c < m_cols; ++c)
      if(m_table[r][c] > hits_max){
	hits_max = m_table[r][c];
	c_max = c; r_max = r;
      }

  if(hits_max > 0){	 
    *angle = theta(c_max);
    *dist  = distance(r_max);
  }

  return hits_max;
}

void Hough::debug_dump() const{
  int i,j;

  printf("Hough Debug Dump\n");
  printf("Table Size is %d rows by %d columns.\n", m_rows, m_cols);

  for(i=0;i<m_rows;++i){
    for(j=0;j<m_cols;++j){
      printf("%d ",m_table[i][j]);
    }
    printf("\n");
  }
}

