#ifndef HOUGH_H
#define HOUGH_H

#include "geometry.h"

extern const double DEFAULT_MINTHETA;
extern const double DEFAULT_MAXTHETA;
extern const double DEFAULT_DTHETA;


class Hough{
 private:
  double MINTHETA;
  double MAXTHETA;
  double DTHETA;

  double MINDIST;
  double MAXDIST;
  double DDIST;

  int m_rows;
  int m_cols;
  int** m_table;

  void initTable();
  void destroyTable();
  Point *prune(const Point* point,int &size);

 public:
  Hough();
  Hough(double dmin, double dmax, double dstep,
	double tmin  = DEFAULT_MINTHETA, 
	double tmax  = DEFAULT_MAXTHETA, 
	double tstep = DEFAULT_DTHETA);

  ~Hough(){destroyTable();}

  int rows()const {return m_rows;}
  int cols()const {return m_cols;}

  int **table(){return m_table;}
  operator int **(){return m_table;}

  void reset();
  void compute(const Point *points,int size);
  void add(double x1, double y1, double x2, double y2);

  double theta(int col)const{
    return MINTHETA + (col+0.5)*DTHETA;
  }

  double distance(int row)const{
    return MINDIST + (row+0.5)*DDIST;
  }

  int findmax(double *a, double *r)const;

  void debug_dump()const;

  void setBounds(double dmin, double dmax, double dstep,
		 double tmin  = DEFAULT_MINTHETA, 
		 double tmax  = DEFAULT_MAXTHETA, 
		 double tstep = DEFAULT_DTHETA);


};

#endif //HOUGH_H

