#ifndef __ICP_H__
#define __ICP_H__

#ifndef DEPEND
#include <stdio.h>
#endif

#include "kdtree.h"

class LaserScan{
 public:
  unsigned int n;
  double *x;
  double *y;

  double centroid[2];
  double bounds[2][2];

  KDTree *tree;

  LaserScan():n(0),x(NULL),y(NULL),tree(NULL){;}

  //Warning: does NOT copy arrays, keeps pointers, then delete[] them
  LaserScan(double *xx, double *yy, int N):n(N),x(xx),y(yy),tree(NULL){;}

  ~LaserScan(){
    if(x != NULL)    delete[] x;
    if(y != NULL)    delete[] y;
    if(tree != NULL) delete tree;
  }
  void destroy(){
    if(x != NULL)    delete[] x;
    if(y != NULL)    delete[] y;
    if(tree != NULL) delete tree;
    x = y = NULL;
    tree = NULL;
  }

  void init(unsigned int N){
    n = N;
    x = new double[n];
    y = new double[n];
  }

  void computeCentroid();
  void buildTree();

  //translates the points and the centroid
  //  makes tree and bounds invalid
  void translateMe(double xc, double yc, double ca, double sa);

  void matlabDump(FILE*f, const char* var);

};

class ICPScanMatch{
 private:
  unsigned int *closest;

  unsigned int n1, n2;
  const double *x1;
  const double *y1;
  const double *x2;
  const double *y2;

  double maxDist2;
  //Compute MSE: finds point correspondences
  //             computes MSE
  //             Output: return value -- MSE
  //                     nn_changed -- true if data association changed
  //                                   false if remained the same
  double compute_mse(const KDTree* refScan, bool *nn_changed, int *np);

  void icp_step(double *step);

 public:

  ICPScanMatch();
  ~ICPScanMatch();

  int match(const LaserScan &scan1, const LaserScan &scan2
	    , double mse_tol, double mse_imp_per_iter
            , double *step, const double *odo0);

  void setMaxDist(double dist2){ maxDist2 = dist2*dist2;}
  void setMaxDist2(double md2){ maxDist2 = md2; }
};

#endif
