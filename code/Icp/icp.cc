#include "icp.h"
#ifndef DEPEND
#include <math.h>
#include <string.h>
#endif

#define POW2(a) ((a)*(a))

#if 1
extern const double Inf;
#else
#define Inf HUGE_VAL;
#endif

//----------------------------------------------------------------
// Laser Scan
//----------------------------------------------------------------
void LaserScan::computeCentroid(){ //And bounds on x,y
  double xx;
  double yy;
  unsigned int i;

  bounds[0][0] = x[0]; bounds[0][1] = x[0];
  bounds[1][0] = y[0]; bounds[1][1] = y[0];

  xx = x[0];
  yy = y[0];

  for (i = 1; i < n; i++) {
    xx += x[i];
    yy += y[i];

    if(x[i] < bounds[0][0]) bounds[0][0] = x[i];
    if(x[i] > bounds[0][1]) bounds[0][1] = x[i];

    if(y[i] < bounds[1][0]) bounds[1][0] = y[i];
    if(y[i] > bounds[1][1]) bounds[1][1] = y[i];
  }

  centroid[0] = xx / n;
  centroid[1] = yy / n;

}

void LaserScan::buildTree(){
  if(tree != NULL) delete tree;

  tree = new KDTree(2);
  double *xy[2] = {x,y};
  int dim;

  double span_x = bounds[0][1] - bounds[0][0];
  double span_y = bounds[1][1] - bounds[1][0];

  if(span_x > span_y) dim = 0;
  else                dim = 1;

  tree->init(xy, n, dim, false);
}

void LaserScan::translateMe(double xc, double yc, double ca, double sa){
  unsigned int i;
  register double xn;

  for(i = 0; i < n; ++i){
    xn   = x[i]*ca - y[i]*sa + xc;
    y[i] = x[i]*sa + y[i]*ca + yc;
    x[i] = xn;
  }

  xn          = centroid[0]*ca - centroid[1]*sa + xc;
  centroid[1] = centroid[0]*sa + centroid[1]*ca + yc;
  centroid[0] = xn;
}

void LaserScan::matlabDump(FILE*f, const char* var){
  unsigned int i;
  fprintf(f,"%s = [ ...\n", var);
  for(i = 0; i < n; ++i){
    fprintf(f,"%.8e %.8e\n",x[i],y[i]);
  }
  fprintf(f,"];\n");
}

//----------------------------------------------------------------
// ICP
//----------------------------------------------------------------

ICPScanMatch::ICPScanMatch(){
  maxDist2 = POW2(0.2);
}

ICPScanMatch::~ICPScanMatch(){
}

void ICPScanMatch::icp_step(double *step){
  unsigned int i;
  double X1,X2,Y1,Y2,xx,yy,xy,yx;
  unsigned int n = 0;

  X1=X2=Y1=Y2=xx=yy=xy=yx=0.0;
  for (i=0; i < n1; ++i) { // calculate sums
    unsigned int j = closest[i];

    if(j != 0xFFFFFFFF){
      X1 += x1[i];
      X2 += x2[j];
      Y1 += y1[i];
      Y2 += y2[j];
      xx += x1[i]*x2[j];
      yy += y1[i]*y2[j];
      xy += x1[i]*y2[j];
      yx += y1[i]*x2[j];
      n += 1;
    }
  }

  double scale = 1.0/double(n);

  double Sxx = xx - X1*X2*scale; // calculate S
  double Syy = yy - Y1*Y2*scale;
  double Sxy = xy - X1*Y2*scale;
  double Syx = yx - Y1*X2*scale;

  double xm1 = X1*scale; // calculate means
  double xm2 = X2*scale;
  double ym1 = Y1*scale;
  double ym2 = Y2*scale;

  double phi = atan2(Sxy-Syx, Sxx+Syy);

  double ca = cos(phi);
  double sa = sin(phi);
  step[0] = xm2 - (xm1*ca - ym1*sa);
  step[1] = ym2 - (xm1*sa + ym1*ca);
  step[2] = phi;
}

double ICPScanMatch::compute_mse(const KDTree * refScan, bool *nn_changed, 
                                 int *np){ 
  double dist2 = 0;
  unsigned int i;

  *nn_changed = false;
  *np = 0;

  for(i = 0; i < n1; ++i){
    register double d2 = Inf;
    double query[2] = {x1[i],y1[i]};

    int id = refScan->findNN(query, &d2);
    //    printf("NN[%d -> %d] %g,%g %g\n", i, id, x1[i],y1[i], d2);

    if(!*nn_changed && closest[i] != (unsigned int) id){
      *nn_changed = true;
    }

    if(d2 <= maxDist2){
      closest[i] = (unsigned int) id;
      dist2 += d2;
      *np = *np + 1;
    }else{
      closest[i] = 0xFFFFFFFF;
    }
  }

  return dist2;
}


// match scan1 to scan2 
//   step -- pose of scan1 relative to scan2
int ICPScanMatch::match(const LaserScan &scan1, const LaserScan &scan2
			, double mse_tol, double mse_imp_per_iter
			, double *step, const double *odo0){

  bool nn_changed;
  double tmpstep[3];
  unsigned int i;
  double mse;
  double mse_old;
  double sa;
  double ca;
  double *tx;
  double *ty;
  int nPoints;
  double tmp;


  n1 = scan1.n;
  tx = new double[n1];
  ty = new double[n1];

  if(odo0 == NULL){
    memcpy(tx, scan1.x, scan1.n*sizeof(double));
    memcpy(ty, scan1.y, scan1.n*sizeof(double));
    step[0] = step[1] = step[2] = 0;
  }else{
    memcpy(step, odo0, 3*sizeof(double));
    ca = cos(step[2]);
    sa = sin(step[2]);

    for(i = 0; i < n1; ++i){
      tx[i] = step[0] + ca*scan1.x[i] - sa*scan1.y[i];
      ty[i] = step[1] + sa*scan1.x[i] + ca*scan1.y[i];
    }
  }

  x1 = tx; y1 = ty;
  x2 = scan2.x; y2 = scan2.y; n2 = scan2.n;

  closest = new unsigned int[n1];
  //memset(closest,-1, sizeof(int)*n1); //Make valgrind happy

  //Compute closest and MSE
//   printf("Compute NN 1st:\n");
  mse = compute_mse(scan2.tree , &nn_changed, &nPoints);
  //  if(mse < 0) return -1;

  do{
 
    icp_step(tmpstep);

    //Add step to prev step
    ca = cos(tmpstep[2]);
    sa = sin(tmpstep[2]);
    tmp      = tmpstep[0] + ca*step[0] - sa*step[1];
    step[1]  = tmpstep[1] + sa*step[0] + ca*step[1];
    step[0]  = tmp;
    step[2] += tmpstep[2];

    //Translate scan1
    for(i = 0; i < n1; ++i){
      tmp   = tmpstep[0] + ca*x1[i] - sa*y1[i];
      ty[i] = tmpstep[1] + sa*x1[i] + ca*y1[i];
      tx[i] = tmp;
    }

    //Recompute closest points and mse
    mse_old = mse;
//     printf("Compute NN more:\n");
    mse = compute_mse(scan2.tree, &nn_changed,  &nPoints);

    //if(mse < 0) return -1;
  }while(nn_changed && mse > mse_tol && 
	 (mse / mse_old < mse_imp_per_iter));


  delete[] closest;
  delete[] tx;
  delete[] ty;

  return nPoints;
}
