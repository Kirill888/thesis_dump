#ifndef __SIMPLEX_H__
#define __SIMPLEX_H__


class SystemInterface{
 public:
  virtual double Eval(double *){return 0;}

  virtual int GetNumVars(){return 0;}

  virtual double getVar(int i){return 0;}
};


void amoeba(double **p, double *y, int ndim, double tol,
	    double ftol, SystemInterface *s, int &nfunk, int maxfunc);

double amotry(double **p, double *y, double *psum, int ndim,
	      SystemInterface *s, int ihi, double factor);

int SimplexMethod(double* out, SystemInterface *s);


#endif
