
/* simplex.cc - code file for simplex solution routine */

/*
MODIFICATION HISTORY
====================
01a,31dec97,dja  created
*/

//#include <iostream.h>
#include <math.h>
#include <stdlib.h>
#include "simplex.h"

/* defines */

#define NMAX 500000
#define SWAP(a,b) {swap = (b); (b) = (a); (a) = swap;}

typedef double DATA_TYPE; //KK:03-09-04
#define TRUE true

/*****************************************************************************/

DATA_TYPE amotry(DATA_TYPE **p, DATA_TYPE *y, DATA_TYPE *psum, int ndim,
		 SystemInterface *s, int ihi, DATA_TYPE factor)
{
  int j;
  DATA_TYPE fac1, fac2, ytry, *ptry;

  ptry = new DATA_TYPE[ndim];

  fac1 = (1.0 - factor) / ndim;
  fac2 = fac1 - factor;

  for (j = 0; j < ndim; j++)
    ptry[j] = psum[j] * fac1 - p[ihi][j] * fac2;

#ifdef SIMPLEX_DEBUG
  cout << "ptry = ";
  for (j = 0; j < ndim; j++)
    cout << ptry[j] << " ";
  cout << endl;
#endif

  ytry = s->Eval(ptry);
#ifdef SIMPLEX_DEBUG
  cout << "ytry = " << ytry << endl;
#endif

  if (ytry < y[ihi])
    {
      y[ihi] = ytry;
      for (j = 0; j < ndim; j++)
	{
	  psum[j] += ptry[j] - p[ihi][j];
	  p[ihi][j] = ptry[j];
	}
    }

  delete[] ptry;

  return ytry;
}

/*****************************************************************************/

void amoeba(DATA_TYPE **p, DATA_TYPE *y, int ndim, DATA_TYPE tol,
	    DATA_TYPE ftol, SystemInterface *s, int &nfunk, int maxfunc)
{
  int i, j;
  int ihi; // index of the highest point
  int ilo; // index of the lowest point
  int inhi; // index of the second highest point
  DATA_TYPE rtol, sum, ysave, ytry;
  DATA_TYPE swap;
  DATA_TYPE *psum;
  int it;
  int stalled = 0;
  DATA_TYPE last = 1;

  psum = new DATA_TYPE[ndim];
  it = 0;

  // GET_PSUM
  for (j = 0; j < ndim; j++)
    {
      sum = 0.0;
      for (i = 0; i < ndim + 1; i++)
	sum += p[i][j];
      psum[j] = sum;
    }

  while (TRUE)
    {
      // First find the highest, second highest and lowest points
      ilo = 0;
      if (y[0] > y[1])
	{
	  ihi = 0;
	  inhi = 1;
	}
      else
	{
	  ihi = 1;
	  inhi = 0;
	}
      for (i = 1; i < ndim + 1; i++)
	{
	  // Maintain the lowest point
	  if (y[i] <= y[ilo])
	    ilo = i;
	  // Maintain the high points
	  if (y[i] > y[ihi])
	    {
	      inhi = ihi;
	      ihi = i;
	    }
	  else if ((y[i] > y[inhi]) && (i != ihi))
	    {
	      inhi = i;
	    }
	}

      if ((fabs(y[ilo] - last) / last) > tol)
	stalled = 0;
      else
	stalled++;
      //      cout << stalled << " " << (fabs(y[ilo] - last) / last) * 100000.0 << endl;
      last = y[ilo];
#ifdef SIMPLEX_DEBUG
#if 0
      for (i = 0; i < ndim + 1; i++)
	{
	  cout << i << "  :";
	  for (j = 0; j < ndim; j++)
	    cout << p[i][j] << " ";
	  cout << " = " << y[i] << endl;
	}
#endif
      cout << "ilo = " << ilo << " " << y[ilo] << endl;
      cout << "ihi = " << ihi << " " << y[ihi] << endl;
      cout << "inhi = " << inhi << " " << y[inhi] << endl;
#endif


      // Compute the fractional range from the highest to lowest points
      rtol = 2.0 * fabs(y[ihi] - y[ilo]) / (fabs(y[ihi]) + fabs(y[ilo]));
#if 0
      if (it % 100 == 0)
	{
       	  cout << y[ilo] << endl;
	}
#endif
      it++;

      // Return if the range is small enough
      if ((stalled > 1000) || (rtol < tol) || (y[ilo] < ftol) ||
	  (nfunk > maxfunc))
	{
	  SWAP(y[0], y[ilo]);
	  for (j = 0; j < ndim; j++)
	    SWAP(p[0][j], p[ilo][j]);
	  break;
	}

#ifdef SIMPLEX_DEBUG
      cout << "rtol = " << rtol << endl;
#endif

      nfunk += 2;
      // Begin a new iteration
      ytry = amotry(p, y, psum, ndim, s, ihi, -1.0);
      if (ytry <= y[ilo])
	{
#ifdef SIMPLEX_DEBUG
	  cout << "factor of two \n";
#endif
	  // If the trial gives better than the current best point
	  // try an extrapolation by a factor of two
	  ytry = amotry(p, y, psum, ndim, s, ihi, 2.0);
	}
      else if (isnan(ytry) || (ytry >= y[inhi]))
	{
#ifdef SIMPLEX_DEBUG
	  cout << "intermediate \n";
#endif
	  // The reflected point is worse than the second-highest
	  // so look for an intermediate, lower point i.e. try a
	  // one-dimensional contraction
	  ysave = y[ihi];
	  ytry = amotry(p, y, psum, ndim, s, ihi, 0.5);
	  if (isnan(ytry) || (ytry >= ysave))
	    {
	      // Can't seem to get rid of that high point
	      // try a contraction about the best point
#ifdef SIMPLEX_DEBUG
	      cout << "contract n \n";
#endif
	      for (i = 0; i < ndim + 1; i++)
		{
		  if (i != ilo)
		    {
		      for (j = 0; j < ndim; j++)
			p[i][j] = 0.5 * (p[i][j] + p[ilo][j]);
		      y[i] = s->Eval(p[i]);
		    }
		}
	      nfunk += ndim;

	      // GET_PSUM
	      for (j = 0; j < ndim; j++)
		{
		  for (sum = 0.0, i = 0; i < ndim; i++)
		    sum += p[i][j];
		  psum[j] = sum;
		}
	    }
	  else
	    nfunk--;
	}
      else if (ytry > y[ihi])
	{
	  //	  cout << "spammed " << ytry << " " << y[ihi] << endl;
	  break;
	}
    }

  delete[] psum;
}

/*****************************************************************************/

int SimplexMethod(DATA_TYPE* out, SystemInterface *s)
{
  int nfunk; // Number of function calls
  int i, j;
  DATA_TYPE by;
  int retries;
  int ndim;
  DATA_TYPE **p;
  DATA_TYPE *y;

  ndim = s->GetNumVars();
  //  s->SetWeights();

  // Allocate some space for working
  p = new DATA_TYPE *[ndim + 1];
  y = new DATA_TYPE[ndim + 1];
  for (i = 0; i < ndim + 1; i++)
    p[i] = new DATA_TYPE[ndim];

  // Initialise the simplex with the current variable values
  for (i = 0; i < ndim; i++)
    p[0][i] = s->getVar(i);

  // Initialise the simplex with a basis set
  for (i = 1; i < ndim + 1; i++)
    for (j = 0; j < ndim; j++)
      if (i - 1 == j)
	p[i][j] = p[0][j] * 1.1;
      else
	p[i][j] = p[0][j];

  // Evaluate at the initial points
  for (i = 0; i < ndim + 1; i++)
    {
      y[i] = s->Eval(p[i]);
      //cout << i << " y[i] = " << y[i] << endl;
    }
  nfunk = ndim;

  // Run the algorithm to try to get close
  amoeba(p, y, ndim, 1.0e-5, 1.0e-10, s, nfunk, NMAX);

  retries = 0;
  while ((retries < 9) && (y[0] > 1.0e-15))
    {
      // Restart the algorithm

      // Re-initialise the simplex
      for (i = 1; i < ndim + 1; i++)
	for (j = 0; j < ndim; j++)
	  p[i][j] = p[0][j] * pow(1.1,(retries + 1)) *
	    (((DATA_TYPE) random()) / RAND_MAX); 
	  
      // Re-evaluate at the new initial points
      for (i = 0; i < ndim + 1; i++)
	{
	  y[i] = s->Eval(p[i]);
	}

      by = y[0];

#ifdef SIMPLEX_DEBUG_AMOEBA
      cout << "retries = " << retries << endl;
      cout << "by = " << by << " y[0] = " << y[0] << endl;
#endif

      amoeba(p, y, ndim, 1.0e-5, 1.0e-10, s, nfunk, NMAX);

#ifdef SIMPLEX_DEBUG_AMOEBA
      cout << "by = " << by << " y[0] = " << y[0] << " diff = " 
	   << by - y[0] << endl;
#endif

      if (fabs(by - y[0]) / fabs(by) < 1.0e-5)
	retries++;
      else
	retries = 0;
    }

#if 0
  cout << "Solution is " << s->Eval(p[0]) << " obtained with " 
       << nfunk << " evaluations"
       << endl;
#endif

  //Copy the output
  for(i = 0; i < ndim; ++i) out[i] = p[0][i];

  for (i = 0; i < ndim + 1; i++)
    delete[] p[i];
  delete[] p;
  delete[] y;

  return nfunk;
}





