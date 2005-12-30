#ifndef DEPEND
#include <math.h>
#endif
#include "random.h"

//Random Number
///////////////////////////////////////////////////////////////////////////////
double GaussianRandomNumber::nextDoubleRandom()const{
  if(m_second){
    m_second = false;
    m_md = m_md2;

    return m_gauss2;
  }

  double x1,x2,w;

  do{
    x1 = 2.0*m_uniRand.nextRandom()-1.0;
    x2 = 2.0*m_uniRand.nextRandom()-1.0;
    w  = x1*x1+x2*x2;
  }while(w >= 1.0);

  w = sqrt((-2.0*log(w))/w);

  m_md1 = x1*w;
  m_md2 = x2*w;
  m_md  = m_md1;

  m_gauss1 = (m_md1)*m_standartDeviation + m_mean;
  m_gauss2 = (m_md2)*m_standartDeviation + m_mean;

  m_second = true;

  return m_gauss1;
}

/* Cholesky decomposition*/
bool choldc(double out[3][3], const double a[3][3])
{
   int i,j,k;
   double sum;
   const int n = 3;

   for (i=0; i < n ; i++)
     for (j=0; j < n ; j++) out[i][j] = 0;

   for (i=0; i < n;i++)
   {
      for (j=i; j < n; j++)
      {
	 for (sum = a[i][j],k=i-1; k >= 0; k--) 
	 {
	   sum -= out[k][i]*out[k][j];
	 }

	 if (i==j)
	 {
	    if (sum <=0.0)
	    {
#if 0
	       printf("choldc: failed. (matrix probably is not "
		     "positive definite)\n");
#endif
	       return false;
	    }
	    out[i][i] = sqrt(sum);
	 }else{
           out[i][j] = sum/out[i][i];
	 }
      }
   }

   return true;
}

/* Cholesky decomposition*/
bool choldc(double out[2][2], const double a[2][2])
{
   int i,j,k;
   double sum;
   const int n = 2;

   for (i=0; i < n ; i++)
     for (j=0; j < n ; j++) out[i][j] = 0;

   for (i=0; i < n;i++)
   {
      for (j=i; j < n; j++)
      {
	 for (sum = a[i][j],k=i-1; k >= 0; k--) 
	 {
	   sum -= out[k][i]*out[k][j];
	 }

	 if (i==j)
	 {
	    if (sum <=0.0)
	    {
#if 0
	       printf("choldc: failed. (matrix probably is not "
		     "positive definite)\n");
#endif
	       return false;
	    }
	    out[i][i] = sqrt(sum);
	 }else{
           out[i][j] = sum/out[i][i];
	 }
      }
   }

   return true;
}

