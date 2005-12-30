#ifndef DEPEND
#include <stdio.h>
#endif /* !DEPEND */

#include "debug.h"
#include "geometry.h"
#include "AngleTools.h"
#include "matrix2.h"
#include "matrix3.h"

const double DEGTORAD = M_PI/180;
const double RADTODEG = 180/M_PI;
const double NaN = NAN;
const double Inf = HUGE_VAL;

const double logPI = log(M_PI);

const double VOL2_FIXED_PARAM1    =  1.0/(2*M_PI);
const double LOGVOL2_FIXED_PARAM1 = -log(2*M_PI);

//Point class
//////////////////////////////////////////////////////////////////////////////

Point Point::translate(double xc,double yc, double ca, double sa)const{ 
  double xn,yn;

  xn = x*ca - y*sa + xc;
  yn = x*sa + y*ca + yc;
  return Point(xn,yn);
}

void Point::translateMe(double xc,double yc, double ca, double sa){ 
  double xn;

  xn = x*ca - y*sa + xc;
  y  = x*sa + y*ca + yc;

  x = xn;
}

void Point::rotateMe(double ca, double sa){
  double xn;

  xn = x*ca - y*sa;
  y  = x*sa + y*ca;

  x = xn;
}

double Point::distance(const Point &p)const{
  double dx = x - p.x;
  double dy = y - p.y;

  return sqrt(dx*dx + dy*dy);
}

double Point::dist2(const Point &p)const{
  double dx = x - p.x;
  double dy = y - p.y;

  return dx*dx + dy*dy;
}

void Point::toPolar(double *radius, double *angle)const{
  *radius = sqrt(x*x + y*y);
  *angle  = atan2(y,x);
}

//Gaussian2d class
//////////////////////////////////////////////////////////////////////////////
Gaussian2d Gaussian2d::translate(double xc, double yc, 
                                       double ca, double sa)const{ 
  double xn,yn;
  double covn[2][2];

  xn = x*ca - y*sa + xc;
  yn = x*sa + y*ca + yc;

  rotate_cov(covn,cov,ca,sa);

  return Gaussian2d(xn,yn,covn);
}

void Gaussian2d::translateMe(double xc,double yc, double ca, double sa){ 
  double xn;

  xn = x*ca - y*sa + xc;
  y  = x*sa + y*ca + yc;

  x = xn;

  rotate_cov(cov,ca,sa);
}

double Gaussian2d::mahalanobis2(double xx, double yy)const{
  double inv[2][2];
  double dx,dy;

  matrix2_inverse(inv,cov);
  dx = x - xx;
  dy = y - yy;
   
  return ( dx*( dx*inv[0][0] + dy*inv[1][0] ) +
	   dy*( dx*inv[0][1] + dy*inv[1][1] ));

}


void Gaussian2d::covIntersect(double c[2][2], double cx[2]
				 ,const double a_inv[2][2],const double ax[2]
				 ,const double b_inv[2][2],const double bx[2]
				 ,double w)const{

  double c_inv[2][2];
  double aux1[2][2];
  double aux2[2][2];

  double v1[2];
  double v2[2];

  //c_inv = w*a_inv + (1-w)*b_inv
  matrix2_multiply(aux1, a_inv, w);
  matrix2_multiply(aux2, b_inv, 1-w);
  matrix2_add(c_inv, aux1, aux2);
  matrix2_inverse(c, c_inv);

  //cx = c*(w*a_inv*ax + (1-w)*b_inv*bx)
  matrix2_multiply(v1, aux1, ax);
  matrix2_multiply(v2, aux2, bx);
  vector2_add(v1, v1, v2);
  matrix2_multiply(cx, c, v1);
  
}

Gaussian2d Gaussian2d::covIntersect(const Gaussian2d &p
					  , double w)const{
  double a_inv[2][2];
  double b_inv[2][2];
  double c[2][2];

  double ax[2];
  double bx[2];
  double cx[2];

  matrix2_inverse(a_inv, cov);
  matrix2_inverse(b_inv, p.cov);

  ax[0] = x;
  ax[1] = y;
  bx[0] = p.x;
  bx[1] = p.y;

  covIntersect(c, cx, a_inv, ax, b_inv, bx, w);

  return Gaussian2d(cx[0], cx[1], c);
}

const double W_STEP = 1.0/20;

double Gaussian2d::probMatch(const Gaussian2d &p)const{
  return volumeOfTheProduct(p,*this);
}

Gaussian2d Gaussian2d::covIntersect(const Gaussian2d &p)const{
  return covIntersect(p,0.5);
}


//
// RobotPose Class
///////////////////////////////////////////////////////////////////////////////

// MISCelanious
///////////////////////////////////////////////////////////////////////////////

double mahalanobis2(const Point &x, const Gaussian2d &obs){
  double inv[2][2];
  double dx,dy;

  matrix2_inverse(inv,obs.cov);
  dx = obs.x - x.x;
  dy = obs.y - x.y;
   
  return ( dx*( dx*inv[0][0] + dy*inv[1][0] ) +
	   dy*( dx*inv[0][1] + dy*inv[1][1] ));

}

inline void gauss2a(double a[6], const Gaussian2d &l){
  double sxx  = l.cov[0][0];
  double syy  = l.cov[1][1];
  double sxsy = sqrt(sxx)*sqrt(syy);
  double p    = l.cov[0][1]/sxsy;
  
  double mx  = l.x;
  double my  = l.y;
  
  double scaler = 1.0/(-2*(1-p*p));
  
  a[1] = scaler/sxx;
  a[2] = scaler/syy;
  a[3] = scaler*(-2*p/sxsy);
  a[4] = scaler*2*(-mx/sxx + p*my/sxsy);
  a[5] = scaler*2*(-my/syy + p*mx/sxsy);
  a[0] = scaler*(mx*mx/sxx + my*my/syy - 2*p*mx*my/sxsy);
  
  a[0] = a[0] - log(2*M_PI*sxsy*sqrt(1 - p*p));

#if 0
  printf("gauss2a: m = [%.3e,%.3e]; S = [%.3e,%.3e; %.3e,%.3e]\na =[ "
         ,mx,my
	 ,l.cov[0][0]
	 ,l.cov[0][1]
	 ,l.cov[1][0]
	 ,l.cov[1][1]
         );
  for(int i = 0; i < 6; ++i)
    printf(",%.5e ",a[i]);
  
  printf("]\n");
#endif

}

inline double volume_a(double a[6]){
  double a1_inv = 1/a[1];

  double aux1 = 0.25*POW2(a[3])*a1_inv - a[2];
  double aux2 = a[5] - 0.5*a[3]*a[4]*a1_inv;

  double vol;

  vol = M_PI*exp(a[0] + 0.25*(-POW2(a[4])*a1_inv + POW2(aux2)/aux1))
        /sqrt(-a[1]*aux1);
 
  return vol;
}

double volumeOfTheProduct(const Gaussian2d &l1, 
                          const Gaussian2d &l2){
  double B[2][2];
  double cov[2][2];

  matrix2_add(cov,l1.cov,l2.cov);

  double det = matrix2_det(cov);

  //B = (Cov1 + Cov2)^-1
  matrix2_inverse(B,cov);


  double m[2];
  m[0] = l1.x - l2.x;
  m[1] = l1.y - l2.y;

  double md2 = matrix2_multiply2(B,m);

  double vol = VOL2_FIXED_PARAM1*sqrt(1/det)*exp(- 0.5*md2);

  return vol;
}

inline double logVolume_a(double a[6]){
  double a1_inv = 1.0/a[1];

  double aux1 = 0.25*POW2(a[3])*a1_inv - a[2];
  double aux2 = a[5] - 0.5*a[3]*a[4]*a1_inv;

  double vol;

  //  vol = M_PI*exp(a[0] + 0.25*(-POW2(a[4])*a1_inv + POW2(aux2)/aux1))
  //      /sqrt(-a[1]*aux1);
 
  vol = logPI + a[0] + 0.25*(-POW2(a[4])*a1_inv + POW2(aux2)/aux1)
    - 0.5*log(-a[1]*aux1);
 
  return vol;
}

double logVolumeOfTheProduct(const Gaussian2d &l1, 
                             const Gaussian2d &l2){
  double B[2][2];
  double cov[2][2];

  matrix2_add(cov,l1.cov,l2.cov);

  double det = matrix2_det(cov);

  //B = (Cov1 + Cov2)^-1
  matrix2_inverse(B,cov);

  double m[2];
  m[0] = l1.x - l2.x;
  m[1] = l1.y - l2.y;

  double md2 = matrix2_multiply2(B,m);

  double vol = LOGVOL2_FIXED_PARAM1 - 0.5*(log(det) + md2);

  return vol;
}


const double VOL3_FIXED_PARAM1    = 1.0/(sqrt(2*M_PI)*2*M_PI);
const double LOGVOL3_FIXED_PARAM1 = - 1.5*log(2*M_PI);

double volumeOfTheProduct(const Gaussian3d &l1, 
                          const Gaussian3d &l2){
  double B[3][3];
  double cov[3][3];

  matrix3_add(cov,l1.cov,l2.cov);

  double det = matrix3_det(cov);

  //B = (Cov1 + Cov2)^-1
  matrix3_inverse(B,cov);


  double m[3];
  m[0] = l1.x - l2.x;
  m[1] = l1.y - l2.y;
  m[2] = l1.z - l2.z;

  double md2 = matrix3_multiply2(B,m);

  double vol = VOL3_FIXED_PARAM1*sqrt(1/det)*exp(-0.5*md2);

  return vol;
}

double logVolumeOfTheProduct(const Gaussian3d &l1, 
                          const Gaussian3d &l2){
  double B[3][3];
  double cov[3][3];

  matrix3_add(cov,l1.cov,l2.cov);

  double det = matrix3_det(cov);

  //B = (Cov1 + Cov2)^-1
  matrix3_inverse(B,cov);


  double m[3];
  m[0] = l1.x - l2.x;
  m[1] = l1.y - l2.y;
  m[2] = l1.z - l2.z;

  double md2 = matrix3_multiply2(B,m);

  double vol = LOGVOL3_FIXED_PARAM1 - 0.5*log(det) - 0.5*md2;

  return vol;
}

////////////////////////////////////////////////////////////
// Line
////////////////////////////////////////////////////////////
bool Line::intersect(double *x, double *y, const Line &line)const{
  double det = a*line.b - b*line.a;

  if(fabs(det) < 1e-7) return false;

  *x = ( b*line.c - c*line.b)/det;
  *y = (-a*line.c + c*line.a)/det;

  return true;
}

void Line::project(double *xo, double *yo, double x, double y){
  double c2 = b*x - a*y;

  *xo =  b*c2 -  a*c;
  *yo = -b*c  -  a*c2;
}


////////////////////////////////////////////////////////////
// Line Segment
////////////////////////////////////////////////////////////
void LineSegment::set(double x1, double y1, double x2, double y2){
  this->x1 = x1; this->y1 = y1;
  this->x2 = x2; this->y2 = y2;

  double dx = x2 - x1;
  double dy = y2 - y1;
  angle = atan2(dy,dx);
  length = sqrt(dx*dx + dy*dy);

  ca = cos(angle);
  sa = sin(angle);
}

//Integral of the N(mean, sigma) from x1 to x2
double gaussianIntegral(double mean, double sigma, double x1, double x2){
  x1 -= mean;
  x2 -= mean;
  double scaler = M_SQRT1_2/sigma; //1/(sqrt(2)*sigma)

  double p = 0.5*(erf(x2*scaler) - erf(x1*scaler));

  return p;
}

//Match 2d Gaussian to a line segment 0,0 -> L,0
double matchPointToLine(const Gaussian2d &p, double L){
  double A[2][2];
  double M[2];
  double a,b,c;
  double p_log = 0;

  //Compute cross section of the Gaussian

  //        exp(-1/2*(a*x^2 + b*x + c) )
  // p(x) = ----------------------------
  //           2*PI*sqrt(det(cov))

  matrix2_inverse(A, p.cov);
  M[0] = p.x;  M[1] = p.y;
  double detCov = matrix2_det(p.cov);

  a = A[0][0];
  b = -( 2*A[0][0]*M[0] + M[1]*(A[0][1] + A[1][0]) );
  c = matrix2_multiply2(A,M);

#if 0
  printf("MatchPointToLine: (%g,%g) -> 0,0->%g,0\n"
         " %+.5e %+.5e\n"
         " %+.5e %+.5e\ndet = %g\n",
	 p.x, p.y, L,
         p.cov[0][0], p.cov[0][1],
	 p.cov[1][0], p.cov[1][1], detCov);

  printf("inv(cov) = \n%e %e\n%e %e\n",
	 A[0][0],A[0][1], A[1][0], A[1][1]);

  printf("A = %g, B = %g, C = %g\n", a, b, c);
#endif

  // Compute log[ Integral 0->L{ p(x) } ]

  //1. part -Inf->Inf
  p_log = -0.5*(LN_2PI + log(a*detCov) + c - 0.25*b*b/a);
  //  printf("Infinite log: %g\n", p_log);

  double sqrt_a = sqrt(a);
  double v1 = 0.5*b*M_SQRT1_2/sqrt_a;
  double v2 = v1 + L*sqrt_a*M_SQRT1_2;

  double p_log2;

  if(v1 < -5 && v2 > 5){
    return p_log;
  }else{
    p_log2 = log( 0.5*(erf(v2) - erf(v1) ) );
  }

  //  printf("Correction: %g->%g, %g\n",v1,v2, p_log2);

  return (p_log + p_log2);
}

//Returns: number of affected rays ( n - n_nomatch )
int matchScanToLine(double *wlog  //-- output
		    ,int *match   //-- output: 0-nomatch , 1-match,
		                  //           2-conflict (crosses the line)
		    ,const double *range
		    ,const double *angle //Sorted in ascending order
		    ,int n
		    ,const RobotPose* sensor
		    ,const LineSegment* line
		    ,const struct LineLaserMatchParams *params){

  const int NO_MATCH       = 0;
  const int MATCHED        = 1;
  const int CONFLICT_LONG  = 2;

  //Translate robot Pose in to line coordinate frame
  // We are now observing line segment (0,0) -> (L,0)
  double x0,y0,a0;
  double dx = sensor->x - line->X1();
  double dy = sensor->y - line->Y1();
  x0 =  dx*line->CA() + dy*line->SA();
  y0 = -dx*line->SA() + dy*line->CA();
  a0 = angleDiffRad(sensor->rot, line->Angle());

  double L = line->Length();

  //  printf("Pose: %g %g %g(deg)\n", x0,y0,rad2deg(a0));

  //Check if we can observe anything

  if(y0 <= 0){
//     printf("Other side of the line\n");
    return -1; //Wrong side of the line segments
  }

  if(y0 > params->maxRange){
//     printf("Outside sensor range (distance)\n");
    return -2; //Too far from the line
  }

  double angle_margin = 2*params->sigma_a;

  double angleMin  = atan2(-y0,  -x0);
  double angleMin2 = angleMin - deg2rad(angle_margin);
  double angleMax  = atan2(-y0,  L  - x0);
  double angleMax2 = angleMax + deg2rad(angle_margin);

  double da = angleDiffRad(angleMax2,angleMin2);
  double a1 = angleDiffRad(angle[0]   + a0, angleMin2);

  //Check if line is to the left of the sensor range
  if(a1 > da){
    return -3;
  }


  //Check if line is to the right of the sensor range
  if(a1 < 0){
    double a2 = angleDiffRad(angle[n-1] + a0, angleMin2);
    if(a2 < 0){
      return -4;
    }
  }


//   printf("Angle Range: %g -> %g\n", rad2deg(angleMin), rad2deg(angleMax));


  int i;
  int n_matched  = 0;
  int n_conflict = 0;

  for(i = 0; i < n; ++i){
    double a = modPI(angle[i] + a0);
    match[i] = NO_MATCH;
//     printf("[%d].a = %g\n", i+1, rad2deg(a));

    if(a > angleMin2 && a < angleMax2){
      double ca,sa;
      double x,y;
      double w;
      sa = sin(a); y = range[i]*sa + y0;

//       printf("[%d].y = %g\n",i+1, y);
      
      if(y > params->maxDistToLine){
        // Too short
	// Nothing to be done here.

      }else if(y < -params->maxDistToLine){
	// Too long
	// 1. Compute the probability of the ray crossing the line
	//    p(crossing) =  1/(2*PI*sqrt(saa)) * 
	//         integral angleMin->angleMax {exp(-0.5*(x-a)^2/saa)dx}
	// 2. Check if it is high enough
	w = gaussianIntegral(a, params->sigma_a, angleMin, angleMax);

	if(w > 0.01){ //If it's likely to intersect the line
  	  wlog[i]  = log(w);
	  match[i] = CONFLICT_LONG;
	  n_conflict += 1;
	}
      }else{

	if(range[i] < params->maxRange){
	  //Within angular and distance range
	  ca = cos(a); x = range[i]*ca + x0;

	  //Convert to Cartesian coordinates
	  Gaussian2d p;
	  p.x = x; p.y = y;
	  double caca = ca*ca;
	  double sasa = sa*sa;
	  double r2 = range[i]*range[i];
	  
	  p.cov[0][0] =  caca* params->sigma_r2 + r2*sasa*params->sigma_a2;
	  p.cov[1][1] =  sasa* params->sigma_r2 + r2*caca*params->sigma_a2;
          double sxy  = ca*sa*(params->sigma_r2 - r2*params->sigma_a2);

	  p.cov[0][1] = p.cov[1][0] = fabs(sxy);

	  wlog[i] = matchPointToLine(p,L);
	  match[i] = MATCHED;
	  n_matched += 1;

	}else{
	  //No reflection within the proximity of a line segment.
	  // do nothing -- assume no match.
	}
      }

    }
    
  }

  return n_matched + n_conflict;
}


void fitLinesToScan(Line* lines, double *dist2
		    ,const double *x
		    ,const double *y
		    ,int n
		    ,int nPointsPerLine){
  double *xx = new double[n];
  double *xy = new double[n];
  double *yy = new double[n];

  double Sxx, Sxy,Sy,Sx;
  double m,k;   // y = m*x + k
  double a,b,c; // ax + by + c = 0
  double dx,dy;
  double det;

  bool swap;
  bool skip;

  int i,j, i_stop;
  int nLines = n - nPointsPerLine + 1;

  for(i = 0; i < n; ++i){
    xx[i] = x[i]*x[i];
    xy[i] = x[i]*y[i];
    yy[i] = y[i]*y[i];
  }

  for(i = 0; i < nLines; ++i){
    swap = skip = false;
    Sxx = Sxy = Sx = Sy = 0.0;
    i_stop = i + nPointsPerLine - 1;

    dx = x[i] - x[i_stop];
    dy = y[i] - y[i_stop];

    swap = (fabs(dx) < fabs(dy));

    for(j = i; j <= i_stop && !skip; ++j){
      if(isnan(x[j])){
	skip = true;
      }else if(swap){
	Sx  += y[j];   Sy += x[j]; 
        Sxx += yy[j]; Sxy += xy[j]; 
      }else{
	Sx  += x[j];   Sy += y[j]; 
        Sxx += xx[j]; Sxy += xy[j]; 
      }
    }

    if(!skip){
      //Compute least squares y = mx + k
      det = 1.0/(nPointsPerLine*Sxx - Sx*Sx);
      m   = (nPointsPerLine*Sxy - Sx*Sy) * det;
      k   = (Sy*Sxx - Sx*Sxy)*det;

      //compute a,b,c
      if(!swap){
	b = 1/sqrt(1 + m*m);
	a = -m*b;
	c = -k*b;
      }else{
	a = 1/sqrt(1 + m*m);
	b = -m*a;
	c = -k*a;
      }

      if(c > 0){ a = -a; b = -b; c = -c;}

      //set the line
      lines[i].set(a,b,c);

      //Compute point to line distances.
      dist2[i] = 0;
      for(j = i; j <= i_stop; ++j){
	double d = x[j]*a + y[j]*b + c;
	dist2[i] += d*d;
      }
    }else{
      dist2[i] = NaN;
      while(i < j-1 && i < nLines - 1){  i += 1; dist2[i] = NaN;}

      if(j < nLines) dist2[j] = NaN;
    }
  }

  delete[] xx; delete[] yy; delete[] xy;
}
