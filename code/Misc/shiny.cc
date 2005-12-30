#ifndef DEPEND
#include <math.h>
#endif

#include "AngleTools.h"
#include "DataSource.h"
#include "debug.h"
#include "landmark2d.h"

static int INTENSITY_THRESHOLD = 1;
static double R_THRESHOLD      = 0.1;

int extractShinyFeatures(double *range_out, double *bearing_out,
                         const LaserSource &las){
  int i;

  int nf = 0;
  int from,to;
  i = 0;

  while(i < las.NumOfPoints()){
    
    while(i < las.NumOfPoints() &&
	  (las.Intensity(i) < INTENSITY_THRESHOLD ||
	   !las.isValidReading(i) )) ++i;

    from = i;

    if(from >= las.NumOfPoints()) break;

    i = i + 1;
    
    while(i < las.NumOfPoints() &&
          las.isValidReading(i) &&
          las.Intensity(i) >= INTENSITY_THRESHOLD &&
	  fabs(las.r(from) - las.r(i)) < R_THRESHOLD) ++i;

    to   = i - 1;

    if(to >= from){
      int j;
      double n = to - from + 1;

      //       if(n == 0){
      //         DEBUG_error("Feature Extractor doesn't work!\n"); 
      //       }

      double sR = 0.0;
      double sA = 0.0;

      for(j = from;j <= to; ++j){
	sR += las.r(j);
	sA += las.a(j);
      }

      double avg_r = sR/n;

      if(avg_r <= 0.05){
	DEBUG_error("Feature close to 0,0!\n");
	for(j = from;j <= to; ++j){
	  DEBUG_error("%d, R = %f, I = %d\n"
		      ,j
		      ,las.r(j)
		      ,las.Intensity(j));
	}
      }else{
        range_out[nf]   = avg_r;
	bearing_out[nf] = angleDiffRad(sA/n,M_PI/2);
	nf += 1;
      }

    }

  }

  return nf;
}

int extractShinyFeatures(ObservationInterface **obs,
                         const LaserSource &las){
  int i;

  int nf = 0;
  int from,to;
  i = 0;

  while(i < las.NumOfPoints()){
    
    while(i < las.NumOfPoints() &&
	  (las.Intensity(i) < INTENSITY_THRESHOLD ||
	   !las.isValidReading(i) )) ++i;

    from = i;

    if(from >= las.NumOfPoints()) break;

    i = i + 1;
    
    while(i < las.NumOfPoints() &&
          las.isValidReading(i) &&
          las.Intensity(i) >= INTENSITY_THRESHOLD &&
	  fabs(las.r(from) - las.r(i)) < R_THRESHOLD) ++i;

    to   = i - 1;

    if(to >= from){
      int j;
      double n = to - from + 1;

      //       if(n == 0){
      //         DEBUG_error("Feature Extractor doesn't work!\n"); 
      //       }

      double sR = 0.0;
      double sA = 0.0;

      for(j = from;j <= to; ++j){
	sR += las.r(j);
	sA += las.a(j);
      }

      double avg_r = sR/n;

      if(avg_r <= 0.05){
	DEBUG_error("Feature close to 0,0!\n");
	for(j = from;j <= to; ++j){
	  DEBUG_error("%d, R = %f, I = %d\n"
		      ,j
		      ,las.r(j)
		      ,las.Intensity(j));
	}
      }else{
//         range_out[nf]   = avg_r;
// 	bearing_out[nf] = angleDiffRad(sA/n,M_PI/2);
	double srr,saa,range,bearing;
	range   = avg_r;
	bearing = angleDiffRad(sA/n,M_PI/2);
	srr = POW2(0.15/2.0) + POW2(0.01*range/3.0);
	saa = POW2(4.0*DEGTORAD/3.0);

	obs[nf] = new Landmark2dObs(range, srr, bearing,saa);

	nf += 1;
      }

    }

  }

  return nf;
}
