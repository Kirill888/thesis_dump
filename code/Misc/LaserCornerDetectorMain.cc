/**
 * LaserCornerDetectorMain.cc - laser corner detector
 * 
 * Copyright (C) 2003 David Austin
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 * 
 * 
 * Longer description
 * 
 * 
 * @author  David Austin
 *          d.austin@computer.org
 *          Dept of Systems Engineering,
 *          Research School of Information Sciences and Engineering,
 *          Australian National University,
 *          Canberra, 0200, Australia.
 *
 * @version 
 *   $Id: LaserCornerDetectorMain.cc,v 1.18 2003/11/18 01:00:18 david Exp $ 
 */

#ifndef DEPEND
#include <stdlib.h>
#endif /* !DEPEND */

#include "DROSDebug.h"
#include "LaserCornerDetectorMain.h"
#include "DROSParamSet.h"
#include "AngleTools.h"
#include "DROSDefines.h"
#include "LeastSquares2D.h"
#include "DROSVector.h"

#define LASER_CENTRE_X   0.0 //m
#define LASER_CENTRE_Y   0.0   //m

//=============================================================================

LaserCornerDetectorMain::LaserCornerDetectorMain(){
  corners_ = NULL;
  numCorners_ = 0;
  sizeofCorners_ = 0;

  lines_ = NULL;
  numLines_ = 0;

  leftLines_ = NULL;
  sizeofLeftLines_ = 0;

  rightLines_ = NULL;
  sizeofRightLines_ = 0;

  params_ = (LineFittingParamType *) malloc(sizeof(LineFittingParamType));
  params_->minPtsForLine = 10;
  params_->minDistForLine = 0.4;
  params_->maxProbScaleLen = 3.0;
  params_->maxDistForLine = 500.0; 
  params_->lineLenProbScale = 0.2;
  params_->ptErrorThresh = 0.01;
  params_->errorLenScale = 3.5; /* 
				 * Roughly speaking, want a 0.5m line
				 * with one erroroneous point 0.1m off
				 * to have a probability of 0.5 
				 */
  params_->maxPtSpacing = 0.5;
  params_->ptSpacingThresh = 0.1;
  params_->ptSpacingScale = 2.0;

  minJumpOffset_ = 20.0e-3;
  jumpOffsetSpacingScale_ = 0.1;
  jumpAngleScale_ = 2.0;
  jumpGapPtsScale_ = 0.01;
  maxRadiusAngForDisc_ = deg2rad(20.0);
  minDiscDist_ = 0.2;
  discProbScale_ = 1.0;

  linesParallelError_ = deg2rad(10.0);

  maxConvexCornerError_ = 20.0e-3;
  maxConcaveCornerError_ = 20.0e-3;

  minCornerSep_ = 0.2;
}

LaserCornerDetectorMain::~LaserCornerDetectorMain(){
  free(params_);

  free(lines_);
  free(leftLines_);
  free(rightLines_);
  free(corners_);
}
  
//=============================================================================

unsigned int LaserCornerDetectorMain::ExtractCorners(double *x, double *y, 
						     unsigned int nData,
						     double thresh) {
  if (nData < (unsigned int) (params_->minPtsForLine * 2 + 2)) {
    return 0;
  }

  unsigned int pivot;

  FitLines(x, y, nData);

  for (pivot = params_->minPtsForLine + 1; 
       pivot < nData - params_->minPtsForLine - 1; 
       pivot++) {
    ExtractOneCorner(x, y, nData, thresh, pivot);
  }

  if (numCorners_ > 1) 
    ClusterCorners();

  DROS_DO(10000, 
	  FILE *f;
	  unsigned int i;

	  f = fopen("points", "w");

	  for (i = 0; i < numCorners_; i++) {
	    fprintf(f, "%.4f %.4f\n", corners_[i].x, corners_[i].y);
	  }
	  
	  fclose(f);
	  );

  return numCorners_;
}

//=============================================================================

void LaserCornerDetectorMain::FitLines(double *x, double *y, 
				       unsigned int nData) {
  unsigned int i;
  int l;
  int r;

  //  for(i = 0; i < nData; ++i) printf("las[%d] = %.3f, %.3f\n", i, x[i],y[i]);

  //#define EXCESSIVE_DEBUG
#ifdef EXCESSIVE_DEBUG
  DROS_DO(1000, for (i = 0; i < nData; i++) {
    DROS_DEBUG(1, "input[" << i << "] = " << x[i] << ", " << y[i]);
    
  });
  DROS_DO(10000, 
	  FILE *f;
	  f = fopen("scan", "w");

	  for (i = 0; i < nData; i++) {
	    fprintf(f, "%.4f %.4f\n", x[i], y[i]);
	  }
	  
	  fclose(f);
	  );
#endif
  if (lines_ != NULL)
    free(lines_);

  numLines_ = FindLines2DProb(x, y, nData, &lines_, 0,
			      params_, TRUE);
  DROS_DEBUG(200, "Found " << numLines_ << " lines");
  ResetLines(nData);  

  //  printf("Found: %d lines.\n",numLines_);

  for (i = 0; i < numLines_; i++) {
    l = lines_[i].start;
    r = lines_[i].start + lines_[i].nPoints - 1;
    if ((l >= 0) && (l < (int)nData)) {
      leftLines_[l] = &(lines_[i]);
    }
    if ((r >= 0) && (r < (int)nData)) {
      rightLines_[r] = &(lines_[i]);
    }
  }
  
#ifdef EXCESSIVE_DEBUG
  DROS_DO(1000, DROS_DEBUG(1, "Found lines ...");
	  Print2DLines(lines_, numLines_););
#if 0
  DROS_DO(1000, DumpLines("leftlines", leftLines_, sizeofLeftLines_););
#endif
#endif
}

//=============================================================================

void LaserCornerDetectorMain::ClearCorners() {
  numCorners_ = 0;
}
 
//=============================================================================

unsigned int LaserCornerDetectorMain::NumCorners() {
  return numCorners_;
}

//=============================================================================

int LaserCornerDetectorMain::CornerType(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].type;
  } else {
    DROS_DEBUG(5, "Warning: CornerType called for " << i 
	       << " out of " << numCorners_);
    return -1;
  }
}

//=============================================================================

double LaserCornerDetectorMain::Prob(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].prob;
  } else {
    DROS_DEBUG(5, "Warning: Prob called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::X(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].x;
  } else {
    DROS_DEBUG(5, "Warning: X called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::Y(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].y;
  } else {
    DROS_DEBUG(5, "Warning: Y called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::XTangentLeft(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].xtl;
  } else {
    DROS_DEBUG(5, "Warning: XTangentLeft called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::YTangentLeft(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].ytl;
  } else {
    DROS_DEBUG(5, "Warning: YTangentLeft called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::XTangentRight(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].xtr;
  } else {
    DROS_DEBUG(5, "Warning: XTangentRight called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::YTangentRight(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].ytr;
  } else {
    DROS_DEBUG(5, "Warning: YTangentRight called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

double LaserCornerDetectorMain::Angle(unsigned int i) {
  if (i < numCorners_) {
    return corners_[i].angle;
  } else {
    DROS_DEBUG(5, "Warning: Angle called for " << i 
	       << " out of " << numCorners_);
    return -DBL_MAX;
  }
}

//=============================================================================

void LaserCornerDetectorMain::DumpCorners() {
  unsigned int i;

  for (i = 0; i < numCorners_; i++) {
    switch (corners_[i].type) {
    case DROS_CORNER_CONCAVE:
      fprintf(stderr, "CONCAVE      ");
      break;

    case DROS_CORNER_CONVEX:
      fprintf(stderr, "CONVEX       ");
      break;

    case DROS_CORNER_CONVEX_HIDDEN:
      fprintf(stderr, "CONVEX_HIDDEN");
      break;

    case DROS_CORNER_JUMP:
      fprintf(stderr, "JUMP         ");
      break;

    default:
      fprintf(stderr, "UNKNOWN      ");
    }

    fprintf(stderr, " %.2f (%.3f,%.3f) tl (%.3f,%.3f) tr (%.3f,%.3f) a %.3f\n",
	    corners_[i].prob, corners_[i].x, corners_[i].y,
	    corners_[i].xtl, corners_[i].ytl, corners_[i].xtr, corners_[i].ytr,
	    rad2deg(corners_[i].angle));

  }
}

//=============================================================================

unsigned int LaserCornerDetectorMain::ExtractOneCorner(double *x, double *y, 
						       unsigned int nData,
						       double thresh, 
						       unsigned int pivot) {
  unsigned int found;
  
  found = 0;
  
  found += FindConvexHiddens(x, y, nData, thresh, pivot);
  found += FindBestJump(x, y, nData, thresh, pivot);
  found += FindConvexAndConcave(x, y, nData, thresh, pivot);

  return found;
}

//=============================================================================

void LaserCornerDetectorMain::ClusterCorners() {
  unsigned int i;
  unsigned int j;
  int found;
  double d;

  i = 0; 
  while (i < numCorners_ - 1) {
    j = i + 1;
    found = FALSE;
    while ((!found) && (j < numCorners_)) {
      if (CornerTypesEquiv(corners_[i].type, corners_[j].type)) {
	d = sqrt((corners_[i].x - corners_[j].x) * 
		 (corners_[i].x - corners_[j].x) + 
		 (corners_[i].y - corners_[j].y) * 
		 (corners_[i].y - corners_[j].y));
	DROS_DEBUG(10000, i << " " << j << " sep=" << d);
	if (d < minCornerSep_) {
	  DROS_DO(500, 
		  DROS_DEBUG(1, "Found common corners " << i << " " << j);
		  DumpCorners();
		  );
	  found = TRUE;
	}
      }
      if (!found)
	j++;
    }

    if (found) {
      MergeCorners(i, j);
      DeleteCorner(j);
    } else {
      i++;
    }
  }
}

//=============================================================================

unsigned int LaserCornerDetectorMain::AddCorner(unsigned int type, 
						double prob, 
						double x, double y, 
						double xtl, double ytl, 
						double xtr, double ytr, 
						double angle) {
  if (numCorners_ >= sizeofCorners_) {
    sizeofCorners_ += 10;
    corners_ = (CornerDataType *) realloc(corners_, sizeofCorners_ * 
					  sizeof(CornerDataType));
  }

  corners_[numCorners_].type = type;
  corners_[numCorners_].prob = prob;
  corners_[numCorners_].x = x;
  corners_[numCorners_].y = y;
  corners_[numCorners_].xtl = xtl;
  corners_[numCorners_].ytl = ytl;
  corners_[numCorners_].xtr = xtr;
  corners_[numCorners_].ytr = ytr;
  corners_[numCorners_].angle = angle;
  numCorners_++;

  return numCorners_;
}

//=============================================================================

void LaserCornerDetectorMain::MergeCorners(unsigned int i, unsigned int j) {
  /*
   * Store the merged corner in i, weighting by probabilities.
   */

  double a;
  double b;

  a = corners_[i].prob;
  b = corners_[j].prob;

  if (corners_[i].type != corners_[j].type) {
    /*
     * Must have a CONVEX and CONVEX_HIDDEN corner.  Then we want to
     * take the CONVEX data in preference to the other data for the angle
     */
    
    if (corners_[i].type != DROS_CORNER_CONVEX) {
      corners_[i].type = DROS_CORNER_CONVEX;
      corners_[i].angle = corners_[j].angle;
    }
  }
#if 0
  corners_[i].prob = 1.0 - (1.0 - a) * (1.0 - b);
  corners_[i].x = (a * corners_[i].x + b * corners_[j].x) / (a + b);
  corners_[i].y = (a * corners_[i].y + b * corners_[j].y) / (a + b);
  corners_[i].xtl = (a * corners_[i].xtl + b * corners_[j].xtl) / (a + b);
  corners_[i].ytl = (a * corners_[i].ytl + b * corners_[j].ytl) / (a + b);
  corners_[i].xtr = (a * corners_[i].xtr + b * corners_[j].xtr) / (a + b);
  corners_[i].ytr = (a * corners_[i].ytr + b * corners_[j].ytr) / (a + b);
#else
  //Choose best one
  if(a < b){
    corners_[i].prob = b;
    corners_[i].x   = corners_[j].x;
    corners_[i].y   = corners_[j].y;
    corners_[i].xtl = corners_[j].xtl;
    corners_[i].ytl = corners_[j].ytl;
    corners_[i].xtr = corners_[j].xtr;
    corners_[i].ytr = corners_[j].ytr;
  }
#endif
}

//=============================================================================

void LaserCornerDetectorMain::DeleteCorner(unsigned int i) {
  if (i >= numCorners_) {
    DROS_DEBUG(1, "Warning: DeleteCorner called with " << i << " > " 
	       << numCorners_);
    return ;
  }

  if (i < numCorners_ - 1) {
    unsigned int j;
    
    for (j = i + 1; j < numCorners_; j++) 
      corners_[j - 1] = corners_[j];
  }
  
  numCorners_--;
  DROS_DEBUG(990, "Deleted corner " << i << " now have " << numCorners_ 
	     << " corners");
}

//=============================================================================

int LaserCornerDetectorMain::FindConvexHiddens(double *x, double *y, 
					       unsigned int nData,
					       double thresh, 
					       unsigned int pivot) {
  double p_dl;
  double p_dr;
  double xc;
  double yc;
  double xctl;
  double yctl;
  double angle;
  unsigned int found;
  found = 0;

  if ((leftLines_[pivot] != NULL) && (leftLines_[pivot]->prob >= thresh)) {
    p_dr = LookDiscRight(x, y, nData, pivot, 
			 leftLines_[pivot]->xs, leftLines_[pivot]->ys, 
			 leftLines_[pivot]->xt, leftLines_[pivot]->yt, 
			 &angle, &xc, &yc, &xctl, &yctl);
    DROS_DEBUG(200, "Prob disc right = " << p_dr * leftLines_[pivot]->prob
	       << " angle is " << rad2deg(angle));
    if (p_dr * leftLines_[pivot]->prob >= thresh) {
      DROS_DEBUG(150, "Adding CONVEX_HIDDEN corner at " << xc << ", " << yc 
		 << " angle " << rad2deg(angle));
      AddCorner(DROS_CORNER_CONVEX_HIDDEN, p_dr * leftLines_[pivot]->prob, 
		xc, yc, xctl, yctl, 0.0, 0.0, angle);
      found++;
    }
  }

  if ((rightLines_[pivot] != NULL) && (rightLines_[pivot]->prob >= thresh)) {
    p_dl = LookDiscLeft(x, y, nData, pivot, 
			rightLines_[pivot]->xe, rightLines_[pivot]->ye, 
			-rightLines_[pivot]->xt, -rightLines_[pivot]->yt, 
			&angle, &xc, &yc, &xctl, &yctl);
    DROS_DEBUG(200, "Prob disc left = " << p_dl * rightLines_[pivot]->prob
	       << " angle is " << rad2deg(angle));
    if (p_dl * rightLines_[pivot]->prob >= thresh) {
      DROS_DEBUG(150, "Adding CONVEX_HIDDEN corner at " << xc << ", " << yc 
		 << " angle " << rad2deg(angle));
      AddCorner(DROS_CORNER_CONVEX_HIDDEN, p_dl * rightLines_[pivot]->prob, 
		xc, yc, xctl, yctl, 0.0, 0.0, angle);
      found++;
    }
  }
  return found;
}

//=============================================================================

double LaserCornerDetectorMain::LookDiscRight(double *x, double *y, 
					      unsigned int nData, 
					      unsigned int pivot, 
					      double xl, 
					      double yl, 
					      double xt, 
					      double yt, 
					      double *angle,
					      double *xc,
					      double *yc,
					      double *xct,
					      double *yct) {
  double xd;
  double yd;
  double dd;
  double xr;
  double yr;
  double dr;
  double dot;
  double dot2;
  double theta;
  double theta2;
  double dp;
  double xoff;
  double yoff;
  double doff;
  double ap;

  DROS_DEBUG(400, "LookDiscRight " << pivot);

  xd = x[pivot - 1] - xl;
  yd = y[pivot - 1] - yl;
  xr = x[pivot - 1] - LASER_CENTRE_X;
  yr = y[pivot - 1] - LASER_CENTRE_Y;

  DROS_DEBUG(400, "x[" << pivot << "] = " << x[pivot] 
	     << " y[" << pivot << "] = " << y[pivot]);
  if ((xd * xr + yd * yr) < 0) {
    return 0.0;
  }

  Perp2D(xd, yd, xt, yt, &xoff, &yoff);
  doff = sqrt(xoff * xoff + yoff * yoff);
  DROS_DEBUG(400, "xd = " << xd << " yd = " << yd 
	     << " xt = " << xt << " yt = " << yt
	     << " xoff = " << xoff << " yoff = " << yoff);	     
  DROS_DEBUG(400, "doff = " << doff << " minDiscDist_ = " << minDiscDist_);
  if (doff < minDiscDist_) {
    return 0;
  }

  dr = sqrt(xr * xr + yr * yr);
  dd = sqrt(xd * xd + yd * yd);

  DROS_DEBUG(400, "dd = " << dd << " dr = " << dr 
	     << " xr = " << xr << " yr = " << yr);
  dot = (xd * xr + yd * yr) / (dd * dr);
  theta = acos(dot);

  DROS_DEBUG(400, "theta = " << rad2deg(theta)
	     << " maxRadiusAngForDisc_ = " << rad2deg(maxRadiusAngForDisc_));
  if (fabs(theta) > maxRadiusAngForDisc_) {
    return 0;
  }

  dot2 = (xr * xt + yr * yt) / dr;
  theta2 = fabs(acos(dot2) - M_PI / 2.0);
  ap = 1.0 - theta2 / M_PI * jumpAngleScale_;
  DROS_DEBUG(400, "dot2 = " << dot2 << " theta2 = " << rad2deg(theta2) 
	     << " ap = " << ap);
  if (ap < 0.0)
    return 0;

  dp = (dd - minDiscDist_) * discProbScale_;
  if (dp > 1.0)
    dp = 1.0;

  dot = (xd * xt + yd * yt) / dd;              /* xt,yt is a unit vector */
  *angle = acos(dot);

  Project2D(x[pivot] - xl, y[pivot] - yl, xt, yt, xc, yc);

  *xc += xl;
  *yc += yl;
  //*xc = x[pivot];
  //*yc = y[pivot];
  *xct = xt;
  *yct = yt;

  return exp(-fabs(theta) / maxRadiusAngForDisc_) * dp * ap;
}

//=============================================================================

double LaserCornerDetectorMain::LookDiscLeft(double *x, double *y, 
					     unsigned int nData, 
					     unsigned int pivot, 
					     double xl, 
					     double yl, 
					     double xt, 
					     double yt, 
					     double *angle,
					     double *xc,
					     double *yc,
					     double *xct,
					     double *yct) {
  double xd;
  double yd;
  double dd;
  double xr;
  double yr;
  double dr;
  double dot;
  double dot2;
  double theta;
  double theta2;
  double dp;
  double xoff;
  double yoff;
  double doff;
  double ap;

  DROS_DEBUG(400, "LookDiscLeft " << pivot);

  xd = x[pivot + 1] - xl;
  yd = y[pivot + 1] - yl;
  xr = x[pivot + 1] - LASER_CENTRE_X;
  yr = y[pivot + 1] - LASER_CENTRE_Y;

  if ((xd * xr + yd * yr) < 0) {
    return 0.0;
  }

  Perp2D(xd, yd, xt, yt, &xoff, &yoff);
  doff = sqrt(xoff * xoff + yoff * yoff);
  DROS_DEBUG(400, "xd = " << xd << " yd = " << yd 
	     << " xt = " << xt << " yt = " << yt
	     << " xoff = " << xoff << " yoff = " << yoff);	     
  DROS_DEBUG(400, "doff = " << doff << " minDiscDist_ = " << minDiscDist_);
  if (doff < minDiscDist_) {
    return 0;
  }

  dr = sqrt(xr * xr + yr * yr);
  dd = sqrt(xd * xd + yd * yd);

  dot = (xd * xr + yd * yr) / (dd * dr);
  theta = acos(dot);

  DROS_DEBUG(400, "theta = " << rad2deg(theta)
	     << " maxRadiusAngForDisc_ = " << rad2deg(maxRadiusAngForDisc_));
  if (fabs(theta) > maxRadiusAngForDisc_) {
    return 0;
  }

  dot2 = (xr * xt + yr * yt) / dr;
  theta2 = fabs(acos(dot2) - M_PI / 2.0);
  ap = 1.0 - theta2 / M_PI * jumpAngleScale_;
  DROS_DEBUG(400, "dot2 = " << dot2 << " theta2 = " << rad2deg(theta2) 
	     << " ap = " << ap);
  if (ap < 0.0)
    return 0;

  dp = (dd - minDiscDist_) * discProbScale_;
  if (dp > 1.0)
    dp = 1.0;

  dot = -(xd * xt + yd * yt) / dd;              /* xt,yt is a unit vector */
  *angle = acos(dot);

  Project2D(x[pivot] - xl, y[pivot] - yl, xt, yt, xc, yc);
  *xc += xl;
  *yc += yl;
  //*xc = x[pivot];
  //*yc = y[pivot];
  *xct = xt;
  *yct = yt;


  return exp(-fabs(theta) / maxRadiusAngForDisc_) * dp * ap;
}

//=============================================================================

int LaserCornerDetectorMain::FindBestJump(double *x, double *y, 
					  unsigned int nData,
					  double thresh, unsigned int pivot) {
  const char *dros_function_name = "FindBestJump";
  unsigned int i;
  double prob;
  double bestprob;
  unsigned int l;
  unsigned int r;

  DROS_DEBUG(400, "FindBestJump " << pivot);

  l = pivot;
  r = pivot;
  bestprob = -1.0;
  for (i = 0; i < 5; i++) {
    prob = CheckForJump(x, y, nData, pivot + i + 1, pivot - i, thresh) 
      - i * jumpGapPtsScale_;
    if (prob > bestprob) {
      bestprob = prob;
      l = pivot + i + 1;
      r = pivot - i;
      DROS_DEBUG(399, "Best Jump is now " << l << " " << r << " with prob " 
		 << bestprob);
    }    
    prob = CheckForJump(x, y, nData, pivot + i + 1, pivot - i - 1, thresh)
      - (i + 1) * jumpGapPtsScale_;
    if (prob > bestprob) {
      bestprob = prob;
      l = pivot + i + 1;
      r = pivot - i - 1;
      DROS_DEBUG(399, "Best Jump is now " << l << " " << r << " with prob " 
		 << bestprob);
    }    
  }

  if (bestprob >= thresh) {
    double xc;
    double yc;
    double dot;
    double theta;
    
    dot = -(leftLines_[l]->xt * rightLines_[r]->xt + 
	    leftLines_[l]->yt * rightLines_[r]->yt);
    theta = acos(dot);

    ComputeJumpCentre(x, y, nData, l, r,
		      leftLines_[l]->xs, leftLines_[l]->ys,
		      leftLines_[l]->xt, leftLines_[l]->yt,
		      rightLines_[r]->xe, rightLines_[r]->ye,
		      rightLines_[r]->xt, rightLines_[r]->yt,
		      &xc, &yc);

    DROS_DEBUG(150, "Adding JUMP corner at " << xc << ", " 
 	       << yc << " angle " << rad2deg(theta));

//     printf("Adding JUMP[pivot=%d]: (%d,%d) (%g,%g) [prob %g]\n"
//            ,pivot, l,r,xc,yc,bestprob);

    AddCorner(DROS_CORNER_JUMP, bestprob, xc, yc, 
	      leftLines_[l]->xt, leftLines_[l]->yt,
	      rightLines_[r]->xt, rightLines_[r]->yt,
	      theta);
    return 1;
  }
  
  return 0;
}

//=============================================================================

double LaserCornerDetectorMain::CheckForJump(double *x, double *y, 
					     unsigned int nData,
					     unsigned int l, unsigned int r, 
					     double thresh) {
  const char *dros_function_name = "CheckForJump";
  double p_l;

  if ((leftLines_[l] == NULL) || (rightLines_[r] == NULL))
    return 0.0;

  if ((r >= nData) || (l >= nData))
    return 0.0;

  p_l = leftLines_[l]->prob * rightLines_[r]->prob;
  DROS_DEBUG(400, "CheckForJump: left " << l << " right " << r << " probs: " 
	     << leftLines_[l]->prob << " & " <<  rightLines_[r]->prob << " = "
	     << p_l);

  if ((leftLines_[l]->prob > 0) && (rightLines_[r]->prob > 0) && 
      (p_l >= thresh)) {
    double dot;
    double theta;

    dot = leftLines_[l]->xt * rightLines_[r]->xt + 
      leftLines_[l]->yt * rightLines_[r]->yt;
    theta = acos(dot);

    DROS_DEBUG(150, "Angle between left and right lines is "
	       << rad2deg(theta) << " dot is " << dot);
    
    /* DROS_CORNER_JUMP */
    if (fabs(theta) < linesParallelError_) {
      double p_j;
      p_j = LookJump(x, y, nData, l, r, 
		     leftLines_[l]->xs, leftLines_[l]->ys, 
		     leftLines_[l]->xt, leftLines_[l]->yt,
		     rightLines_[r]->xe, rightLines_[r]->ye, 
		     rightLines_[r]->xt, rightLines_[r]->yt);
      
      p_j *= p_l;
      
      if (p_j >= thresh) {	
	return p_j;
      }
    } 
  }
  return -1.0;
}

//=============================================================================

void LaserCornerDetectorMain::ComputeJumpCentre(double *x, double *y, 
						unsigned int nData, 
						unsigned int l,
						unsigned int r,
						double xll, double yll, 
						double xtl, double ytl, 
						double xlr, double ylr, 
						double xtr, double ytr,
						double *xc, double *yc) {
#if 0
  double dot1;
  double dot2;
  double xoff1;
  double yoff1;
  double xoff2;
  double yoff2;
  double xcl;
  double ycl;
  double xcr;
  double ycr;


  xoff1 = xll - x[l];
  yoff1 = yll - y[l];
  
  dot1 = (xoff1 * xtl + yoff1 * ytl) / sqrt(xoff1 * xoff1 + yoff1 * yoff1);
  
  xoff2 = xlr - x[r];
  yoff2 = ylr - y[r];
  
  dot2 = (xoff2 * xtr + yoff2 * ytr) / sqrt(xoff2 * xoff2 + yoff2 * yoff2);
  
  xcl = xll + xoff1 * dot1;
  ycl = yll + yoff1 * dot1;
  xcr = xlr + xoff2 * dot2;
  ycr = ylr + yoff2 * dot2;

  DROS_DEBUG(400, "xcl = " << xcl << " ycl = " << ycl 
	     << " xcr = " << xcr << " ycr = " << ycr);
  //*xc = (xcl + xcr) / 2.0;
  //*yc = (ycl + ycr) / 2.0;

  *xc = (x[l] + x[r]) / 2.0;
  *yc = (y[l] + y[r]) / 2.0;
#else
  double dist_l;
  double dist_r;

  dist_l = fabs(xtl*yll - ytl*xll);
  dist_r = fabs(xtr*ylr - ytr*xlr);

  if(dist_l < dist_r){
    *xc = x[l];
    *yc = y[l];
  }else{
    *xc = x[r];
    *yc = y[r];
  }

#endif

//   printf("ComputeJumpCentre: l=%d, r=%d,"
//          " (%+g,%+g  %+g,%+g) (%+g,%+g  %+g,%+g) -->(%+g,%+g)\n",
// 	 l,r,xll,yll,xtl,ytl,xlr,ylr,xtr,ytr,*xc,*yc);
}

//=============================================================================

double LaserCornerDetectorMain::LookJump(double *x, double *y, 
					 unsigned int nData, 
					 unsigned int l, 
					 unsigned int r, 
					 double xll, double yll, 
					 double xtl, double ytl, 
					 double xlr, double ylr, 
					 double xtr, double ytr) {
  double xd;
  double yd;
  double len2;
  double len;
  double dot;
  double offset;
  double prob;

  xd = x[l] - x[r];
  yd = y[l] - y[r];

  len2 = xd * xd + yd * yd;
  dot = xd * xtl + yd * ytl;
  offset = len2 - dot * dot;
  DROS_DEBUG(400, "xd = " << xd << " yd = " << yd << " len2 = " << len2 
	     << " dot = " << dot << " diff = " << offset);
  if (offset <= 0) {
    return 0.0;
  }

  len = sqrt(len2);
  offset = sqrt(offset);

  DROS_DEBUG(400, "Jump offset = " << offset << " thresh is " 
	     << minJumpOffset_ + len * jumpOffsetSpacingScale_);
  if ((offset > minJumpOffset_+ len * jumpOffsetSpacingScale_) && 
      (offset < minDiscDist_)) {
    double epow;

    epow = offset / (minDiscDist_ / 2) - 1.0;
    prob = 1.0 - fabs(epow);

    DROS_DEBUG(400, "offset = " << offset << " epow = " << epow 
	       << " prob = " << prob);

    return prob;
  } else {
    return 0.0;
  }
}

//=============================================================================

int LaserCornerDetectorMain::ComputeIntersect(double xll, double yll, 
					      double xtl, double ytl, 
					      double xlr, double ylr, 
					      double xtr, double ytr, 
					      double *xc, double *yc) {
  double det;
  double s;
#ifdef MEGA_DEBUG
  double q;
  double xc2;
  double yc2;
#endif

  det = xtl * ytr - ytl * xtr;

#ifdef MEGA_DEBUG
  DROS_DEBUG(1, "det = " << det);
#endif
  if (fabs(det) < 1e-12) 
    return -1;

  s = (ytr * (xlr - xll) - xtr * (ylr - yll)) / det;
#ifdef MEGA_DEBUG
  q = (ytl * (xlr - xll) - xtl * (ylr - yll)) / det;

  DROS_DEBUG(1, "s = " << s << " q = " << q);
#endif

  *xc = xll + s * xtl;
  *yc = yll + s * ytl;

#ifdef MEGA_DEBUG
  xc2 = xlr + q * xtr;
  yc2 = ylr + q * ytr;
  DROS_DEBUG(1, "xc = " << *xc << " yc = " << *yc << " xc2 = " << xc2
	     << " yc2 = " << yc2);
#endif
  return 0;
}

//=============================================================================

int LaserCornerDetectorMain::FindConvexAndConcave(double *x, double *y, 
						  unsigned int nData,
						  double thresh, 
						  unsigned int pivot) {
  double xc;
  double yc;
  double d;
  unsigned int found;
  
  found = 0;
  if ((leftLines_[pivot] != NULL) && (rightLines_[pivot] != NULL) && 
      (leftLines_[pivot]->prob * rightLines_[pivot]->prob >= thresh)) {
    double dot;
    double theta;
    
    dot = -(leftLines_[pivot]->xt * rightLines_[pivot]->xt + 
	    leftLines_[pivot]->yt * rightLines_[pivot]->yt);
    theta = acos(dot);
    DROS_DEBUG(150, "Angle between left and right lines is "
	       << rad2deg(theta) << " dot is " << dot);
    
    /* Check that the lines are not close to parallel */
    if ((fabs(theta) > linesParallelError_) &&
	(fabs(theta - M_PI) > linesParallelError_)) {
      double sign;
      double p_cc;
      
      sign = leftLines_[pivot]->xt * rightLines_[pivot]->yt -
	rightLines_[pivot]->xt * leftLines_[pivot]->yt;
      
      p_cc = (1.0 - fabs(theta - M_PI / 2.0) / (M_PI / 2.0)) * 
	leftLines_[pivot]->prob * rightLines_[pivot]->prob;
      DROS_DEBUG(200, "tangents: left=(" << leftLines_[pivot]->xt << "," 
		 << leftLines_[pivot]->yt << ") right=(" 
		 << rightLines_[pivot]->xt << "," << rightLines_[pivot]->yt
		 << ")");
      DROS_DEBUG(150, "sign = " << sign << " p_cc = " << p_cc 
		 << " thresh = " << thresh);
      
      if (p_cc >= thresh) {
	ComputeIntersect(leftLines_[pivot]->xs, leftLines_[pivot]->ys, 
			 leftLines_[pivot]->xt, leftLines_[pivot]->yt,
			 rightLines_[pivot]->xe, rightLines_[pivot]->ye, 
			 -rightLines_[pivot]->xt, -rightLines_[pivot]->yt, 
			 &xc, &yc);
	d = sqrt((x[pivot] - xc) * (x[pivot] - xc) + 
		 (y[pivot] - yc) * (y[pivot] - yc));
	DROS_DEBUG(200, "d = " << d 
		   << " maxConcaveCornerError_ = " << maxConcaveCornerError_);
	if (sign < 0) {
	  if (d < maxConcaveCornerError_) {
	    /* DROS_CORNER_CONCAVE */
            DROS_DEBUG(150, "Adding CONCAVE corner at " << xc << ", " 
		       << yc << " angle " << rad2deg(theta));
	    AddCorner(DROS_CORNER_CONCAVE, p_cc, xc, yc, 
		      leftLines_[pivot]->xt, leftLines_[pivot]->yt,
		      -rightLines_[pivot]->xt, -rightLines_[pivot]->yt,
		      theta);
	    found++;
	  }
	} else {
	  if (d < maxConvexCornerError_) {
	    /* DROS_CORNER_CONVEX */
	    DROS_DEBUG(150, "Adding CONVEX corner at " << xc << ", " 
		       << yc << " angle " << rad2deg(theta));
	    AddCorner(DROS_CORNER_CONVEX, p_cc, xc, yc,
		      leftLines_[pivot]->xt, leftLines_[pivot]->yt,
		      -rightLines_[pivot]->xt, -rightLines_[pivot]->yt,
		      theta);
	    found++;
	  }
	}
      }
    }
  }

  return found;
}

//=============================================================================

int LaserCornerDetectorMain::CornerTypesEquiv(int t1, int t2) {
  if (((t1 == DROS_CORNER_CONVEX) || (t1 == DROS_CORNER_CONVEX_HIDDEN)) &&
      ((t1 == DROS_CORNER_CONVEX) || (t1 == DROS_CORNER_CONVEX_HIDDEN)))
    return TRUE;
  else 
    return t1 == t2;
}

//=============================================================================

void LaserCornerDetectorMain::ResetLines(unsigned int n) {
  if (sizeofLeftLines_ < n) {
    sizeofLeftLines_ = n;
    leftLines_ = (Line2DCartDataType **) 
      realloc(leftLines_, sizeof(Line2DCartDataType *) * sizeofLeftLines_);
  }
  bzero(leftLines_, sizeofLeftLines_ * sizeof(Line2DCartDataType *));

  if (sizeofRightLines_ < n) {
    sizeofRightLines_ = n;
    rightLines_ = (Line2DCartDataType **) 
      realloc(rightLines_, sizeof(Line2DCartDataType *) * sizeofRightLines_);
  }
  bzero(rightLines_, sizeofRightLines_ * sizeof(Line2DCartDataType *));
}

//=============================================================================

/*
 * Local Variables:
 * mode: C++
 * End:
 */
