/**
 * LaserCornerDetectorMain.h - laser corner detector
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
 *   $Id: LaserCornerDetectorMain.h,v 1.12 2003/09/13 11:46:10 david Exp $ 
 */

#ifndef __INC_LaserCornerDetectorMain_h
#define __INC_LaserCornerDetectorMain_h

#include "LeastSquares2D.h"

class LaserCornerDetectorMain{
public:
  LaserCornerDetectorMain();

  ~LaserCornerDetectorMain();
  
  unsigned int ExtractCorners(double *x, double *y, unsigned int nData,
			      double thresh);
  void FitLines(double *x, double *y, unsigned int nData);
  
  void ClearCorners();

  unsigned int NumCorners();
  int CornerType(unsigned int i);
  double Prob(unsigned int i);
  double X(unsigned int i);
  double Y(unsigned int i);
  double XTangentLeft(unsigned int i);
  double YTangentLeft(unsigned int i);
  double XTangentRight(unsigned int i);
  double YTangentRight(unsigned int i);
  double Angle(unsigned int i);

  void DumpCorners();

  unsigned int ExtractOneCorner(double *x, double *y, 
				unsigned int nData,
				double thresh, unsigned int pivot);


private:
  typedef struct {
    unsigned int type;
    double prob;
    double x;
    double y;
    double xtl;
    double ytl;
    double xtr;
    double ytr;
    double angle;
  } CornerDataType;

  CornerDataType *corners_;
  unsigned int numCorners_;
  unsigned int sizeofCorners_;

  LineFittingParamType *params_;

  double maxRadiusAngForDisc_;
  double jumpAngleScale_;
  double minDiscDist_;
  double discProbScale_;
  double linesParallelError_;
  double minJumpOffset_;
  double jumpOffsetSpacingScale_;
  double jumpGapPtsScale_;
  double maxConcaveCornerError_;
  double maxConvexCornerError_;
  double minCornerSep_;

  Line2DCartDataType *lines_;
  unsigned int numLines_;

  Line2DCartDataType **leftLines_;
  unsigned int sizeofLeftLines_;
  unsigned int numLeftLines_;

  Line2DCartDataType **rightLines_;
  unsigned int sizeofRightLines_;
  unsigned int numRightLines_;

  void ClusterCorners();
  unsigned int AddCorner(unsigned int type, double prob, double x, double y, 
			 double xtl, double ytl, double xtr, double ytr, 
			 double angle);
  void MergeCorners(unsigned int i, unsigned int j);
  void DeleteCorner(unsigned int i);
  int FindConvexHiddens(double *x, double *y, unsigned int nData,
			double thresh, unsigned int pivot);
  double LookDiscRight(double *x, double *y, unsigned int nData, 
		       unsigned int pivot, double xl, double yl, 
		       double xt, double yt, double *angle,
		       double *xc, double *yc, double *xct, double *yct);
  double LookDiscLeft(double *x, double *y, unsigned int nData, 
		      unsigned int pivot, double xl, double yl, 
		      double xt, double yt, double *angle,
		      double *xc, double *yc, double *xct, double *yct);
  int FindBestJump(double *x, double *y, unsigned int nData, 
		   double thresh, unsigned int pivot);
  double CheckForJump(double *x, double *y, unsigned int nData, 
		      unsigned int l, unsigned int r, double thresh);
  void ComputeJumpCentre(double *x, double *y, unsigned int nData, 
			 unsigned int l, unsigned int r,
			 double xll, double yll, double xtl, double ytl, 
			 double xlr, double ylr, double xtr, double ytr,
			 double *xc, double *yc);
  double LookJump(double *x, double *y, unsigned int nData, 
		  unsigned int l, unsigned int r, double xll, double yll, 
		  double xtl, double ytl, double xlr, double ylr, 
		  double xtr, double ytr);

  int ComputeIntersect(double xll, double yll, double xtl, double ytl, 
		       double xlr, double ylr, double xtr, double ytr, 
		       double *xc, double *yc);
  int FindConvexAndConcave(double *x, double *y, unsigned int nData,
			   double thresh, unsigned int pivot);
  int CornerTypesEquiv(int t1, int t2);
  void ResetLines(unsigned int n);
};

#endif /* __INC_LaserCornerDetectorMain_h */

/*
 * Local Variables:
 * mode: C++
 * End:
 */
