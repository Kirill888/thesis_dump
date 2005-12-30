#ifndef DEPEND
#include <math.h>
#endif

#include "LaserCornerExtractor.h"
#include "corner.h"
#include "geometry.h"


LaserCornerDetector::LaserCornerDetector():
   sz_lines(0),n_lines(0),lines(NULL),dist2(NULL)
  ,sz_corners(0), n_corners(0),corners(NULL){

  initLineParams();
  setParams(&params);
}

LaserCornerDetector::LaserCornerDetector(const CornerDetectorParams* p):
   sz_lines(0),n_lines(0),lines(NULL),dist2(NULL)
  ,sz_corners(0), n_corners(0),corners(NULL){

  initLineParams();
  setParams(p);
}

void LaserCornerDetector::destroy(){
  DESTROY_ARRAY(lines);
  DESTROY_ARRAY(dist2);
  DESTROY_ARRAY(corners);
}

void LaserCornerDetector::setParams(const struct CornerDetectorParams* p){
  params = *p;

  n_fit = p->n_fit;
  goodLineThresh = n_fit*p->maxAvgDistPerPoint*p->maxAvgDistPerPoint;

  CONVEX_ANGLE_THRESH = cos(p->angleConvexThresh);
  COLLINEAR_THRESH    = cos(p->angleCollinearThresh);
  HIDDEN_RAY_THRESH   = cos(p->angleMinRayAngle);

  maxConvexDist2 = p->maxConvexDist*p->maxConvexDist;
  minLineLen2 = p->minLineLength*p->minLineLength;

  lineFitParams.minPtsForLine  = n_fit;
  lineFitParams.minDistForLine = p->minLineLength;
}

void LaserCornerDetector::initLineParams(){
//   lineFitParams.minPtsForLine = 10;
//   lineFitParams.minDistForLine = 0.4;

  lineFitParams.maxProbScaleLen = 3.0;
  lineFitParams.maxDistForLine = 500.0; 
  lineFitParams.lineLenProbScale = 0.2;
  lineFitParams.ptErrorThresh = 0.01;
  lineFitParams.errorLenScale = 3.5; /* 
				 * Roughly speaking, want a 0.5m line
				 * with one erroroneous point 0.1m off
				 * to have a probability of 0.5 
				 */
  lineFitParams.maxPtSpacing = 0.5;
  lineFitParams.ptSpacingThresh = 0.1;
  lineFitParams.ptSpacingScale = 2.0;
}

int LaserCornerDetector::ExtractCorners(const LaserSource &las){
  n_data = las.numPoints();
  range  = las.getR();
  angle  = las.getA();
  ca     = las.getCA();
  sa     = las.getSA();
  x      = las.getX();
  y      = las.getY();

  n_lines = n_data - n_fit + 1;

  if(n_lines > sz_lines){
    DESTROY_ARRAY(lines); 
    DESTROY_ARRAY(dist2);

    sz_lines = n_lines;

    lines = new Line[sz_lines];
    dist2 = new double[sz_lines];
  }

  if(sz_corners < n_data){
    sz_corners = n_data;
    DESTROY_ARRAY(corners);
    corners = new struct Corner[sz_corners];
  }

  n_corners = 0;

  //First fit the lines
  fitLinesToScan(lines,dist2, x, y, n_data, n_fit); //geometry.cc

  //Now look for corners
  lookForCorners();

  return n_corners;
}

void LaserCornerDetector::lookForCorners(){
  int i, lastGood;

  i = 0;

  //Find first good line
  while(i < n_lines && (isnan(dist2[i]) || dist2[i] > goodLineThresh)){ ++i; }

  if(i >= n_lines) return;

  do{
    //Find first bad line
    while(i < n_lines && !isnan(dist2[i]) && dist2[i] < goodLineThresh){++i;}

    if(i >= n_lines) return;

    //The one before was good
    lastGood = i-1;

    //Now find good again
    while(i < n_lines && (isnan(dist2[i]) || dist2[i] > goodLineThresh))
     { ++i; }

    if(i >= n_lines){//No more good ones left
      //Just check last good if it is convex hidden
      checkConvexHidden(lastGood, false, true);
      return;
    }

    bool found = false;
    const Line *l1, *l2;

    l1 = &lines[lastGood]; l2 = &lines[i];

    if(i - lastGood < 2*n_fit){
//      printf("%%Corner suspect  -- %03d %03d\n", lastGood+1, i + 1);
// 	  printf("   %%Lines: (%g,%g,%g) (%g,%g,%g)\n"
// 		 ,l1->A(), l1->B(), l1->C()
// 		 ,l2->A(), l2->B(), l2->C());

      double dot;
      double d1,d2;
      int i1,i2;
      i1 = lastGood + n_fit - 1;
      i2 = i;

#if 0
      if(i1 >= n_data || i1 < 0 || i2 >= n_data || i2 < 0
         || lastGood < 0 || lastGood >= n_lines 
	 || i < 0 || i >= n_lines
         ){
	ABORT("i1=%d, i2=%d (%d),l1=%d,l2=%d (%d)\n"
              ,i1,i2,n_data, lastGood,i, n_lines);
      }
#endif

      //      printf("i1=%d, i2=%d,l1=%d,l2=%d\n",i1,i2,lastGood,i);
      d1 = l2->distance(x[i1],y[i1]);
      d2 = l1->distance(x[i2],y[i2]);

      dot = l1->A()*l2->A() + l1->B()*l2->B();

      if(fabs(dot) > COLLINEAR_THRESH){
	//Collinear lines
	//	printf("   %%Suspect colliniear (jump)\n");

	found = checkJump(lastGood, i, i1,i2,d1,d2);


      }else if(fabs(dot) < CONVEX_ANGLE_THRESH){
        //Intersecting lines
	found = checkConvexConcave(lastGood, i, i1,i2, d1, d2);
      }

    }

    if(!found){
    //      printf("%%Suspect Convex Hidden (after)  %03d\n",lastGood + n_fit);
      checkConvexHidden(lastGood, false, true);

    //      printf("%%Suspect Convex Hidden (before) %03d\n", i + 1);
      checkConvexHidden(i, true, false);
    }

  }while(1);

}

//Check if the line is long enough or if it can be extended.
// dir should -1 or 1
bool LaserCornerDetector::checkLineLength(int il, int dir, 
                                          double *xt, double *yt){
  double len2;
  double dx,dy;
  double x1,y1,x2,y2;

  if(dir > 0){
    x1 = x[il];
    y1 = y[il];
    x2 = x[il + n_fit - 1];
    y2 = y[il + n_fit - 1];
  }else{
    x2 = x[il];
    y2 = y[il];
    x1 = x[il + n_fit - 1];
    y1 = y[il + n_fit - 1];
  }

  dx = x2 - x1;
  dy = y2 - y1;
  len2 = dx*dx + dy*dy;

  if(len2 > minLineLen2){
    //Line is long enough just compute the tangents
    const Line *l = lines + il;
    bool prop_a;
    bool prop_b;

    if(fabs(dx) > fabs(dy)){
      prop_a = dx < 0;
      prop_b = l->B() < 0;
    }else{
      prop_a = dy < 0;
      prop_b = l->A() > 0;
    }

    if( prop_a^prop_b ){
      *xt = -l->B();
      *yt =  l->A();
    }else{
      *xt =  l->B();
      *yt = -l->A();
    }

//     printf("%%Line%03d: dx=%g,dy=%g,xt=%g,yt=%g,a=%g,b=%g,dir=%d\n"
// 	   ,il,dx,dy ,*xt,*yt,l->A(),l->B(),dir);

    return true;
  }

  //Line is short try to extend it

  Line2DCartDataType l;

  if(dir < 0){
    FindAdjLine2D( x, y, il + n_fit - 1, 0, 0, &lineFitParams, &l);
  }else{
    FindAdjLine2D( x, y, il       , n_data, 0, &lineFitParams, &l);
  }

  if(l.prob > params.minLineProb){
    *xt = l.xt; *yt = l.yt;
    double a,b,c;

    c = l.xs*l.yt - l.ys*l.xt;
    if(c > 0){
      c = -c;
      a =  l.yt;
      b = -l.xt;
    }else{
      a = -l.yt;
      b =  l.xt;
    }

    lines[il].set(a,b,c);

    return true;
  }

  return false;
}

bool LaserCornerDetector::checkJump(int il1, int il2, 
				    int i1,  int i2,
				    double d1, double d2){

  if(fabs(d1) < params.minJumpDist || fabs(d2) < params.minJumpDist){
    //	  printf("    %%Jump failed: dist %g,%g\n",d1,d2);
    return false;
  }

  double xt1 = 0;
  double yt1 = 0;
  double xt2 = 0;
  double yt2 = 0;

  //Check line legth
  if(checkLineLength(il1,-1, &xt1, &yt1) && 
     checkLineLength(il2, 1, &xt2, &yt2)){

    if( fabs( lines[il1].A()*x[i2] + lines[il1].B()*y[i2] ) 
	> fabs(lines[il1].C()) ){

      addCorner(DROS_CORNER_JUMP, i1, il1, xt1, yt1);
    }else{
      addCorner(DROS_CORNER_JUMP, i2, il2, xt2, yt2);
    }


    return true;
  }

  return false;
}

bool LaserCornerDetector::checkConvexConcave(int il1, int il2, 
                                             int i1,  int i2,
                                             double d1, double d2){

  double x0,y0;
  double dx,dy;

  if(fabs(d1) > params.maxConvexDist ||
     fabs(d2) > params.maxConvexDist){
//	  printf("   %%Convex failed: Lines are far apart (%g,%g)\n",d1,d2);
    return false;
  }

  double xt1,yt1;
  double xt2,yt2;
  xt1 = yt1 = xt2 = yt2 = 0.0;

  //Check line legth
  if(checkLineLength(il1,-1, &xt1, &yt1) && 
     checkLineLength(il2, 1, &xt2, &yt2)){

    lines[il1].intersect(&x0,&y0, lines[il2]);
    //	  printf("   %%Convex or Concave @ (%+.3f,%+.3f)\n",x0,y0);

    dx = x[i1] - x0;
    dy = y[i1] - y0;
    d2 = dx*dx + dy*dy;
    if( d2 > maxConvexDist2) return false;

    dx = x[i2] - x0;
    dy = y[i2] - y0;
    d2 = dx*dx + dy*dy;
    if( d2 > maxConvexDist2) return false;

    int type;

    double da = - xt2*yt1 + yt2*xt1; // da = sin(a2 - a1);

    if(da > 0){
      type = DROS_CORNER_CONVEX;
    }else{
      type = DROS_CORNER_CONCAVE;
    }

    addCorner(type,x0,y0, xt1,yt1, xt2,yt2);

    return true;
  }

  return false;
}


bool LaserCornerDetector::checkRay(const Line *l, int i_ray){
  double r;
  double cos_a;

  if(i_ray >= 0 && i_ray < n_data){

    cos_a = -sa[i_ray]*l->A() + ca[i_ray]*l->B();

    if(fabs(cos_a) > HIDDEN_RAY_THRESH){
       return false;
    }

    r = -l->C()/(l->A()*ca[i_ray] + l->B()*sa[i_ray]);

    if(range[i_ray] - r > params.minRayPenetration){

      r = l->distance(x[i_ray], y[i_ray]);

      if(fabs(r) > 0.5*params.minRayPenetration){
	return true;
      }
    }
  }

  return false;
}

void LaserCornerDetector::checkConvexHidden(int iline, 
                                            bool before, bool after){
  int i_ray;
  const Line* l = lines + iline;
  double xt,yt;

  xt = yt = 0;

  if(before){
    i_ray = iline - 1;

    //Check line legth
    if(checkLineLength(iline,1,&xt,&yt)){
      if(checkRay(l,i_ray) || checkRay(l,i_ray - 1) ){
	// 	printf("   %%Convex Hidden @ %d  (%d, %g,%g)\n"
	// 	       , iline + 1,i_ray+1, r, range[i_ray]);

	addCorner(DROS_CORNER_CONVEX_HIDDEN, iline,iline, xt, yt);
      }
    }
  }

  if(after){
    i_ray = iline + n_fit;

    if(checkLineLength(iline,-1,&xt,&yt)){
      if(checkRay(l,i_ray) || checkRay(l,i_ray+1)){
	// 	printf("   %%Convex Hidden @ %d  (%d, %g,%g)\n"
	// 	       , iline + n_fit,i_ray+1, r, range[i_ray]);
	addCorner(DROS_CORNER_CONVEX_HIDDEN, iline + n_fit - 1,iline, xt, yt);
      }
    }
  }
}


void LaserCornerDetector::addCorner(int type, int i_data, int il,
                                    double xt1, double yt1){

  double x0,y0;

  lines[il].project(&x0,&y0,x[i_data],y[i_data]);

  addCorner(type,x0,y0,xt1,yt1,0,0);
}

void LaserCornerDetector::addCorner(int type, double x, double y, 
                                    double xt1, double yt1,
                                    double xt2, double yt2){
  if(n_corners >= sz_corners){    //Resize:
    if(corners == NULL){
      sz_corners = n_corners + 10;
      corners    = new struct Corner[sz_corners];
    }else{
      sz_corners = n_corners + 10;
      struct Corner * cc = new struct Corner[sz_corners];
      memcpy(cc, corners, n_corners*sizeof(struct Corner));

      delete[] corners;
      corners = cc;
    }
  }

  corners[n_corners].type  = type;
  corners[n_corners].x     = x;
  corners[n_corners].y     = y;
  corners[n_corners].range = sqrt(x*x + y*y);
  corners[n_corners].angle = atan2(y,x);

  corners[n_corners].xt1 = xt1;
  corners[n_corners].xt2 = xt2;
  corners[n_corners].yt1 = yt1;
  corners[n_corners].yt2 = yt2;

  n_corners += 1;
}




/*===========================================================================*/
//Copied from DROSVector.h
inline void Unitise2D(double *x, double *y) {
  double l;
  l = sqrt(*x * *x + *y * *y);

  if (l != 0.0) {
    *x = *x / l;
    *y = *y / l;
  }
}

/*===========================================================================*/

void Project2D_unit(double x, double y, double ox, double oy,
	       double *px, double *py) {
  //ox,oy should be a unit vector i.e. >> ox^2 + oy^2 == 1
  double dot;

  dot = x * ox + y * oy;
    
  *px = ox * dot;
  *py = oy * dot;
}


/****************************************************
 ** FindAdjLine2D

Grow the line from point 'start' to the left or to the right, 
to find the most likely line.

 * x,y   -- data Points
 * start -- First data point to use
 * nData -- Either maximum number of points in the array if growing
            along the increasing index. 
            OR
            smallest index to use if growing towards zero index.
 
 * debug -- Debug level to use.
 * p     -- Line fitting parameteres
 * l     -- Pointer to the line (the output)
 *
 * Return Value: probability of the line
 *
 * Example: you have 361 data points want to fit a line starting at index 111
 *      to fit line along increasing index use start=111 nData=361
 *      to fit line towards zero use           start=111 nData=0
 *      to fit line towards zero, but ignoring the first 30 data points
 *                               use           start=111 nData=30  
 */

double FindAdjLine2D(const double *x, const double *y, unsigned int start,
                     unsigned int nData, 
                     int debug, LineFittingParamType *p,
                     Line2DCartDataType *l){
  unsigned int i;
  unsigned int j;

  double dx;
  double dy;
  const double *xn;
  const double *yn;
  int swapped;
  double xSum;
  double x2Sum;
  double ySum;
  double y2Sum;
  double xySum;
  double b;
  double m;
  double det;
  double err;
  double len;
  double len2;
  double prob_l;
  double prob_e;
  double prob;
  double t;
  double spaceProb;
  double d2;
  double maxd2;
  double xm;
  double ym;

  unsigned int n;

  int ind;
  int dir;

  double minDistForLine2  = p->minDistForLine*p->minDistForLine;
  double maxDistForLine2  = p->maxDistForLine*p->maxDistForLine;
  double ptSpacingThresh2 = p->ptSpacingThresh*p->ptSpacingThresh;
  double maxPtSpacing2    = p->maxPtSpacing*p->maxPtSpacing;

  unsigned int minPoints;

  if(nData < start){
    nData = start - nData + 1;
    dir = -1;
  }else{
    nData = nData - start;
    dir = 1;
  }
  x = x + start; 
  y = y + start;

  //First number of points that satisfy the good line
  minPoints = p->minPtsForLine - 1;
  maxd2 = 0;

  for(i = 1; i < minPoints && i < nData && maxd2 < maxPtSpacing2; ++i){
    ind = i*dir;

    if(isnan(x[ind])){
      maxd2 = maxPtSpacing2 +1;
      break;
    }

    d2 = (x[ind] - x[ind - dir])*(x[ind] - x[ind - dir]) + 
         (y[ind] - y[ind - dir])*(y[ind] - y[ind - dir]);

    if(d2 > maxd2){ maxd2 = d2; }
  }

  len2 = 0;
  dx = dy = 0.0;

  while(minPoints < nData && len2 < minDistForLine2 && maxd2 < maxPtSpacing2){
    ind = minPoints*dir;

    dx = x[0] - x[ind];
    dy = y[0] - y[ind];
    len2 = dx*dx + dy*dy;

    if(isnan(x[ind])){
      maxd2 = maxPtSpacing2 +1;
      break;
    }

    d2 = (x[ind] - x[ind - dir])*(x[ind] - x[ind - dir]) + 
         (y[ind] - y[ind - dir])*(y[ind] - y[ind - dir]);

    if(d2 > maxd2){ maxd2 = d2; }

    minPoints += 1;
  }

  l->xs = 0.0;
  l->ys = 0.0;
  l->xe = 0.0;
  l->ye = 0.0;
  l->xt = 0.0;
  l->yt = 0.0;
  l->start = 0;
  l->nPoints = 0;
  l->prob = 0.0;

  if(len2 < minDistForLine2 || maxd2 > maxPtSpacing2 || 
     len2 > maxDistForLine2){
    return 0.0;
  }

  if(fabs(dx) > fabs(dy)){
    swapped = FALSE;
    xn = x; yn = y;
  }else{
    swapped = TRUE;
    xn = y; yn = x;
  }

  xSum = x2Sum = ySum = xySum = 0.0;

  for(i = 0; i < minPoints - 1; ++i){
    ind    = i*dir;
    xSum  += xn[ind];
    x2Sum += xn[ind] * xn[ind];
    ySum  += yn[ind];
    y2Sum += yn[ind] * yn[ind];
    xySum += xn[ind] * yn[ind];
  }

  if(maxd2 > ptSpacingThresh2){
    spaceProb = 1.0 - (sqrt(maxd2) - p->ptSpacingThresh) * p->ptSpacingScale;
  }else{
    spaceProb = 1.0;
  }

  for(i = minPoints - 1; i < nData; ++i){
    ind = i*dir;
    n = i + 1;

    // Treat NaN as huge gaps
    if(isnan(x[ind])){
      maxd2 = maxPtSpacing2 + 1;
      break;
    }

    d2 = (x[ind] - x[ind - dir])*(x[ind] - x[ind - dir]) + 
         (y[ind] - y[ind - dir])*(y[ind] - y[ind - dir]);

    if(d2 > maxd2){ 
       maxd2 = d2;
       //Recompute spaceProb
       if(maxd2 > ptSpacingThresh2){
         spaceProb = 1.0 - (sqrt(maxd2) - p->ptSpacingThresh) * 
                            p->ptSpacingScale;
       }else{
         spaceProb = 1.0;
       }
    }

    if(maxd2 > maxPtSpacing2 
      || spaceProb <= 0.0){ break; } //This line is worse than previous

    dx = x[0] - x[ind];
    dy = y[0] - y[ind];
    len2 = dx*dx + dy*dy;

    if(len2 > maxDistForLine2){ break; } //This line is too long

    len = sqrt(len2);

    if (len > p->maxProbScaleLen) {
      prob_l = 1;
    } else {
      prob_l = (len - p->minDistForLine) / 
        (p->maxProbScaleLen - p->minDistForLine) 
        * p->lineLenProbScale + 1.0 - p->lineLenProbScale;
    }

    //Fit the line
    xSum  += xn[ind];
    x2Sum += xn[ind] * xn[ind];
    ySum  += yn[ind];
    y2Sum += yn[ind] * yn[ind];
    xySum += xn[ind] * yn[ind];

    det = x2Sum * n - xSum * xSum;
    b = (x2Sum * ySum + (-xSum * xySum)) / det;
    m = ((-xSum * ySum) + n * xySum) / det;
    xm = xSum / n;
    ym = ySum / n;

    //Compute point->line error
    err = 0;
    for(j = 0; j < n; ++j){
      t = fabs(yn[j*dir] - m*xn[j*dir] - b);
      if(t > p->ptErrorThresh){
        err += t;
      }
    }
  
    prob_e = spaceProb * exp(- p->errorLenScale * err / len);
    prob = prob_l*prob_e;

    if(prob < l->prob) break; //This line is worse

    //Set the best line
    if (swapped) {
      if (xn[0] < xn[ind]) {
        l->xt = m;
        l->yt = 1.0;
      } else {
        l->xt = -m;
        l->yt = -1.0;
      }
      Unitise2D(&(l->xt), &(l->yt));

      Project2D_unit(yn[0] - ym, xn[0] - xm, l->xt, l->yt, 
                &(l->xs), &(l->ys));
      l->ys += xm;
      l->xs += ym;

      Project2D_unit(yn[ind] - ym, xn[ind] - xm, l->xt, l->yt, 
                &(l->xe), &(l->ye));
      l->ye += xm;
      l->xe += ym;
    } else {
      if (xn[0] < xn[ind]) {
        l->xt = 1.0;
        l->yt = m;
      } else {
        l->xt = -1.0;
        l->yt = -m;
      }
      Unitise2D(&(l->xt), &(l->yt));

      Project2D_unit(xn[0] - xm, yn[0] - ym, l->xt, l->yt, 
                &(l->xs), &(l->ys));
      l->xs += xm;
      l->ys += ym;

      Project2D_unit(xn[ind] - xm, yn[ind] - ym, l->xt, l->yt, 
                &(l->xe), &(l->ye));
      l->xe += xm;
      l->ye += ym;
    }

    l->prob = prob;
    l->nPoints = i+1;
//     if (debug >= 5000) {
//       Print2DLine(l);
//     }
  }

  return l->prob;
}



#if 1
static LaserCornerDetector* cornerDetector = NULL;

void initLaserCornerExtractor(){
  cornerDetector = new LaserCornerDetector();
}

int extractCorners(ObservationInterface **obs, 
		   const LaserSource &las){
    
  int i;
  int  nc = cornerDetector->ExtractCorners(las);

  for(i = 0; i < nc; ++i){
    double srr,saa;
    double range   = cornerDetector->R(i);
    double bearing = angleDiffRad(cornerDetector->A(i), M_PI/2);

    int type = cornerDetector->CornerType(i);
    double a1,a2,sa2;
    double sa1 = POW2(deg2rad(3));

    a1 = atan2(cornerDetector->YTangentLeft(i), 
               cornerDetector->XTangentLeft(i));
    a1 = angleDiffRad(a1, M_PI/2);

    a2 = 0;
    sa2 = 0;
    CornerType t;

    switch(type){
    case DROS_CORNER_JUMP:
      t = JUMP;
      srr = POW2(0.1) + POW2(0.05*range);
      saa = POW2(deg2rad(3));
      break;

    case DROS_CORNER_CONVEX_HIDDEN:
      t = CONVEX_HIDDEN;
      srr = POW2(0.1) + POW2(0.05*range);
      saa = POW2(deg2rad(3));
      break;

    case DROS_CORNER_CONCAVE:
      t = CONCAVE;
      srr = POW2(0.05) + POW2(0.01*range);
      saa = POW2(deg2rad(2));
      a2 = atan2(cornerDetector->YTangentRight(i), 
		 cornerDetector->XTangentRight(i));
      a2 = angleDiffRad(a2, M_PI/2);
      sa2 = sa1;
      break;

    case DROS_CORNER_CONVEX: //fall through
    default:
      t = CONVEX;
      srr = POW2(0.05) + POW2(0.01*range);
      saa = POW2(deg2rad(2));
      a2 = atan2(cornerDetector->YTangentRight(i), 
		 cornerDetector->XTangentRight(i));
      a2 = angleDiffRad(a2, M_PI/2);
      sa2 = sa1;
    }

    //TODO: add observable side
    obs[i] = new CornerObservation(range, srr, bearing, saa
                 , t, a1, sa1, 0, a2, sa2, 0);
  }
  return nc;
}



#else
#include "LaserCornerDetectorMain.h"

static LaserCornerDetectorMain* cornerDetector = NULL;

const double CORNER_THRESHOLD = 0.19;


void initLaserCornerExtractor(){
  cornerDetector = new LaserCornerDetectorMain();
}

int extractCorners(double *range, double *bearing, 
		   const LaserSource &las){

  int n = 0;
  double *x = new double[las.numPoints()];
  double *y = new double[las.numPoints()];

  int i;

  for(i = 0; i < las.numPoints(); ++i){
    if(las.isValidReading(i) && !las.noReturn(i)){
      x[n] = las.x(i);
      y[n] = las.y(i);
      n += 1;
    }
  }
  
  int nc = 0;

  if(n > 0){

    cornerDetector->ClearCorners();
    
    nc = cornerDetector->ExtractCorners(x,y,n, CORNER_THRESHOLD);

    for(i = 0; i < nc; ++i){
      double xx = cornerDetector->X(i);
      double yy = cornerDetector->Y(i);

      double r2 = xx*xx + yy*yy;

      range[i] = sqrt(r2);
      bearing[i] = angleDiffRad(atan2(yy,xx), M_PI/2);

      //     printf("Corner: (%.2f, %.2f ) -->%.2f %.2f (deg)\n"
      //        , xx,yy, range[i], bearing[i]*RADTODEG);
    }
  }

  delete[] x;
  delete[] y;

  return nc;

}


int extractCorners(ObservationInterface **obs, 
		   const LaserSource &las){

  int n = 0;
  double *x = new double[las.numPoints()];
  double *y = new double[las.numPoints()];

  int i;

  for(i = 0; i < las.numPoints(); ++i){
    if(las.isValidReading(i) && !las.noReturn(i)){
      x[n] = las.x(i);
      y[n] = las.y(i);
      n += 1;
    }
  }
  
  int nc = 0;

  if(n > 0){

    cornerDetector->ClearCorners();
    
    nc = cornerDetector->ExtractCorners(x,y,n, CORNER_THRESHOLD);

    for(i = 0; i < nc; ++i){
      double xx = cornerDetector->X(i);
      double yy = cornerDetector->Y(i);

      double r2 = xx*xx + yy*yy;

      double srr,saa;
      double range   = sqrt(r2);
      double bearing = angleDiffRad(atan2(yy,xx), M_PI/2);

      int type = cornerDetector->CornerType(i);
      switch(type){
      case DROS_CORNER_JUMP:
      case DROS_CORNER_CONVEX_HIDDEN:
	srr = POW2(0.1) + POW2(0.05*range);
	saa = POW2(deg2rad(1.5));
	break;
      default:
	srr = POW2(0.05) + POW2(0.01*range);
	saa = POW2(deg2rad(1));
      }

      obs[i] = new Landmark2dObs(range, srr, bearing, saa);
    }
  }

  delete[] x;
  delete[] y;

  return nc;
}
#endif
