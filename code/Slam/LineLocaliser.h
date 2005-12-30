#ifndef __LINE_LOCALISER_H__
#define __LINE_LOCALISER_H__

#include "util.h"
#include "geometry.h"
#include "pfilter.h"
#include "map.h"

class LineMap{
 private:
  int nlines;
  LineSegment *lines;
  int *n_observed;
  int *n_conflict;

 public:
  LineMap():nlines(0),lines(NULL),n_observed(NULL),n_conflict(NULL){;}

  ~LineMap(){ 
    DESTROY_ARRAY(lines); 
    DESTROY_ARRAY(n_observed); 
    DESTROY_ARRAY(n_conflict); 
   }

  bool load(FILE* f);

  int numLines()const{ return nlines;}
  const LineSegment & line(int i)const{ return lines[i]; }

  const LineSegment & operator[](int i)const{ return lines[i]; }

  void observe(int i) {n_observed[i] += 1;}
  void conflict(int i){n_conflict[i] += 1;}

  void resetScores(){
    memset(n_observed,0, nlines*sizeof(int));
    memset(n_conflict,0, nlines*sizeof(int));
  }

  int numObserved(int i){ return n_observed[i];}
  int numConflict(int i){ return n_conflict[i];}

};

class LineObservation: public ObservationInterface{
 public:
  int n;
  double *range;
  double *angle;

  LineObservation():n(0),range(NULL),angle(NULL){;}

  ~LineObservation(){
    DESTROY_ARRAY(range);
    DESTROY_ARRAY(angle);
  }

  void setMaxSize(int sz){
    DESTROY_ARRAY(range);
    DESTROY_ARRAY(angle);

    range = new double[sz];
    angle = new double[sz];
  }
};

class LineLocaliserParticle: public ParticleInterface{
 private:
  LineMap *map;
  const struct LineLaserMatchParams* params;

 public:
  LineLocaliserParticle():map(NULL),params(NULL){;}

  LineLocaliserParticle(const LineLocaliserParticle* other):
    map(other->map),params(other->params){
    ParticleInterface::set(other);
  }

  ParticleInterface* clone()const{
    return new LineLocaliserParticle(this);
  }

  void evaluate(double dt,const ObservationStore &obs_store);

  void setMap(LineMap *M){ map = M; }
  void setLineParams(const  struct LineLaserMatchParams* p){params = p;}

  LineMap * getMap()const{ return map; }
};

class LineLocaliser: public ParticleFilter{
 private:
  void computeCumSum();

 public:
  LineLocaliser(){;}

  void init(int np, const MotionModelInterface *m,
	    LineMap *map,const  struct LineLaserMatchParams* params);

  void setPose(int i, const RobotPose &x){
    particles[i]->setPose(x);
  }

  void getPose(RobotPoseCov* out)const;

  void evaluate(double TimeStamp, 
		const ObservationStore &obs_store);

  void resample();
  void resample(int newSz);

};



#endif
