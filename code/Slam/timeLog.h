#ifndef __TIME_LOG_H__
#define __TIME_LOG_H__

#ifndef DEPEND
#include <stdio.h>
#include <string.h>
#endif

#include "TimeTools.h"
#include "util.h"

class TimeLog{

 public:
  TimeLog(){ 
   f = NULL; 
   n_timers = 0;
   timers = NULL;
   timer_n = NULL;
  }

  TimeLog(FILE *out, int nt){
    init(out,nt);
  }

  TimeLog(const char* file, int nt){
    f = NULL; 
    n_timers = 0;
    timers = NULL;
    timer_n = NULL;
    init(file, nt);
  }

  ~TimeLog(){
    DESTROY_ARRAY(timers);
    DESTROY_ARRAY(timer_n);
  }

  void start(int id){
    timers[id] = GetCurrentTime();
  }

  void stop(int id){
    double dt = GetCurrentTime() - timers[id];
    timer_n[id] += 1;
    
    if(f != NULL){
      fprintf(f,"timer_%02d(%d,1:2) = [%.8e %.8e];\n"
               ,id,timer_n[id],dt,timers[id] - T0);
    }
  }

  void stop(int id, double extraInfo, const char* comment = NULL){
    double dt = GetCurrentTime() - timers[id];
    timer_n[id] += 1;
    
    if(f != NULL){
      if(comment != NULL){
	fprintf(f,"timer_%02d(%d,1:3) = [%.8e %.8e %8e]; %%%s\n"
		,id,timer_n[id],dt,timers[id] - T0
		,extraInfo, comment);
      }else{
	fprintf(f,"timer_%02d(%d,1:3) = [%.8e %.8e %8e];\n"
		,id,timer_n[id],dt,timers[id] - T0
		,extraInfo);
      }
    }
  }


  void close(){
    if(f != NULL){
      fclose(f);
      f = NULL;
    }
    DESTROY_ARRAY(timers);
    DESTROY_ARRAY(timer_n);
    n_timers = 0;
  }

  bool init(const char* file, int nt){
    f = fopen(file,"w");
    if(f == NULL) return false;
    init(f,nt);

    return true;
  }

  void init(FILE *out, int nt){
    f = out;
    n_timers = nt;
    timers = new double[n_timers];
    timer_n = new int[n_timers];
    memset(timer_n,0, n_timers*sizeof(int));
    T0 = GetCurrentTime();
  }

 private:
  FILE* f;
  double T0;
  double* timers;
  int* timer_n;
  int n_timers;
};

#if DO_TIME_LOG

extern TimeLog t_log;

#define TLOG_DECLARE TimeLog t_log;
#define TLOG_INIT(f) t_log.init((f),7)

#define TLOG_ODO_START          t_log.start(0)
#define TLOG_ODO_STOP           t_log.stop(0)

#define TLOG_LAS_START          t_log.start(1)
#define TLOG_LAS_STOP           t_log.stop(1)

#define TLOG_EVAL_START         t_log.start(2)
#define TLOG_EVAL_STOP          t_log.stop(2)

#define TLOG_RESAMPLE_START     t_log.start(3)
#define TLOG_RESAMPLE_STOP      t_log.stop(3)

#define TLOG_CHECK_START        t_log.start(4)
#define TLOG_CHECK_STOP         t_log.stop(4)

#define TLOG_MAPMATCH_START            t_log.start(5)
#define TLOG_MAPMATCH_STOP(m1,m2)      t_log.stop(5,(m1) + 0.001*(m2))

#define TLOG_LOOPCLOSE_START           t_log.start(6)
#define TLOG_LOOPCLOSE_STOP(m1,m2)     t_log.stop(6,(m1) + 0.001*(m2))


#else

#define TLOG_DECLARE
#define TLOG_INIT(f)

#define TLOG_ODO_START
#define TLOG_ODO_STOP

#define TLOG_LAS_START
#define TLOG_LAS_STOP

#define TLOG_EVAL_START
#define TLOG_EVAL_STOP

#define TLOG_RESAMPLE_START
#define TLOG_RESAMPLE_STOP

#define TLOG_CHECK_START
#define TLOG_CHECK_STOP

#define TLOG_MAPMATCH_START
#define TLOG_MAPMATCH_STOP(m1,m2)

#define TLOG_LOOPCLOSE_START
#define TLOG_LOOPCLOSE_STOP(m1,m2)

#endif



#endif





