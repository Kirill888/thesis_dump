#ifndef DEPEND
#include <stdio.h>
#include <time.h>
#endif

#include "debug.h"
#include "odoModel.h"
#include "DataStore.h"
#include "DataSource.h"
#include "odoXR4000.h"

int main(int argc, char *argv[]){
  int NP = 10;
  int i;

  if(argc != 3){
     fprintf(stderr,"Usage: %s odo.log np", argv[0]);
     return 1;
  }


  srand(time(NULL));

  FileOdoSource odo(argv[1]);
  NP = atoi(argv[2]);

  OdometryStore *odoStore = new OdometryStore[NP];
  MotionModel odoModel;
  bool first_odo = true;


  while(odo.next()){

     odoModel.newOdometry(odo.TimeStamp(), odo);

     DEBUG_print("ODO: %e %e %e\n",odo.x(), odo.y(), odo.a()*RADTODEG);

     if(first_odo){
       for(i = 0; i < NP; ++i){
         odoStore[i].add(odo);
       }

       first_odo = false;
     }else{
       for(i = 0; i < NP; ++i){
         OdoControlInput u = odoModel.sampleControlInput();
         odoStore[i].add(u.advance(odoStore[i].getLastPose()));
       }
     }

  }

  FILE *mfile = fopen("odo_test.m","w");

  if(mfile == NULL){
     fprintf(stderr,"Couldn't open matlab file, dumping to stdout\n");
     mfile = stdout;
  }

  const char *styles[][2] = { {"r." , "r-"}
                             ,{"g." , "g-"}
                             ,{"b." , "b-"}
                             ,{"m." , "m-"}
                             ,{"c." , "c-"}
                             ,{"k." , "k-"}
                             ,{"y." , "y-"}
		            };

  const int NSTYLES = 7;
  const int PLOT_STEP = 10;

  for(i = 0; i < NP; ++i){
     char var[256];


     sprintf(var,"odo_%03d",i);

     odoStore[i].matlabDump(mfile,var);

     int si = i % NSTYLES;

     fprintf(mfile,"plot_robot(%s(1:%d:size(%s,1),:),'%s','%s',0.05);"
                   "hold on; axis equal\n"
             ,var,PLOT_STEP, var
	     ,styles[si][0], styles[si][1]);
  }

  return 0;
  
}
