#ifndef DEPEND
#include <time.h>
#include <stdio.h>
#include <signal.h>
#endif

#include "AngleTools.h"
#include "TimeTools.h"
#include "odoXR4000.h"
#include "multiSLAM.h"
#include "landmark3d.h"
#include "simpleSLAM.h"
#include "TemplateStore.h"

extern void ekf_update(Gaussian3d *p, const Landmark3dObs *obs0, 
                const RobotPose *robot);
void run_ekf_test(const char* f_odo, const char* f_obs);


class MonitorMovieLog: public SLAMMonitor{
 private:
  const char* file_mask;
  int obsId;
  FILE *f;
  const SLAMParticle *bestP;


 public:
  MonitorMovieLog():file_mask("slam_%07d.m"), obsId(0),f(NULL),bestP(NULL){;}
  MonitorMovieLog(const char* mask):file_mask(mask), 
                                    obsId(0), f(NULL), bestP(NULL){;}

  void setMask(const char* mask){ file_mask = mask;}

  void startObservation(const SLAM *,
                        const ObservationStore &obs_store){
    obsId += 1;
    bestP = NULL;

    char buf[1024];
    sprintf(buf,file_mask,obsId);
    f = fopen(buf,"w");

    printf("Open file: %s ", buf);

    if(f == NULL){ 
      printf("[failed]\n");
      perror("error:");
      return;
    }

    printf("[ok]\n");

    int i;
    fprintf(f,"obs = [\n");
    bool ok = true;
    const ObservationInterface *const* scan = obs_store.getLastScan();

    for(i = 0; i < obs_store.numObsLastScan() && ok; ++i){
      ok = scan[i]->matlabDump(f);
      fprintf(f,"\n");
    }
    fprintf(f,"];\n"
              "odo = [\n");
   
  }

  void observation(const SLAMParticle* p, const RobotPose* r){
    if(f != NULL){
      fprintf(f,"%e %e %e %e\n", r->x,r->y,r->rot, p->getWeight());
      if(bestP == NULL) bestP = p;
      else if(bestP->getWeight() < p->getWeight()){
	bestP = p;
      }
    }
  }

  void endObservation(const SLAM *){
    if(f != NULL){
      fprintf(f,"];%%odo\n");
      if(bestP != NULL){
	RangeMature range;
	SimpleMap *m = bestP->getMap()->getSubMap(range);

	int i;
	fprintf(f,"map = [\n");
	for(i = 0; i < m->numElements(); ++i){
	  m->get(i)->matlabDump(f);
	  fprintf(f,"\n");
	}
	fprintf(f,"];\n");
	delete m;
      }

      fclose(f);
    }
  }

};


//Input
FILE *f_man = NULL;
FILE *f_odo = NULL;
FILE *f_obs = NULL;
//Output
FILE *f_mat = NULL;

bool stop = false;

//To catch ^C
void onKillSignal(int signal){
  printf("Received kill signal: exiting.\n");
  stop = true;
}

const RobotPose SENSOR_POSE(0.095,0.155,0);

void robot2sensor(RobotPose* sensor, const RobotPose *robot){
  sensor->set(SENSOR_POSE);
  sensor->translateMe(*robot);
}


const int SENSOR_ODO = 1;
const int SENSOR_OBS = 2;

ObservationStore obs0(100000, 10000);
int obsCount = 0;

MotionModel robot;
OdometryStore odoStore;

Landmark3dMappingProcess mapper;

MapBuilder DEFAULT_BUILDER;

SimpleSLAM Slam;

double T0 = -1;

bool odoEvent(FILE *f, double){
  double TimeStamp;
  double x,y,rot;

  if(fscanf(f,"%lf %lf %lf %lf", 
            &TimeStamp, &x, &y, &rot) != 4) return false;


  printf("ODO: %9.2fs/%9.2fs -- %9.3f %9.3f %9.2f deg\n"
         , TimeStamp, GetCurrentTime() -T0
         , x, y, angleDiffRad(rot,0)*RADTODEG);
  

  RobotPoseOdo odo(RobotPose(x,y,rot),TimeStamp);

  robot.newOdometry(&odo);
  Slam.odometry(&odo);

  return true;
}

int obsId = -1;

Landmark3dObs* readObs(FILE *f){
  double r, a, b;
  double COV[3][3];

  if(fscanf(f,"%lf %lf %lf"
           ,&r, &b, &a) != 3) return NULL;

  if(fscanf(f,"%lf %lf %lf %lf %lf %lf %lf %lf %lf"
	    ,&COV[0][0]	    ,&COV[0][1]	    ,&COV[0][2]
	    ,&COV[1][0]	    ,&COV[1][1]	    ,&COV[1][2]
	    ,&COV[2][0]	    ,&COV[2][1]	    ,&COV[2][2]) != 9) return NULL;

  obsId += 1;

  return new Landmark3dObs(r,b,a,COV,obsId);
}

#ifdef MAX_OBS
#undef MAX_OBS
#endif

const int MAX_OBS = 100;
int *obs_ind = new int[MAX_OBS];


bool obsEvent(FILE *f, double TimeStamp){

  static int last_frame = -1;

  int numObs = 0;
  int n = 0;
  int frame = 0;

  if(last_frame < 0){
    if(fscanf(f,"%d", &frame) != 1) return false;

    last_frame = frame;
  }else{
    frame = last_frame;
  }

  obs0.startScan();

  while(frame == last_frame){
    Landmark3dObs * obs = readObs(f);
    if(obs == NULL) return false;

    numObs += 1;

    obs_ind[n] = obs0.add(obs,frame);
    n += 1;

    if(fscanf(f,"%d", &frame) != 1){
      frame = last_frame + 1;
    }
  }

  last_frame = frame;

  if(n <= 0) return false;

  RobotPose p = robot.getTruePose(TimeStamp);
  odoStore.add(p);

  printf("OBS: %d (%10.2f sec) [# %d/%d]", frame, TimeStamp, n, numObs);


  if(!Slam.hasMoved()){
    Slam.storeOdo(TimeStamp, obs0.getLastScanId());
    printf("[no move]\n");
    return true;
  }

  printf("\n");

  if(n > 0) Slam.observation(TimeStamp, obs0, obs_ind, n);
  else Slam.storeOdo(TimeStamp, obs0.getLastScanId());

  return true;
}

void run_simulation(){
  double Time;
  int id;
  bool ok = true;

  while(!stop && ok && fscanf(f_man, "%lf %d", &Time, &id) == 2){
    switch(id){
    case SENSOR_ODO:
      ok = odoEvent(f_odo,Time);
      break;
    case SENSOR_OBS:
      ok = obsEvent(f_obs,Time);
      break;
    }
  }
}

extern TemplateStore *templateStore;

int main(int argc, char** argv){

  if(argc == 3){
    run_ekf_test(argv[1],argv[2]);
    return 1;
  }

  char *man_file = NULL;
  char *odo_file = NULL;
  char *obs_file = NULL;

  char *map_out = NULL;
  char *movie_mask = NULL;

  char* template_dir = "/home/users/kirill/Data/Vision/18042004_templates";

  int numP = 100;
  int seed = time(NULL);

  MonitorMovieLog monitor;


  if(argc > 1) man_file   = argv[1];
  if(argc > 2) odo_file   = argv[2];  
  if(argc > 3) obs_file   = argv[3];
  if(argc > 4) map_out    = argv[4];
  if(argc > 5) numP       = atoi(argv[5]);
  if(argc > 6) seed       = atoi(argv[6]);
  if(argc > 7) movie_mask = argv[7];

  if(odo_file == NULL || obs_file == NULL || man_file == NULL 
     || map_out == NULL){
    fprintf(stderr,"Usage: %s manager odo obs out [numP seed movie_mask]\n"
            ,argv[0]);
    return 1;
  }

  f_man = fopen(man_file,"r");
  if(f_man == NULL){
    fprintf(stderr,"Failed to open: %s\n", man_file);
    return 2;
  }

  f_odo = fopen(odo_file,"r");
  if(f_odo == NULL){
    fprintf(stderr,"Failed to open: %s\n", odo_file);
    return 2;
  }

  f_obs = fopen(obs_file,"r");
  if(f_obs == NULL){
    fprintf(stderr,"Failed to open: %s\n", obs_file);
    return 2;
  }

  char mat_file[1024];
  char map_file[1024];
  sprintf(mat_file,"%s_dat.m",map_out);
  sprintf(map_file,"%s_map.m",map_out);

  f_mat = fopen(mat_file,"w");
  if(f_mat == NULL){
    fprintf(stderr,"Failed to open: %s\n", mat_file);
    return 2;
  }

  templateStore = new TemplateStore(template_dir);

  printf("Starting SLAM:\n"
         "---manager     file: %s\n"
         "---odometry    file: %s\n"
         "---observation file: %s\n"
         "---template dir    : %s\n"
         "---Num. Particles  : %d\n"
         "---Seed            : %d\n"
         "---Output          : %s\n"
         ,man_file, odo_file, obs_file, template_dir, numP, seed, map_out
	 );

  fprintf(f_mat, "%%Starting SLAM:\n"
         "%%---manager     file: %s\n"
         "%%---odometry    file: %s\n"
         "%%---observation file: %s\n"
         "%%---template dir    : %s\n"
         "%%---Num. Particles  : %d\n"
         "%%---Seed            : %d\n"
         "%%---Output          : %s\n"
         ,man_file, odo_file, obs_file, template_dir, numP, seed, map_out
	 );
  fflush(f_mat);

  signal(SIGINT, onKillSignal);

  //Set Mapping Process parameters
  mappingProcess             = &mapper;
  mappingProcess->mapBuilder = &DEFAULT_BUILDER;
  mappingProcess->odoModel   = &robot;
  mappingProcess->setRobot2SensorFunction(robot2sensor);

  DEFAULT_BUILDER.setSensorRange(mappingProcess->sensorRange);

  Slam.NP_MIN = numP;
  Slam.NP_MAX = 10*numP;
  Slam.RESAMPLE_STEP = 5;

  Slam.setup(&robot, &DEFAULT_BUILDER);
  Slam.setSeed(seed);

  if(movie_mask != NULL){
    monitor.setMask(movie_mask);
    Slam.setMonitor(&monitor);
  }

  T0 = GetCurrentTime();
  Slam.start();
  run_simulation();

  printf("Storing Map\n");
  Slam.finalise();

  FILE* f = fopen(map_file,"w");
  if(f != NULL){
    Slam.getMap()->store(f);

    dumpDataAssociations(f, Slam.getMap()->getMap());

    Slam.getPath()->matlabDump(f,"odo");
    Slam.dumpOdo2Scan(f,"odo2scan");

    fclose(f);
  }else{
    printf("Failed to open: %s\n", map_file);
    Slam.getMap()->store(stdout);
    Slam.dumpOdo2Scan(stdout,"odo2scan");
  }

  printf("Storing Observations:\n");
  obs0.matlabDump(f_mat,"obs");

  printf("Storing Odometry:\n");
  odoStore.getReverse().matlabDump(f_mat, "odo");

  return 0;
}


void run_ekf_test(const char* f_odo, const char* f_obs){
  FILE *fodo = fopen(f_odo,"r");
  FILE *fobs = fopen(f_obs,"r");

  if(fodo == NULL || fobs == NULL){
    fprintf(stderr,"Failed to open files.\n");
    return;
  }

  double x,y,rot;
  double r,a,b;

  double Sr = 0.1/3;
  double Sa = 5*DEGTORAD/3;
  double Sb = 5*DEGTORAD/3;
  double Szz[3][3] = {{Sr*Sr, 0, 0},
                      {0, Sa*Sa, 0},
                      {0,0,Sb*Sb}};
  int nobs = 0;

  RobotPose robot;
  Landmark3dObs obs(0,0,0,Szz);
  Gaussian3d point;

  printf("%% EKF Test:\n");

  printf("obs = zeros(0,3); odo = zeros(0,3);\n");
  printf("obs0 = [");
  obs.matlabDump(stdout);
  printf("];\n ");

  while(fscanf(fodo,"%lf %lf %lf", &x, &y, &rot) == 3 &&
        fscanf(fobs,"%lf %lf %lf", &r, &a, &b) == 3){

    robot.set(x,y,rot);

    obs.range   = r;
    obs.bearing = a;
    obs.azimuth = b;

    printf(" odo(%d,:) = [%e, %e, %e];\n",nobs+1, x, y, rot);
    printf(" obs(%d,:) = [%e, %e, %e];\n",nobs+1, r, a, b);

    if(nobs == 0){
      point = obs.toCartesian(&robot);
    }else{
      ekf_update(&point,&obs,&robot);
      //printf("w(%d) = %e;\n",nobs+1,w);
    }

    printf("x   = [%e; %e; %e];\n", point.x, point.y, point.z);
    printf("Sxx = [%e, %e, %e; %e, %e, %e; %e, %e, %e];\n"
	   ,point.cov[0][0] ,point.cov[0][1], point.cov[0][2]
	   ,point.cov[1][0] ,point.cov[1][1], point.cov[1][2]
	   ,point.cov[2][0] ,point.cov[2][1], point.cov[2][2]
	   );

    nobs += 1;

//     double w;
//     Landmark3d lnd(point);
//     ObservationInterface *o = lnd.expectedObservation(&robot);
//     w = obs.logLikelihood(o);
//     printf("%% md2 = %e;\n",w),
//     delete o;
  }
}

