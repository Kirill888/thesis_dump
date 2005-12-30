#include "globalmap.h"
//#include "odoModel.h"
#include "landmark2d.h"

#define ERR(format,args...) fprintf(stderr,format, ##args)

const int NUM_POINTS_PER_SCAN = 361;
const double ANGLE_STEP = 0.5*M_PI/180.0;
const int LASER_FILE_LINE_LENGTH = 4*1024;
static const double LAS_MAX_RANGE  = 8.180;
static const double LAS_MIN_RANGE  = 0.010;
static const double LAS_NOT_AVALUE = -1.0;


//+1 for TimeStamp
double (*las)[NUM_POINTS_PER_SCAN + 1];
int    (*lasi)[NUM_POINTS_PER_SCAN];
int sz_las = 0;
int n_las = 0;

double (*xy)[2];
int *xyi;
int sz_xy = 0;
int n_xy  = 0;


bool loadLaser(const char* fname){
  FILE *f = fopen(fname,"r");
  if(f == NULL) return false;

  char buf[LASER_FILE_LINE_LENGTH];
  char *token;
  int i;
  int npoints = NUM_POINTS_PER_SCAN;

  n_las = 0;

  while(fgets(buf,LASER_FILE_LINE_LENGTH,f) != NULL){
    token = strtok(buf,"\t ");

    if(token == NULL){
      break;
    }
      
    las[n_las][0] = atof(token);

    for(i = 0, token = strtok(NULL,"\t ");
	i < npoints && token != NULL;
	++i, token = strtok(NULL,"\t ")){

      las[n_las][1 + i] = atof(token);
      if(las[n_las][1+i] >= LAS_MAX_RANGE || 
         las[n_las][1+i] <= LAS_MIN_RANGE){

         las[n_las][1 + i] = LAS_NOT_AVALUE;
      }
    }

    if( i != npoints){
      ERR("Incomplete Laser Range Data");
      fclose(f);
      return false;
    }

    for(i = 0;
	i < npoints && token != NULL;
	++i, token = strtok(NULL,"\t\n ")){
 
      lasi[n_las][i] = atoi(token);
    }

    if(i != npoints && i != 0){
      ERR("Incomplete Laser Intensity Data: %d\n",i);
      fclose(f);
      return false;
    }

    n_las += 1;
    if(n_las >= sz_las){
      ERR("Laser size reached maximum: %d\n",n_las);
      fclose(f);
      return false;
    }

  }

  fclose(f);
  return true;
}

extern RobotPose robot2laser(const RobotPose &r);

void addReading(const RobotPose &p, int lasId){
  //  printf("......AddReading: %.2f %.2f %.2f [%05d]\n",p.x,p.y,p.rot,lasId);
  int i;

  const double *r = &las[lasId][1];
  const int *intensity = &lasi[lasId][0];

  for(i = 0; i < NUM_POINTS_PER_SCAN; ++i){
    if(r[i] != LAS_NOT_AVALUE){
      RobotPose plaser = robot2laser(p);

      double a = angleDiffRad(i*ANGLE_STEP, -plaser.rot);
      xy[n_xy][0] = plaser.x + r[i]*cos(a);
      xy[n_xy][1] = plaser.y + r[i]*sin(a);
      xyi[n_xy] = intensity[i];

      n_xy += 1;
      if(n_xy >= sz_xy){
	ERR("Data point store is full. (%d)\n",n_xy);
	exit(0);
      }
    }
  }

}

void processMap(const LocalMap &map, const RobotPose & ref){
  int ipass, i;

  for(ipass = 0; ipass < map.numPasses(); ++ipass){
    const OdometryStore *odo = map.getPass(ipass);
    int nodo = odo->numOdo();
    printf("...Pass[%d] = %d\n",ipass, nodo);

    int t[nodo];
    RobotPose p[nodo];

    odo->copyToArray(p,t,nodo);

    for(i = 0; i < nodo; ++i){
      p[i].translateMe(ref);
      addReading(p[i],t[i]);
    }
  }
}

bool load_map_ref(const char* fname, RobotPose **pose_out, int *n_out){
  int i;
  FILE *f = fopen(fname,"r");

  if(f == NULL) return false;

  MatlabVariable m;

  if(m.read(f) != MatlabVariable::noError) return false;

  if(m.numCols() < 3) return false;

  *n_out = m.numRows();
  *pose_out = new RobotPose[*n_out];

  RobotPose *p = *pose_out;

  for(i = 0; i < *n_out; ++i){
    p[i].x   = m(i,0);
    p[i].y   = m(i,1);
    p[i].rot = m(i,2);
  }

  return true;

}

bool storeResults(const char* fname){
  FILE *f = fopen(fname,"w");

  if(f == NULL) return false;

  int i;
  for(i = 0; i < n_xy; ++i){
    fprintf(f,"%.3e %.3e %d\n", xy[i][0], xy[i][1], xyi[i]);
  }

  fclose(f);

  return true;
}

extern SimpleMap* Landmark2d_LoadMap(const MatlabVariable&);

int main(int argc, char *argv[]){

  if(argc != 5){
    fprintf(stderr,"Usage: %s map map_poses laser_log out \n", argv[0]);
    return 1;
  }

  LMAP_LoadMap = Landmark2d_LoadMap;
  LMAP_MapAreaFactory = new Landmark2dMapAreaFactory();

  GlobalMap gm;
  printf("Loading global map: %s ...",argv[1]); fflush(stdout);

  if(!gm.load(argv[1])){
    printf("failed.\n");
    return 1;
  }
  printf("done\n");

  //  gm.setReferenceFrame(0);
  RobotPose *map_ref;
  int n_ref;

  printf("Loading Map Poses: %s ...", argv[2]);
  if(!load_map_ref(argv[2],&map_ref, &n_ref)){
    printf("failed.\n");
    return 2;
  }
  if(n_ref != gm.numMaps()){
    printf("error\nError: size do not match map %d, poses %d\n",
	   gm.numMaps(), n_ref);
    return 2;
  }
  printf("done.\n");

  printf("Loading Laser Data: %s...", argv[3]); fflush(stdout);

  sz_las = 6000;
  las  = new double[sz_las][1 + NUM_POINTS_PER_SCAN];
  lasi = new    int[sz_las][NUM_POINTS_PER_SCAN];

  sz_xy = sz_las*NUM_POINTS_PER_SCAN;
  xy = new double[sz_xy][2];
  xyi = new int[sz_xy];

  if(! loadLaser(argv[3])){
    printf("failed.\n");
  }

  printf("done.[%d scans]\n",n_las);

  int i;

  for(i = 0; i < gm.numMaps(); ++i){
    printf("Map: %02d (%.3f %.3f %.3f)\n",i
           ,map_ref[i].x
           ,map_ref[i].y
           ,map_ref[i].rot
           );
    processMap(gm[i],map_ref[i]);
  }

  printf("Number of Laser Points: %d\n",n_xy);
  printf("Storing to %s", argv[4]); fflush(stdout);

  if(!storeResults(argv[4])){
    printf("...failed.\n");
  }

  printf("...done.\n");

  delete[] las; 
  delete[] xy;
  return 0;
}









