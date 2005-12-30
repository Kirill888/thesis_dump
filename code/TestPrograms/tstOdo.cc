#include "odoXR4000.h"
#include "DataSource.h"

void robot2sensor(RobotPose*,const RobotPose*){
}

int main(int argc, char *argv[]){

  if(argc < 3){
    fprintf(stderr,"Usage: %s odo np [nodo]\n", argv[0]);
    return 1;
  }

  FileOdoSource fodo(argv[1]);
  int np   = 100;
  int nodo = -1;


  if(!fodo.isOk()){
    fprintf(stderr,"Failed to open: %s\n", argv[1]);
    return 1;
  }

  if(argc > 2) np = atoi(argv[2]);
  if(argc > 3) nodo = atoi(argv[3]);

  int n = 0,i;

  MotionModel robot;

  robot.setP1(0,0,POW2(0.1));             //time    =f(t)
  robot.setP1(1,1,POW2(0.25));            //rotaion =f(rot)
  robot.setP1(1,2,POW2(deg2rad(1)));      //        =f(dist)
  robot.setP1(2,2,POW2(0.5));             //distance=f(dist)
  robot.setP1(2,1,POW2(0.01/deg2rad(1))); //distance=f(rot)

  OdometryStore store[np + 1];

  for(i = 0; i < np  + 1; ++i){
    store[i].add(RobotPose(0,0,0));
  }

  while(fodo.next()){

    fprintf(stderr,"ODO: %.2f sec %+10.2f m %+10.2f m %+10.2f (deg)\n"
	   ,fodo.TimeStamp()
	   ,fodo.x()
	   ,fodo.y()
	   ,fodo.a()*RADTODEG);

    RobotPoseOdo o(fodo, fodo.TimeStamp());

    robot.newOdometry(&o);

    if(robot.hasMoved()){

      for(i = 0; i < np; ++i){
	RobotPose p = store[i].getLastPose();
	OdoControlInputInterface *u = robot.sampleControlInput();

	u->advance(&p);
	store[i].add(p);

	delete u;
      }

      //Update true path
      RobotPose p = store[np].getLastPose();
      const OdoControlInputInterface *u = robot.getControlInput();
      u->advance(&p);
      store[i].add(p);
    }

    n += 1;
    if(nodo > 0 && n >= nodo) break;
  }

  FILE *f = stdout;

  for(i = 0; i < np +1 ; ++i){
    RobotPose r = store[i].getLastPose();
    fprintf(f,"%e %e %e\n", r.x,r.y,r.rot);
  }

  store[np].dumpToFile(f);


  return 0;
}
