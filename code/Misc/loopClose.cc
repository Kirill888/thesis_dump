#ifndef DEPEND
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#endif

#include "mapMatch.h"
#include "globalmap.h"
#include "util.h"

#include "MappingAgent.h"



RobotPose propogate(const int* path, int npath, int map_start
		    ,const MapTransition *mp, const RobotPose &p0){
  int i;

  RobotPose p(p0);
  int current_map = map_start;

  for(i = 0; i < npath; ++i){
    mp[i].transit(&p,current_map, path[i]);
    current_map = mp[i].transit(current_map);
  }

  return p;
}

RobotPose propogate_rev(const int* path, int npath, int map_start
		       ,const MapTransition *mp, const RobotPose &p0){
  int i;

  RobotPose p(p0);
  int current_map = map_start;

  for(i = npath-1; i >= 0; --i){
    mp[i].transit(&p,current_map, path[i]);
    current_map = mp[i].transit(current_map);
  }

  return p;
}

RobotPoseCov propogate(const MapTransition *mp, int npath, int map_start){
  int i;
  RobotPoseCov p0;

  int current_map = map_start;
  for(i = 0; i < npath; ++i){
    mp[i].propagate(&p0,current_map);
    current_map = mp[i].transit(current_map);
  }

  return p0;
}

RobotPoseCov propogate_rev(const MapTransition *mp, int npath, int map_start){
  int i;
  RobotPoseCov p0;

  int current_map = map_start;
  for(i = npath-1; i >= 0; --i){
    mp[i].propagate(&p0,current_map);
    current_map = mp[i].transit(current_map);
  }

  return p0;
}


void dumpFinalPose(const char* fdump, const RobotPose *pFinal, int np){
  int i;

  printf("Saving to file: %s\n", fdump);
  FILE *f = fopen(fdump,"w");

  if(f == NULL){
    fprintf(stderr,"Failed to open file, dumping to stdout\n");
    f = stdout;
  }

  for( i = 0; i < np; ++i){
    fprintf(f,"%.9e %.9e %.9e\n"
	    ,pFinal[i].x
	    ,pFinal[i].y
	    ,pFinal[i].rot);
  }

  if(f != stdout) fclose(f);
}

void dumpFinalPose(const char* fdump, const RobotPose *pFinal, 
                   double *w, int np){
  int i;

  printf("Saving to file: %s\n", fdump);
  FILE *f = fopen(fdump,"w");

  if(f == NULL){
    fprintf(stderr,"Failed to open file, dumping to stdout\n");
    f = stdout;
  }

  for( i = 0; i < np; ++i){
    fprintf(f,"%.9e %.9e %.9e %.9e\n"
	    ,pFinal[i].x
	    ,pFinal[i].y
	    ,pFinal[i].rot
            ,w[i]);
  }

  if(f != stdout) fclose(f);
}

void dumpFinalPose(FILE *f, const RobotPose *pFinal, 
                   const double *w, int np){
  int i;

  for( i = 0; i < np; ++i){
    fprintf(f,"%.9e, %.9e, %.9e, %.9e\n"
	    ,pFinal[i].x
	    ,pFinal[i].y
	    ,pFinal[i].rot
            ,w[i]);
  }

}

void matlabDump(FILE *f, 
                int i1, int i2,
		const RobotPoseCov r1in2,
		const Gaussian2d & map_a1,
		const Gaussian2d & map_a2,
		const Gaussian2d & odo_a1,
		const Gaussian2d & odo_a2,
		const RobotPose *fp, const double *w, int np,
		const Array2d       & path,
		const MapMatcher &matcher){

  fprintf(f,"imap1 = %d;\n",i1+1);
  fprintf(f,"imap2 = %d;\n",i2+1);
  print_rcov(f,"r1in2",r1in2);

  print_obs(f,"map_a1", map_a1);
  print_obs(f,"map_a2", map_a2);

  print_obs(f,"odo_a1", odo_a1);
  print_obs(f,"odo_a2", odo_a2);

  matcher.getCommonMap1().matlabDump(f,"map1");
  matcher.getCommonMap2().matlabDump(f,"map2");

  int imax = find_max(w,np);
  fprintf(f,"best_path = [");
  int i;
  for(i = 0; i < path.numCols(); ++i)
    fprintf(f," %2d", path.get(imax,i));
  fprintf(f,"];\n");


  fprintf(f,"fp = [\n");
  dumpFinalPose(f,fp,w,np);
  fprintf(f,"];\n");

}


RobotPose match_maps(MapMatcher &matcher,
                     const LocalMap &map1, const LocalMap &map2){
  int i;
  SimpleMap m1 = map1.getMap();
  SimpleMap m2 = map2.getMap();

  matcher.match(m1, m2);

  printf("Matched %d nodes\n", matcher.numMatch());

  printf(" M1 => M2\n");

  for( i = 0; i < m1.numMap(); ++i){
    if(matcher.mapping(i) < 0){
      printf("  %03d => none\n", i + 1);
    }else{
      printf("  %03d => %03d", i+1, matcher.mapping(i)+1);
      printf("\t%+.3e %+.3e => %+.3e %+.3e\n"
	     , m1[i].x, m1[i].y
	     , m2[matcher.mapping(i)].x, m2[matcher.mapping(i)].y);
    }
  }


  SimpleMap m1_ = matcher.getCommonMap1();
  SimpleMap m2_ = matcher.getCommonMap2();


  //  printf("Maps : %d %d\n", m1_.numMap(), m2_.numMap());
  RobotPose xyr = matcher.getTranslation();

  m2_.translateMe(xyr);

  printf("====common=elements==\n");
  for(i = 0; i < m1_.numMap(); ++i){
    printf("  %+.3e %+.3e ==> %+.3e %+.3e (dist %6.3f mm, vol %.7e)\n"
	   ,m1_[i].x, m1_[i].y
	   ,m2_[i].x, m2_[i].y
	   ,m1_[i].distance(m2_[i])*1000
	   ,volumeOfTheProduct(m1_[i],m2_[i])
	   );
  }

  printf("vol_sum = %7.2f, %7.2f\n", m1_.probMatch(m2_)
         , m1_.probMatch(m2_)/m1_.numMap());


  return xyr;

}

void evaluatePath(const MapPath &path, const Array2d &pth
		  ,const double *log_w
		  ,FILE* f){
  const double P_LOOP = 0.8;
  int i,j;
  MapTransition *tr = new MapTransition[path.getNHops()];

  for(i = 0; i < path.getNHops(); ++i){
    tr[i] = *path[i];
    tr[i].setWeight(0.0);
  }

  double log_w_max = log_w[0];
  for(i = 1; i < pth.numRows(); ++i){
    if(log_w_max <= log_w[i]) log_w_max = log_w[i];
  }

  printf("Max log: %.9e\n",log_w_max);

  for(i = 0; i < pth.numRows(); ++i){
    register double w = exp(log_w[i] - log_w_max);
    const int *row = pth.getRow_c(i);

    for(j = 0; j < path.getNHops(); ++j){
      //printf("Adding (%d,%d) => %d\n", i,j,pth.get(i,j));
      tr[j].addWeight(row[j],w);
    }
  }

  double scale[path.getNHops()][2];

  fprintf(f,"entropy = [\n");
  for(i = 0; i < path.getNHops(); ++i){
    tr[i].normalise();
    scale[i][0] = 1 - path[i]->computeEntropy();
    scale[i][1] = 1 - tr[i].computeEntropy();

    fprintf(f,"%.9e, %.9e\n"
	    ,1 - scale[i][0]
	    ,1 - scale[i][1]);
  }
  fprintf(f,"];");


  for(i = 0; i < path.getNHops(); ++i){
    for(int j=0; j < tr[i].numP(); ++j){
      register double w = tr[i].getWeight(j)*P_LOOP*scale[i][1] + 
     	                  path[i]->getWeight(j)*(1 - P_LOOP);

      tr[i].setWeight(j,w);
    }

    tr[i].normalise();
    tr[i].computeFinalPose();

    char var[256];
    sprintf(var,"tr(%d).", i+1);
    tr[i].store(f,var);
  }


  fprintf(f,"links = [\n");
  for(i = path.getNHops()-1; i >=0; --i){
    RobotPoseCov rcov = tr[i].getR2In1();

    fprintf(f,"%.5e,%.5e,%.5e"
	    ",%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e\n"
	    ,rcov.x, rcov.y, rcov.rot
	    ,rcov.cov[0][0] 
	    ,rcov.cov[0][1] 
	    ,rcov.cov[0][2] 
	    ,rcov.cov[1][0] 
	    ,rcov.cov[1][1] 
	    ,rcov.cov[1][2] 
	    ,rcov.cov[2][0] 
	    ,rcov.cov[2][1] 
	    ,rcov.cov[2][2] );   
  }
  fprintf(f,"];\n");

}


const double MAP_OVERLAP_THRESHOLD = 0.01;
const double ODO_OVERLAP_THRESHOLD = 0.01;
const double POSE_MD2_THRESHOLD    = POW2(5.0);

void loopClose(GlobalMap &mg, int i1, int i2, int np
	       ,bool force = false){

  MapMatcher matcher;
  Gaussian2d map_area1 = mg[i1].getMapArea();
  Gaussian2d odo_area1 = mg[i1].getPathArea();

  RobotPoseCov r2in1 = mg.getMapRef(i2);
  Gaussian2d map_area2 = mg[i2].getMapArea();
  Gaussian2d odo_area2 = mg[i2].getPathArea();
  r2in1.propagate(&map_area2);
  r2in1.propagate(&odo_area2);

  double map_overlap = volumeOfTheProduct(map_area1, map_area2);
  double odo_overlap = volumeOfTheProduct(odo_area1, odo_area2);

  printf("%% Matching %d => %d\n", i1+1, i2+1);
  printf("map_v = %.9e %s\n"
         ,map_overlap
	 ,map_overlap < MAP_OVERLAP_THRESHOLD?"FAILED":"PASSED"
	 );
  printf("odo_v = %.9e %s\n"
	 ,odo_overlap
	 ,odo_overlap < ODO_OVERLAP_THRESHOLD?"FAILED":"PASSED"
	 );

  if(map_overlap < MAP_OVERLAP_THRESHOLD &&
     odo_overlap < ODO_OVERLAP_THRESHOLD){
    printf(" odo || map ==> FAILED\n");
    if(!force) return;
  }else{
    printf(" odo || map ==> PASSED\n");
  }

  printf("%% Map matching %d=>%d\n",i1+1,i2+1);
  RobotPose m2in1 = match_maps(matcher, mg[i1],mg[i2]);

  printf("m2in1 = [%.7e, %.7e %.7e];\n"
	 ,m2in1.x, m2in1.y, m2in1.rot);

  print_rcov("r2in1",r2in1);

  double md2 = r2in1.mahalanobis2(m2in1);
  printf("MD2 = %.5e (%7.3f)  ", md2, sqrt(md2));

  if(md2 >= POSE_MD2_THRESHOLD){
    printf("FAILED\n");
    if(!force) return;
  }else{
    printf("PASSED\n");
  }

  /*  1. Generate particles that start at 0,0,0 in map frame i1
     *     and end in the map frame i2.
     *
     *  2. For every particle 
     *      a. Translate map1 in to reference frame of i2.
     *      b. Compute overlap quality (particle weight).
     *
     *  3. Dump results to a matlab file, for future display.
     */
  int i;

  printf("Generating Paths....");
  MapPath path = mg.getPath(i1,i2);

  printf("%d",i1+1);
  int map_ind = i1;
  for(i = 0; i < path.getNHops(); ++i){
    map_ind = path[i]->transit(map_ind);
    printf("->%d", map_ind+1);
  }
  printf("\n");

  Array2d pth(np, path.getNHops());
  RobotPose fp[np];
  double w[np];

  printf("...Sampling\n");
  path.samplePath(pth);
  printf("......computing final pose\n");
  path.path2pose(fp,pth);
  printf(".........done.\n");

  printf("Evaluating particles\n");
  SimpleMap m1_0 = matcher.getCommonMap1();
  SimpleMap m2   = matcher.getCommonMap2();


  for(i = 0; i < np; ++i){
    SimpleMap m1(m1_0);
    m1.translateMe(fp[i]);
    w[i] = m1.probMatch(m2);
  }
      
  char buf[256];
  sprintf(buf,"fp%03d_%03d.m",i1+1,i2+1);
  FILE * f = fopen(buf,"w");

  if(f == NULL){
    printf("Failed to open %s\nDumping to stdout.\n", buf);
    f = stdout;
  }

  RobotPoseCov r1in2;
  path.propagate(&r1in2);
  map_area2 = mg[i2].getMapArea();
  odo_area2 = mg[i2].getPathArea();
  r1in2.propagate(&map_area1);
  r1in2.propagate(&odo_area1);


  printf("....Saving to %s\n",buf);
  matlabDump(f,i1,i2,r1in2, map_area1, map_area2
	     , odo_area1, odo_area2
	     , fp, w, np
	     , pth
	     , matcher);

  printf("Evaluating path.\n");
  evaluatePath(path,pth,w,f);

  //Adding new transition
  int NUM_T = path[0]->numP();
  int ind[NUM_T];
  double weight[NUM_T];
  RobotPose trans[NUM_T];

  fprintf(f,"smple = [\n");
  sample_log(ind, w, np, NUM_T);
  for(i = 0; i < NUM_T; ++i){
    weight[i] = w[ind[i]];
    trans[i]  = fp[ind[i]];

    fprintf(f,"%d\n",ind[i]);
  }
  fprintf(f,"];\n");

  mg.addTransition(new MapTransition(i2,i1,trans,weight,NUM_T));

  fclose(f);

  printf("------LOOP_CLOSED----%d->%d--\n",i1+1,i2+1);

}

void loopClose(GlobalMap &mg, int mi, int np){
  int i;
  mg.setReferenceFrame(mi);

  for(i = 0; i < mi; ++i){
    if(!mg.isAdjacent(i,mi)){
      loopClose(mg, mi, i, np);
    }else{
      printf("MAPS %d => %d are adjacent\n", mi, i);
    }
  }
}

void tester(){
  MappingAgent::MappingDecision dd[5];
  MappingAgent::MappingDecision d;
  int nd = 0;
  int i;

  d.weight = 1;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 3;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 2;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 5;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 0.01;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 0.001;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }

  d.weight = 4.0;
  MappingAgent::addDecision(d,dd,nd,5);
  for(i = 0; i < nd; ++i){
    printf("w[%d] = %f\n",i,dd[i].weight);
  }


}

void tester2(GlobalMap &mg){
  int i;
#if 1
  for(i = 0; i < mg.numMaps() - 1; ++i){
    char fname[256];
    sprintf(fname,"tr%02d_%02d.m",i+1,i+2);
    const MapTransition * t = mg.getTransition(i,i+1);

    if(t != NULL){ 
      printf("Storing transition to %s\n",fname);
      t->store(fname);
    }else{
      printf("Failed to get transition %d => %d",i+1,i+2);
    }
  }
#else
  for(i = 0; i < mg.numMaps() - 1; ++i){
    char fname[256];
    sprintf(fname,"tr%02d_%02d.m",i+1,i+2);
    MapTransition t;

    printf("Loading transition from %s.\n",fname);

    if(t.load(fname)){
      sprintf(fname,"_tr%02d_%02d.m",i+1,i+2); 
      printf("Storing it to %s\n",fname);
      t.store(fname);
    }else{
      printf("Failed to load.\n");
    }
  }

#endif
}

int main(int argc, char ** argv){
  int i;
  int i1 = -1;
  int i2 = -2;
  int np = 1000;


  if(argc == 1){
    tester();
    return 1;
  }

  if( argc < 2){
    fprintf(stderr,
	    "Usage:\n"
	    " %s map [np]\n"
	    " OR \n"
	    " %s map i1 i2 np\n", argv[0], argv[0]);
  }

  if( argc == 3){
    np = atoi(argv[2]);
  }
  
  if( argc >= 4){
    i1 = atoi(argv[2]) - 1;
    i2 = atoi(argv[3]) - 1;
    if( argc >= 5) np = atoi(argv[4]);
  }

  srand(time(NULL));
  GlobalMap mg;

  if(!mg.load(argv[1])){
    fprintf(stderr,"Failed to load map: %s\n", argv[1]);
  }

#if 0
  GlobalMap *mg0  = new GlobalMap(mg);
  mg = *mg0;
  delete mg0;
#endif

  if(i1 >= 0){
    mg.setReferenceFrame(i1);
    loopClose(mg,i1,i2,np,true);
  }else{
    for( i = 0; i < mg.numMaps(); ++i){
      loopClose(mg,i,np);
    }
  }

  mg.setReferenceFrame(0);
  printf("map_ref = [\n");
  for(i = 0; i < mg.numMaps(); ++i){
    const RobotPoseCov &rcov = mg.getMapRef(i);
    printf("%.5e,%.5e,%.5e"
         ",%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e,%.5e\n"
	 ,rcov.x, rcov.y, rcov.rot
	 ,rcov.cov[0][0] 
	 ,rcov.cov[0][1] 
	 ,rcov.cov[0][2] 
	 ,rcov.cov[1][0] 
	 ,rcov.cov[1][1] 
	 ,rcov.cov[1][2] 
	 ,rcov.cov[2][0] 
	 ,rcov.cov[2][1] 
	 ,rcov.cov[2][2] );
  }
  printf("];\n");

  if(mg.store("tst")){
    printf("stored modified map to tst\n");
  }else{
    printf("Failed to store modified map.\n");
  }

  return 0;
}












