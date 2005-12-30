#ifndef DEPEND
#include <stdlib.h>
#endif

#include "graph.h"
#include "matlab.h"
#include "matrix.h"
#include "simplex.h"

#define REPORT(format, args...) printf(format, ##args)

void robot2sensor(RobotPose*,const RobotPose*){
}

Graph* loadGraph(const char *fname){
  int i;
  FILE * f = fopen(fname,"r");
  if(f == NULL) return NULL;

  MatlabVariable m;
  if(m.read(f) != MatlabVariable::noError){
    fclose(f);
    return NULL;
  }

  fclose(f);

  if(m.numCols() < 6) return NULL;

  Gaussian2d data[m.numRows()];

  for(i = 0; i< m.numRows(); ++i){
    data[i].x  = m(i, 0);
    data[i].y  = m(i, 1);

    data[i].cov[0][0] = m(i,2);    data[i].cov[0][1] = m(i,3);
    data[i].cov[1][0] = m(i,4);    data[i].cov[1][1] = m(i,5);
  }

  Graph *g = new Graph(data, m.numRows());
//   Graph *g = new Graph();
//   g->set(data, m.numRows());

  return g;
}

void computeMeanAndAngles(double *a, double *cov,
			  const Gaussian2d*, int n);

void testMeanAndAngle(const Graph *g){
  int n = g->numNodes();
  double cov[n+2][n+2];
  double a[n+2];

  computeMeanAndAngles((double*)a,(double*)cov,g->getNodes(),n);

  printf("A = [\n");
  matrix_print(a,1,n+2," %.9e");
  printf("];\n");

  printf("Cov  = [\n");
  matrix_print(cov,n+2,n+2," %.9e");
  printf("];\n");

}

int main(int argc, char* argv[]){

  if(argc != 3){
    fprintf(stderr,"Usage: %s map1 map2\n", argv[0]);
    return -1;
  }
  Graph *g1 = loadGraph(argv[1]);

  if(g1 == NULL){
    fprintf(stderr,"Failed to load: %s\n", argv[1]);
    return -1;
  }

  Graph *g2 = loadGraph(argv[2]);
  if(g2 == NULL){
    fprintf(stderr,"Failed to load: %s\n", argv[2]);
    return -1;
  }

  //  testMeanAndAngle(g1);
  //  testMeanAndAngle(g2);
  //  return 0;

  //  g1->matlab_dump(stdout, "g1");
  //  g2->matlab_dump(stdout, "g2"); fflush(stdout);

  GraphMatcher matcher;

  matcher.match(*g1,*g2);
  int i1[g1->numNodes()];
  int i2[g2->numNodes()];
  int n_matched;

  n_matched = matcher.commonElements1(i1);
  matcher.commonElements2(i2);

  int i;

  printf("ii1 = [");
  for(i = 0; i < n_matched; ++i){
    printf(" %d", i1[i]+1);
  }
  printf("];\n");

  printf("ii2 = [");
  for(i = 0; i < n_matched; ++i){
    printf(" %d", i2[i]+1);
  }
  printf("];\n");

  RobotPoseCov xyr;
  allign_graphs(&xyr, *g1, i1, *g2, i2, n_matched,true);
  print_rcov("xyr",xyr);

//   RobotPose x0;
//   numericalAlign(&x0, g1, i1, g2, i2, n_matched, xyr);

//   xyr.x   = x0.x;
//   xyr.y   = x0.y;
//   xyr.rot = x0.rot;

//   print_rcov("xyr_opt", xyr);


#if 0
  RobotPoseCov midPoint;

  printf("Compute Mid Points: graph1\n odo = [ ...\n");

  for(i = 1; i < g1->numNodes(); ++i){
    computeMiddlePoint(&midPoint, g1->node(0), g1->node(i));
    print_rcov(NULL,midPoint);
  }
  printf("];\n");
#endif

  return 0;
}





#if 0
#include "mapMatch.h"
#include "globalmap.h"
#include "localmap.h"
#include "TimeTools.h"
#include "random.h"
#include "util.h"

#include "landmark2d.h"
#include "tree.h"
#include "matrix.h"
#include "matrix3.h"
#include "polygon.h"

struct fparams{
  //Input
  const SimpleMap *m1;
  const SimpleMap *m2;

  //Output
  int out_size;
  RobotPose *poses;
  double    *weight;
  int currentIndex;

  //Constructor
  fparams(const SimpleMap* M1, const SimpleMap *M2,
          RobotPose* Poses, double *W, int sz):m1(M1),m2(M2),
					     out_size(sz),
					     poses(Poses),
					     weight(W),
					     currentIndex(0){;}
};



void sampleTransition(const SimpleMap *m1, const SimpleMap *m2,
                      const RobotPoseCov *m2in1,
		      RobotPose *samples, double *weight, int nsamples);

void estimateTransition(RobotPoseCov *m2in1,
                        const SimpleMap *m1, const SimpleMap *m2,
                        const RobotPose* x0);

int separateMap(const SimpleMap* m1, int *core1,
		const MapAreaInterface* region);
int separateMap(const SimpleMap* m1, int *core1,
		const Polygon* poly);

int main(int argc, char* argv[]){

#if 0
  if(argc == 2){
    GlobalMap gm;
    printf("Loading global map: %s\n",argv[1]);

    if(!gm.load(argv[1])){
      fprintf(stderr,"Failed to load global map: %s\n",argv[1]);
      return 1;
    }
    printf("Load succesful, storing\n");
    gm.setReferenceFrame(0);
    gm.store("tmp");
    return 0;
  }
#endif
  char *type = "shiny";

  if( argc < 3){
    fprintf(stderr,"Usage: %s map1 map2 [shiny or tree]\n", argv[0]);
    return 1;
  }
  if(argc > 3) type = argv[3];

  if(strcmp(type,"shiny") == 0){
    LMAP_LoadMap = Landmark2d_LoadMap;
    LMAP_MapAreaFactory = new Landmark2dMapAreaFactory();
  }else if(strcmp(type,"tree") == 0){
    LMAP_LoadMap = Tree_LoadMap;
    LMAP_MapAreaFactory = new TreeMapAreaFactory();
  }else{
    printf("Unknown type: %s\n",type);
    return 1;
  }

  LocalMap map1;
  LocalMap map2;


  printf("%%Loading: %s\n", argv[1]);
  if(!map1.load(argv[1])){
    fprintf(stderr,"%%Failed to load map: %s\n", argv[1]);
    return 2;
  }


  printf("%%Loading: %s\n", argv[2]);
  if(!map2.load(argv[2])){
    fprintf(stderr,"Failed to load map: %s\n", argv[2]);
    return 2;
  }

  SimpleMap m1;
  SimpleMap m2;

  m1 = map1.getMap();
  m2 = map2.getMap();

  m1.matlabDump(stdout,"m1");
  m2.matlabDump(stdout,"m2");

  MapMatcher matcher;
  matcher.match(m1,m2);

  if(matcher.numMatch() < 3){
    printf("%%No match.\n");
  }else{
    int i;
    int ind[matcher.numMatch()];
    int n;

    n = matcher.commonElements1(ind);
    SimpleMap* m1_ = m1.subMap(ind,n);
    n = matcher.commonElements2(ind);
    SimpleMap* m2_ = m2.subMap(ind,n);

    RobotPose xyr = matcher.getTranslation();

    printf("%%...Matched %d nodes\n", matcher.numMatch());
    printf("%%...M1 => M2\n");
    for( i = 0; i < m1.numMap(); ++i){
      if(matcher.mapping(i) < 0){
	printf("%%...%03d => none\n", i + 1);
      }else{
	printf("%%...%03d => %03d\n", i+1, matcher.mapping(i)+1);
      }
    }

    int np = 1000;
    RobotPose samples[np];
    double weight[np];

    RobotPoseCov m2in1;

    estimateTransition(&m2in1, m1_, m2_, &xyr);
    print_rcov("m2in1", m2in1);

    sampleTransition(m1_, m2_, &m2in1, samples, weight, np);

    FILE *fsample = fopen("sample.out","w");
    for(int i = 0; i < np; ++i){
      fprintf(fsample,"%.9e %.9e %.9e %.9e\n"
              ,samples[i].x, samples[i].y, samples[i].rot
	      ,weight[i]);
    }
    fclose(fsample);

    m2_->translateMe(xyr);

    printf("vol_sum = [%7.2f, %7.2f]\n", m1_->probMatchLog(*m2_)
	   , m1_->probMatchLog(*m2_)/m1_->numMap());

    printf("xyr = [ %.3f %.3f %.3f];\n", xyr.x, xyr.y, xyr.rot);
    printf("matches = [");
    for( i = 0; i < m1.numMap(); ++i){
      printf("%02d ", matcher.mapping(i)+1);
    }

    printf("];\n");

    printf("i1 = find(matches > 0); i2 = matches(i1);\n");
    printf("m2_ = translate_obs(m2,m2in1);");

#if 1
    int core1[m1.numElements()];
    int core2[m2.numElements()];

    for(int i = 0; i < m1.numElements(); ++i){
      core1[i] = m1.get(i)->isCore();
    }
    for(int i = 0; i < m2.numElements(); ++i){
      core2[i] = 1;
    }

    m2.translateMe(xyr);
    Line l;
    separateMap(&m2, core2,map1.getRegion());

    printf("core1 = [");
    for(int i = 0; i < m1.numElements(); ++i){
      if(core1[i]) printf("%d ", i+1);
    }
    printf("]\n");

    printf("core2 = [");
    for(int i = 0; i < m2.numElements(); ++i){
      if(core2[i]) printf("%d ", i+1);
    }
    printf("]\n");


#endif

    delete m1_;
    delete m2_;
  }

  return 0;
}
#endif




