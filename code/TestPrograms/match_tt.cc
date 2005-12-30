#include "mapMatch.h"
#include "globalmap.h"
#include "localmap.h"
#include "TimeTools.h"
#include "random.h"

int int_sequence[500];

void init_int_sequence(){
  int i;

  for(i = 0; i < 500; ++i) int_sequence[i] = i;
}

void shuffle(int* out,int* in, int n, int nshuffles){
  UniformRandomNumber dice(0,n);
  int i;

  //Init to initial
  memcpy(out, in, n*sizeof(int));

  for(i = 0; i < nshuffles; ++i){
    int i1 = dice.nextIntRandom();
    int i2 = dice.nextIntRandom();

    int tmp = out[i1];
    out[i1] = out[i2];
    out[i2] = tmp;
  }

}

void run_match(FILE* f, 
               const SimpleMap &m1, 
               const SimpleMap &m2, 
               bool do_shuffle){

  int ind[500];
  int i;
  SimpleMap m2_;

  if(do_shuffle){
    shuffle(ind, int_sequence, m2.numMap(), 100);
#if 0
    fprintf(f,"shuffle = [");
    for(i = 0; i < m2.numMap(); ++i){
      fprintf(f,"%d ",ind[i]);
    }
    fprintf(f,"];\n");
#endif
    m2_ = m2.subMap(ind,m2.numMap());
  }else{
    m2_ = m2;
  }

  double t0 = GetCurrentTime();
  MapMatcher matcher;
  matcher.match(m1,m2_);
  double dt =  GetCurrentTime() - t0;

  RobotPose xyr = matcher.getTranslation();

  fprintf(f,"%10.6f %+.5e %+.5e %+.5e",dt,xyr.x, xyr.y, xyr.rot);
  for(i = 0; i < m1.numMap(); ++i){
    if( do_shuffle){
      if(matcher.mapping(i) >= 0){
	fprintf(f," %2d",ind[matcher.mapping(i)]+1);
      }else{
	fprintf(f,"  0");
      }

    }else{
      fprintf(f," %2d",matcher.mapping(i)+1);

    }
  }
  fprintf(f,"\n");


}

int main(int argc, char* argv[]){

  init_int_sequence();

  int i;

  char *file_in = NULL;
  char *file_out = NULL;
  int n_runs = 10;
  int nmap1 = -1;
  int nmap2 = -1;

  FILE *f_in = stdin;
  FILE *f_out = stdout;

  if(argc >= 2) n_runs = atoi(argv[1]);

  if(argc >= 3) nmap1 = atoi(argv[2]);
  if(argc >= 4) nmap2 = atoi(argv[3]);

  if(argc >= 5) file_in = argv[4];
  if(argc >= 6) file_out = argv[5];



  if( argc > 6){
    fprintf(stderr,"Usage: %s n_trial nmap1 nmap2 in  out\n", argv[0]);
    return 1;
  }


  SimpleMap m1;
  SimpleMap m2;

  //Load the maps
  if(file_in){
    printf("%%Loading from file: %s\n", file_in);
    f_in = fopen(file_in,"r");
    if(f_in == NULL){
      fprintf(stderr,"Failed to open file: %s\n", file_in);
      return 2;
    }
  }


  if(!m1.read(f_in) || !m2.read(f_in)){
    fprintf(stderr,"Failed to load maps.\n");
    return 2;
  }

  if(f_in != stdin) fclose(f_in);

  printf("%%Loaded maps: %d and %d\n",m1.numMap(), m2.numMap());

  if(nmap1 > 0){
    printf("%%Shrinking map1 to %d\n",nmap1);
    m1 = m1.subMap(int_sequence,nmap1);
  }

  if(nmap2 > 0){
    printf("%%Shrinking map2 to %d\n",nmap2);
    m2 = m2.subMap(int_sequence,nmap2);
  }

  m1.matlabDump(stdout, "m1");
  m2.matlabDump(stdout, "m2");


  if(file_out != NULL){
    f_out = fopen(file_out,"w");
    if(f_out == NULL){
      fprintf(stderr,"Failed to open: %s\n", file_out);
      return 3;
    }
    printf("Dumping to %s\n", file_out);
  }

  //Do matching
  for(i = 0; i < n_runs; ++i){
    run_match(f_out,m1,m2,true);
  }

  return 0;
}





