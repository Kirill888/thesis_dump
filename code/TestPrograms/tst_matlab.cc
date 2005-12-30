#include "matlab.h"
#include "globalmap.h"

int main(int argc, char **argv){

  GlobalMap m;

  if(argc != 3){
    fprintf(stderr,"Usage: %s from_prefix to_prefix\n", argv[0]);
    return 0;
  }

  if(m.load(argv[1])){
    printf("Loaded global map %s\n", argv[1]);
    printf("Storing it to %s\n", argv[2]);

    if(m.store(argv[2])){
      printf("ok.\n");
    }else{
      fprintf(stderr,"Failed.\n");
    }

  }else{
    fprintf(stderr,"Failed to load global map %s\n", argv[1]);
  }

#if 0
  MatlabVariable m;


  while( m.read(stdin) == MatlabVariable::noError){
    printf("No error, matrix is \"%s\" %dx%d\n",m.getName(), 
	   m.numRows(), m.numCols());

    int r,c;

    for(r = 0; r < m.numRows(); ++r){
      for(c = 0; c < m.numCols(); ++c){
	printf(" %10.3f",m(r,c));
      }
      printf("\n");
    }
  }

  printf("Syntax Error!\n");
#endif
  

  return 1;
}
