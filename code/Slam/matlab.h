#ifndef __MATLAB_H__
#define __MATLAB_H__

#ifndef DEPEND
#include <stdio.h>
#endif

//Matlab misc. stuff

class MatlabVariable{
 public:
  enum readReturn{ noError = 0, syntaxError = -1, fileError = -2};

  MatlabVariable();
  MatlabVariable(const char* name, int nrows, int ncols);

  ~MatlabVariable(){ destroy(); }
 
  void setNumRows(int rows);

  double get(int row, int col)const{return value[row][col];} 
  double & operator()(int row, int col){ return value[row][col];}

  bool isEmpty()const{ return value == NULL; }

  const char* getName()const{ return name; }

  int numRows()const{ return nrows;}
  int numCols()const{ return ncols;}

  readReturn read(FILE *f);
  readReturn read_data(FILE *file);

  double *getRow(int row){ return value[row];}

 private:
  char* name;
  double **value;

  int nrows;
  int ncols;


  void destroy();
};



#endif
