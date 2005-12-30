#include "matlab.h"
#ifndef DEPEND
#include <string.h>
#endif

inline int & min(int &a, int &b){if(a < b) return a; else return b;}
inline int & max(int &a, int &b){if(a > b) return a; else return b;}

//Matlab misc stuff
///////////////////////////////////////////////////////////////////////////

void skip_line(FILE *f){
  char c;

  while(fscanf(f,"%c",&c) == 1){
    if(c == '\n') return;
  }
}

int skip_spaces(FILE *f){
  int c;
  for( c = fgetc(f); c == ' ' || c == '\t'; c = fgetc(f)){
    ;
  }

  ungetc(c,f);

  return c;
}

void skip_emptyStatements(FILE*f){
  int c = 1;

  while(c >= 0){
    c = fgetc(f);

    switch(c){
    case '%':
      skip_line(f);
      break;

    case ' ': case '\t': case ';': case '\n':
      break;

    case -1:
      return;

    default:
      ungetc(c,f);
      return;
    }
  }


}


MatlabVariable::MatlabVariable(){
  name = NULL;
  value = NULL;
  nrows = 0;
  ncols = 0;
}

MatlabVariable::MatlabVariable(const char* Name, int Nrows, int Ncols){
  if(Name == NULL){
    name = NULL;
  }else{
    name = new char[strlen(Name)+1];
    strcpy(name, Name);
  }
  value = NULL;
  ncols = Ncols;
  nrows = 0;

  setNumRows(Nrows);
}


MatlabVariable::readReturn MatlabVariable::read_data(FILE *f){

  double d;
  int i;
  char c;

  int nc_increment = 100;
  int nr_increment = 100;

  int &nc = ncols;
  nc = nc_increment;

  setNumRows(1);
  double *dd = new double[nc];
  value[0] = dd;

  int col = 0;
  int row = 0;

  do{
    if( skip_spaces(f) == '\n' ){

      if(col >0){
	//Start new row
	if(row == 0) nc = col;

	if(col != nc){
	  return syntaxError;
	}

	row += 1;
	col = 0;

	if(row >= nrows){
	  setNumRows(nrows + nr_increment);
	}
        dd = value[row];
      }

      skip_line(f);
    };

    i = fscanf(f,"%lf%c",&d,&c);
    if(i < 0){ //Must be end of file
      return syntaxError;

    }else if(i == 0){
      skip_spaces(f); //Skip spaces, but not EOL

      if( fscanf(f,"%c",&c) != 1){ //Read character
	return syntaxError;

      }else{

	switch(c){
	case '%':
	  skip_line(f);

	case ';': case '\n':
	  if(col >0){
	    //Start new row
	    if(row == 0) nc = col;

	    if(col != nc){
	      return syntaxError;
	    }

	    row += 1;
	    col = 0;

	    if(row >= nrows){
	      setNumRows(nrows + nr_increment);
	    }
	    dd = value[row];
	  }

	  break;

	case '.':
	  skip_line(f);
	  break;

	case ']':
	  if(col == 0){
            setNumRows(row);
	  }else{
	    setNumRows(row +1);
	    if(nrows == 1) nc = col;
	  }
	
	  return noError;

	default:
	  return syntaxError;
	}
      }	
    }else{
      if(row == 0){
	if(col >= nc){
	  double *dd_ = new double[nc + nc_increment];
	  memcpy(dd_, dd , sizeof(double)*nc);
	  delete[] dd;
	  dd = dd_;
	  nc += nc_increment;
	  value[0] = dd;
	}
       }else{
	if(col >= nc){
	  return syntaxError;
	}
      }

      dd[col] = d;
      col += 1;

      switch(c){
      case '\t':
      case ' ':
      case ',': //Read new value

	break;

      case '%': //Start new line
	skip_line(f);
	//fall through
      case '\n': case '\r': case ';':

	if(col >0){
	  //Start new row
	  if(row == 0) nc = col;

	  if(col != nc){
	    return syntaxError;
	  }

	  row += 1;
	  col = 0;

	  if(row >= nrows){
	    setNumRows(nrows + nr_increment);
	  }
	  dd = value[row];
	}

	break;

      case ']':
	if(col == 0){
            setNumRows(row);
	}else{
	  setNumRows(row +1);
	  if(nrows == 1) nc = col;
	}
	
	return noError;

      default:
	return syntaxError;
      }

    }
  }while(1);


  return noError;
}


MatlabVariable::readReturn MatlabVariable::read(FILE *f){
  char buf[1024];

  destroy();

  skip_emptyStatements(f);
 //Read the name;
  if(fscanf(f,"%[A-Za-z_0-9.] = [", buf) != 1){
    return syntaxError;
  }

  name = new char[strlen(buf)+1];
  strcpy(name,buf);

  readReturn ret = read_data(f);

  if(ret != noError){
    destroy();
  }

  return ret;
}

void MatlabVariable::setNumRows(int Rows){
  double **v;
  int i;

  if(Rows >0 ) v = new double*[Rows];
  else v = NULL;

  for(i = 0; i < min(nrows,Rows) ; ++i){
    v[i] = value[i];
  }

  for(i = nrows; i < Rows; ++i){ //If we growing add more rows.
    v[i] = new double[ncols];
  }

  if(value != NULL){
    for(i = Rows; i < nrows; ++i){ //If we shrinking release prev space.
      delete[] value[i];
    }
    delete[] value;
  }

  nrows = Rows;
  value = v;
}

void MatlabVariable::destroy(){
  setNumRows(0);

  if(name != NULL){ 
    delete[] name;
    name = NULL;
  }
}
