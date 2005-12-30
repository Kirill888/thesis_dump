#ifndef __ARRAY2D_H__
#define __ARRAY2D_H__

#ifndef DEPEND
#include <string.h>
#endif

typedef int A2D_type;

class Array2d{
   private:
     A2D_type *buf;

     int nrows, ncols;

   public:

     class Row{
        private:
	  A2D_type *row;
	public:
	  Row(A2D_type *a){ row = a;}

          A2D_type& operator[](int col)const{ return row[col]; }
     };

     Array2d(){
       buf = NULL;
       nrows = ncols = 0;
     }

     Array2d(int rows, int cols){
	ncols = cols;
        nrows = rows;
	buf   = new A2D_type[rows*cols];
     }

     Array2d(const Array2d &other){
	nrows = other.nrows;
	ncols = other.ncols;

	buf = new A2D_type[nrows*ncols];
	memcpy(buf, other.buf, nrows*ncols*sizeof(A2D_type));
     }

     const Array2d & operator=(const Array2d &other){
	if(&other == this) return *this;

	if(buf != NULL) delete[] buf;

	nrows = other.nrows;
	ncols = other.ncols;

	buf = new A2D_type[nrows*ncols];
	memcpy(buf, other.buf, nrows*ncols*sizeof(A2D_type));

	return *this;
     }

     ~Array2d(){
	if(buf != NULL) delete[] buf;
     }

     A2D_type & operator()(int row, int col){
        return buf[row*ncols + col];
     }

     A2D_type get(int row, int col)const{
        return buf[row*ncols + col];
     }

     void set(int row, int col, A2D_type val){
        buf[row*ncols + col] = val;
     }

     Row operator[](int row){ return Row(buf + row*ncols); }

     A2D_type * getRow(int row){ return buf + row*ncols; }
     const A2D_type * getRow_c(int row)const{ return buf + row*ncols; }

     int numRows()const{ return nrows;}
     int numCols()const{ return ncols;}

     void zero(){
        if(buf != NULL){
	   bzero(buf, nrows*ncols*sizeof(A2D_type));
	}
     }

     void setSize(int rows, int cols){
       if(buf != NULL) delete[] buf;

       ncols = cols;
       nrows = rows;
       if(rows != 0 && cols != 0){
          buf   = new A2D_type[nrows*ncols];
       }else{
	 buf = NULL;
       }
     }

     void destroy(){ 
       if(buf != NULL) delete[] buf;
       buf = NULL;
       ncols = nrows = 0;
     }


};

#endif



