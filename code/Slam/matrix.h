#ifndef __MATRIX_H__
#define __MATRIX_H__

inline int index(int r, int c, int nc){
  int i = r*nc + c;
  return i;
}

inline void matrix_print(const void *A, int n, int m
                        , const char* format = " %5g"){

  const double *matrix = (const double*) A;
  int r,c;

  for(r = 0; r < n; ++r){
    for(c = 0; c < m; ++c){
      printf(format,matrix[index(r,c,m)]);
    }
    printf("\n");
  }
}

inline void matrix_add(void* AB, const void *A, const void *B, 
                       int n, int m){
  const double *m1 = (const double*) A;
  const double *m2 = (const double*) B;
  double *res = (double*) AB;

  int N = n*m;  int i;
  for(i = 0; i < N; ++i) res[i] = m1[i] + m2[i];
}

inline void matrix_subtract(void* AB, const void *A, const void *B, 
                            int n, int m){
  const double *m1 = (const double*) A;
  const double *m2 = (const double*) B;
  double *res = (double*) AB;

  int N = n*m;  int i;
  for(i = 0; i < N; ++i) res[i] = m1[i] - m2[i];
}

inline void matrix_transpose(void *At, const void *A, int n, int m){
  double *tmp = NULL;
  double *res = (double *) At;

  if(At == A){
    tmp = new double[n*m];
    res = tmp;
  }else{
    res = (double *)At;
  }

  const double *a = (const double *) A;
  int c,r;

  for(r = 0; r < n; ++r){
    for(c = 0; c < m; ++c){
      res[index(c,r,n)] = a[index(r,c,m)];
    }
  }

  if(tmp != NULL){
    memcpy(At, tmp, n*m*sizeof(double));
    delete tmp;
  }
}

inline void matrix_multiply(void* AB, const void *A, const void *B, 
                           int n, int m, int r){
#define matrix1(row,col)  m1[index(row,col, m)]
#define matrix2(row,col)  m2[index(row,col, r)]
#define matrix0(row,col) res[index(row,col, r)]

  double *tmp      = NULL;
  const double *m1 = (const double*) A;
  const double *m2 = (const double*) B;
  double *res;

  if(AB == A || AB == B){
    tmp = new double[n*r];
    res = tmp;
  }else{
    //Result matrix is different from sources
    res = (double*) AB;
  }


  int i,j,k;
    
#if 0
  printf("A = \n");
  matrix_print(A,n,m);
  printf("B = \n");
  matrix_print(B,m,r);
#endif
  
  for(i = 0; i < n; ++i){
    for(k = 0; k < r; ++k){
      double sum = 0.0;
      for(j = 0; j < m; ++j){
	sum += matrix1(i,j)*matrix2(j,k);
      }
      matrix0(i,k) = sum;
    }
  }

  if(tmp != NULL){
    memcpy(AB,tmp, n*r*sizeof(double));
    delete[] tmp;
  }

#if 0
  printf("AB =\n");
  matrix_print(AB,n,r);
#endif

#undef matrix1
#undef matrix2
#undef matrix0

}


#endif
