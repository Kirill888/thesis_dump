#ifndef DEPEND
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#endif

#include "matrix3.h"
#include "util.h"


int matrix_read(FILE* f, double a[3][3]){
  return fscanf(f,"%lf %lf %lf %lf %lf %lf %lf %lf %lf"
		,&a[0][0]
		,&a[0][1]
		,&a[0][2]
		,&a[1][0]
		,&a[1][1]
		,&a[1][2]
		,&a[2][0]
		,&a[2][1]
		,&a[2][2]) == 9;
}

int matrix_print(FILE *f, const double a[3][3]){
  return fprintf(f,"%8.5f %8.5f %8.5f\n"
                   "%8.5f %8.5f %8.5f\n"
                   "%8.5f %8.5f %8.5f\n"
		,a[0][0]
		,a[0][1]
		,a[0][2]
		,a[1][0]
		,a[1][1]
		,a[1][2]
		,a[2][0]
		,a[2][1]
		,a[2][2]);
            
}

int main(void){

  double a[3][3];
  double b[3][3];
  double res[3][3];
  char op_buf[10];
  char op;

  while(1){
    
    printf("Matrix1:");
    if(!matrix_read(stdin,a)) break;


    //scanf("%*[ \n]");

    printf("Operand:");
    if(scanf("%s",op_buf)!=1) break;
    op = op_buf[0];

    switch(op){
    case '+':
    case '-':
    case '*':
    case '/':
      printf("Matrix2:");
      if(!matrix_read(stdin,b)) return 1;
      switch(op){
      case '+':
	matrix3_add(res,a,b);
	break;
      case '-':
	matrix3_subtract(res,a,b);
	break;
      case '*':
	matrix3_multiply(res,a,b);
	break;
      case '/':
	matrix3_divide(res,a,b);
	break;
      }
      printf("A = \n");
      matrix_print(stdout,a);
      printf("\nB = \n");
      matrix_print(stdout,b);
      printf("\nC = A %c B \n",op);
      matrix_print(stdout,res);
      printf("\n");
      break;
    case 'i':
    case 'I':
      matrix3_inverse(res,a);
      printf("A = \n");
      matrix_print(stdout,a);
      printf("\nC = inverse(A)\n");
      matrix_print(stdout,res);
      printf("\n");
      break;

    case 't':
    case 'T':
      matrix3_transpose(res,a);

      printf("A = \n");
      matrix_print(stdout,a);
      printf("\nC = transpose(A)\n");
      matrix_print(stdout,res);
      printf("\n");
      break;
       
    case '2':
      matrix3_multiply(a,a,a);
      printf("A^2 = \n");
      matrix_print(stdout,a);

      break;

    case 'c':
      if(choldc(res,a)){
	printf("A = L*L'\n");
	matrix_print(stdout,res);
	printf("\n");
      }else{
	printf("Matrix is not positive definite\n");
      }
      break;
    default:
      printf("Wrong operand: %c",op);

    }

  }

  return 0;
}

