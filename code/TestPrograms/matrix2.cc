#ifndef DEPEND
#include <stdlib.h>
#include <stdio.h>
#endif

#include "matrix2.h"


int matrix_read(FILE* f, double a[2][2]){
  return fscanf(f,"%lf %lf;%lf %lf"
		,&a[0][0]
		,&a[0][1]
		,&a[1][0]
		,&a[1][1]) == 4;
}

int matrix_print(FILE *f, const double a[2][2]){
  return fprintf(f,"%8.5f %8.5f\n%8.5f %8.5f\n"
		 ,a[0][0]
		 ,a[0][1]
		 ,a[1][0]
		 ,a[1][1]);
            
}

#if 0
void tst(const double a[2][2]){
  printf("%u\n",&a[0][0]);
  printf("%u\n",&a[0][1]);
  printf("%u\n",&a[1][0]);
  printf("%u\n",&a[1][1]);
}
typedef int arr3[3];

#endif


int main(void){

  double a[2][2];
  double b[2][2];
  double res[2][2];
  char op_buf[10];
  char op;

#if 0
  arr3 *zz;

  zz = new arr3[10];

  zz = (int(*)[3]) malloc(10*3*sizeof(int));

  zz[0][1] = 1;
  zz[9][0] = 1001;

  printf("%d\n",zz[9][0]);

  tst(a);
  tst(b);
  tst(res);
#endif

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
	matrix2_add(res,a,b);
	break;
      case '-':
	matrix2_subtract(res,a,b);
	break;
      case '*':
	matrix2_multiply(res,a,b);
	break;
      case '/':
	matrix2_divide(res,a,b);
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
      matrix2_inverse(res,a);
      printf("A = \n");
      matrix_print(stdout,a);
      printf("\nC = inverse(A)\n");
      matrix_print(stdout,res);
      printf("\n");
      break;

    case 't':
    case 'T':
      matrix2_transpose(res,a);

      printf("A = \n");
      matrix_print(stdout,a);
      printf("\nC = transpose(A)\n");
      matrix_print(stdout,res);
      printf("\n");
      break;
       
    case '2':
      matrix2_multiply(a,a,a);
      printf("A^2 = \n");
      matrix_print(stdout,a);

      break;
    default:
      printf("Wrong operand: %c",op);

    }

  }

  return 0;
}

