#ifndef __MATRIX3_H__
#define __MATRIX3_H__

#ifndef DEPEND
#include <string.h> //memcpy
#endif

extern const double ZEROS_3x3[3][3];
extern const double I_3x3[3][3];

#define ZEROS_3x3_INIT {{0,0,0},{0,0,0},{0,0,0}}
#define I_3x3_INIT     {{1,0,0},{0,1,0},{0,0,1}}

void matrix3_add(double res[3][3],
                 const double a[3][3], const double b[3][3]);
void matrix3_subtract(double res[3][3],
                      const double a[3][3], const double b[3][3]);
void matrix3_multiply(double res[3][3],
                      const double a[3][3], const double b[3][3]);
void matrix3_divide(double res[3][3],
                    const double a[3][3], const double b[3][3]);


void matrix3_transpose(double res[3][3],
                       const double a[3][3]);
void matrix3_inverse(double res[3][3],
                     const double a[3][3]);

double matrix3_det(const double a[3][3]);


void matrix3_multiply(double res[3],
                      const double a[3][3], 
		      const double b[3]);

// c = b'*a*b
double matrix3_multiply2(const double a[3][3], const double b[3]);

void vector3_add(double res[3], double a[3], double b[3]);
void vector3_subtract(double res[3], double a[3], double b[3]);


#if 0
void matrix3_add(double res[3][3],
                 const double a[3][3], double b);
void matrix3_subtract(double res[3][3],
                      const double a[3][3], double b);
void matrix3_multiply(double res[3][3],
                      const double a[3][3], double b);
void matrix3_divide(double res[3][3],
		    const double a[3][3], double b);



void matrix3_eigenvalues(double res[3], const double a[3][3]);

void vector3_add(double res[3], double a[3], double b[3]);
void vector3_subtract(double res[3], double a[3], double b[3]);
void vector3_multiply(double res[3], double a[3], double b);
void vector3_divide(double res[3], double a[3], double b);

double mahalanobis3(const double x[3], const double cov[3][3]
		    , const double o[3]);

double mahalanobis3_inv(const double x[3], const double inv[3][3]
			, const double o[3]);

#endif

/////////////////////////////////////////////////////////////////////////
// INLINED VERSION
////////////////////////////////////////////////////////////////////////


inline void matrix3_add(double res[3][3],
			const double a[3][3], const double b[3][3]){
  res[0][0] = a[0][0] + b[0][0];
  res[0][1] = a[0][1] + b[0][1];
  res[0][2] = a[0][2] + b[0][2];
  res[1][0] = a[1][0] + b[1][0];
  res[1][1] = a[1][1] + b[1][1];
  res[1][2] = a[1][2] + b[1][2];
  res[2][0] = a[2][0] + b[2][0];
  res[2][1] = a[2][1] + b[2][1];
  res[2][2] = a[2][2] + b[2][2];
}

inline void matrix3_subtract(double res[3][3],
			     const double a[3][3], const double b[3][3]){
  res[0][0] = a[0][0] - b[0][0];
  res[0][1] = a[0][1] - b[0][1];
  res[0][2] = a[0][2] - b[0][2];
  res[1][0] = a[1][0] - b[1][0];
  res[1][1] = a[1][1] - b[1][1];
  res[1][2] = a[1][2] - b[1][2];
  res[2][0] = a[2][0] - b[2][0];
  res[2][1] = a[2][1] - b[2][1];
  res[2][2] = a[2][2] - b[2][2];
}

inline void matrix3_multiply(double res[3][3],
                             const double a[3][3], 
			     const double b[3][3]){
/*
a00*b00+a01*b10+a02*b20
a10*b00+a11*b10+a12*b20
a20*b00+a21*b10+a22*b20
a00*b01+a01*b11+a02*b21
a10*b01+a11*b11+a12*b21
a20*b01+a21*b11+a22*b21
a00*b02+a01*b12+a02*b22
a10*b02+a11*b12+a12*b22
a20*b02+a21*b12+a22*b22
*/
#ifdef MATRIX3_NON_SAFE_MULTIPLY
  res[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
  res[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
  res[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
  res[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
  res[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
  res[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
  res[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];
  res[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
  res[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];
#else
  double tmp[3][3];

  tmp[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
  tmp[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
  tmp[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
  tmp[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
  tmp[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
  tmp[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
  tmp[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];
  tmp[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
  tmp[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];

  res[0][0] = tmp[0][0];
  res[1][0] = tmp[1][0];
  res[2][0] = tmp[2][0];
  res[0][1] = tmp[0][1];
  res[1][1] = tmp[1][1];
  res[2][1] = tmp[2][1];
  res[0][2] = tmp[0][2];
  res[1][2] = tmp[1][2];
  res[2][2] = tmp[2][2];
#endif
  //  memcpy(res,tmp,3*3*sizeof(double));
}

inline void matrix3_divide(double res[3][3],
			   const double a[3][3], const double b[3][3]){
  double b_inv[3][3];
  matrix3_inverse(b_inv,b);
  matrix3_multiply(res,a,b_inv);
}

inline void matrix3_transpose(double res[3][3],
                              const double a[3][3]){
  res[0][0] = a[0][0];
  res[1][1] = a[1][1];
  res[2][2] = a[2][2];

  double tmp;

  tmp       = a[0][1];
  res[0][1] = a[1][0];
  res[1][0] = tmp;

  tmp = a[0][2];
  res[0][2] = a[2][0];
  res[2][0] = tmp;

  tmp = a[1][2];
  res[1][2] = a[2][1];
  res[2][1] = tmp;
}

inline void matrix3_inverse(double res[3][3],
                     const double a[3][3]){

/*
   scaler = 1/(a00*a11*a22 - a00*a12*a21 - a10*a01*a22
              +a10*a02*a21 + a20*a01*a12 - a20*a02*a11);

-(-a11*a22+a12*a21)*scaler
-(a10*a22-a12*a20)*scaler
 (a10*a21-a11*a20)*scaler
 (-a01*a22+a02*a21)*scaler
-(-a00*a22+a02*a20)*scaler
 (-a00*a21+a01*a20)*scaler
-(-a01*a12+a02*a11)*scaler
 (-a00*a12+a02*a10)*scaler
-(-a00*a11+a01*a10)*scaler
*/

  double scaler = 1.0/(a[0][0]*a[1][1]*a[2][2] - a[0][0]*a[1][2]*a[2][1] 
		     - a[1][0]*a[0][1]*a[2][2] + a[1][0]*a[0][2]*a[2][1] 
		     + a[2][0]*a[0][1]*a[1][2] - a[2][0]*a[0][2]*a[1][1]);

#ifdef MATRIX3_NON_SAFE_INVERSE
  res[0][0] =  -(-a[1][1]*a[2][2] + a[1][2]*a[2][1])*scaler;
  res[1][0] =  -( a[1][0]*a[2][2] - a[1][2]*a[2][0])*scaler;
  res[2][0] =   ( a[1][0]*a[2][1] - a[1][1]*a[2][0])*scaler;
  res[0][1] =   (-a[0][1]*a[2][2] + a[0][2]*a[2][1])*scaler;
  res[1][1] =  -(-a[0][0]*a[2][2] + a[0][2]*a[2][0])*scaler;
  res[2][1] =   (-a[0][0]*a[2][1] + a[0][1]*a[2][0])*scaler;
  res[0][2] =  -(-a[0][1]*a[1][2] + a[0][2]*a[1][1])*scaler;
  res[1][2] =   (-a[0][0]*a[1][2] + a[0][2]*a[1][0])*scaler;
  res[2][2] =  -(-a[0][0]*a[1][1] + a[0][1]*a[1][0])*scaler;
#else
  double tmp[3][3];
  tmp[0][0] =  -(-a[1][1]*a[2][2] + a[1][2]*a[2][1])*scaler;
  tmp[1][0] =  -( a[1][0]*a[2][2] - a[1][2]*a[2][0])*scaler;
  tmp[2][0] =   ( a[1][0]*a[2][1] - a[1][1]*a[2][0])*scaler;
  tmp[0][1] =   (-a[0][1]*a[2][2] + a[0][2]*a[2][1])*scaler;
  tmp[1][1] =  -(-a[0][0]*a[2][2] + a[0][2]*a[2][0])*scaler;
  tmp[2][1] =   (-a[0][0]*a[2][1] + a[0][1]*a[2][0])*scaler;
  tmp[0][2] =  -(-a[0][1]*a[1][2] + a[0][2]*a[1][1])*scaler;
  tmp[1][2] =   (-a[0][0]*a[1][2] + a[0][2]*a[1][0])*scaler;
  tmp[2][2] =  -(-a[0][0]*a[1][1] + a[0][1]*a[1][0])*scaler;

  memcpy(res,tmp,9*sizeof(double));

#endif


}

inline double matrix3_det(const double a[3][3]){
  return (a[0][0]*a[1][1]*a[2][2] - a[0][0]*a[1][2]*a[2][1] 
        - a[1][0]*a[0][1]*a[2][2] + a[1][0]*a[0][2]*a[2][1] 
        + a[2][0]*a[0][1]*a[1][2] - a[2][0]*a[0][2]*a[1][1]);
}

//res = a'*b*a;
inline double matrix3_multiply2(const double b[3][3], const double a[3]){
  register double res = (a[0]*b[0][0]+a[1]*b[1][0]+a[2]*b[2][0])*a[0]
                       +(a[0]*b[0][1]+a[1]*b[1][1]+a[2]*b[2][1])*a[1]
                       +(a[0]*b[0][2]+a[1]*b[1][2]+a[2]*b[2][2])*a[2];

  return res;
}

inline void matrix3_multiply(double res[3],
                             const double a[3][3], 
       		             const double b[3]){
  double b0 = b[0];  double b1 = b[1];  double b2 = b[2];

  res[0] = a[0][0]*b0 + a[0][1]*b1 + a[0][2]*b2;
  res[1] = a[1][0]*b0 + a[1][1]*b1 + a[1][2]*b2;
  res[2] = a[2][0]*b0 + a[2][1]*b1 + a[2][2]*b2;
}

inline void vector3_add(double res[3], double a[3], double b[3]){
  res[0] = a[0] + b[0];
  res[1] = a[1] + b[1];
  res[2] = a[2] + b[2];
}

inline void vector3_subtract(double res[3], double a[3], double b[3]){
  res[0] = a[0] - b[0];
  res[1] = a[1] - b[1];
  res[2] = a[2] - b[2];
}

#endif
