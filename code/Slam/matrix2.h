#ifndef __MATRIX2_H__
#define __MATRIX2_H__

extern double ZEROS_2x2[2][2];
extern double I_2x2[2][2];

#define ZEROS_2x2_INIT {{0,0},{0,0}}
#define I_2x2_INIT     {{1,0},{0,1}}

typedef double MATRIX2x2[2][2];

void matrix2_set(double a[2][2], const double b[2][2]);
void matrix2_set0(double a[2][2]);

void matrix2_add(double res[2][2],
                 const double a[2][2], const double b[2][2]);
void matrix2_subtract(double res[2][2],
                      const double a[2][2], const double b[2][2]);
void matrix2_multiply(double res[2][2],
                      const double a[2][2], const double b[2][2]);
void matrix2_divide(double res[2][2],
                    const double a[2][2], const double b[2][2]);
//res = b*a*b'
void matrix2_multiply2(double res[2][2],
                      const double a[2][2], const double b[2][2]);

void matrix2_add(double res[2][2],
                 const double a[2][2], double b);
void matrix2_subtract(double res[2][2],
                      const double a[2][2], double b);
void matrix2_multiply(double res[2][2],
                      const double a[2][2], double b);
void matrix2_divide(double res[2][2],
		    const double a[2][2], double b);

void matrix2_multiply(double res[2],
                      const double a[2][2], 
		      const double b[2]);
void matrix2_multiply(double res[2],
                      const double a[2], 
		      const double b[2][2]);
// c = b'*a*b
double matrix2_multiply2(const double a[2][2], const double b[2]);

void matrix2_transpose(double res[2][2],
                       const double a[2][2]);
void matrix2_inverse(double res[2][2],
                     const double a[2][2]);

void matrix2_eigenvalues(double res[2], const double a[2][2]);
double matrix2_det(const double a[2][2]);
double matrix2_norm(const double a[2][2]);

void vector2_add(double res[2], double a[2], double b[2]);
void vector2_subtract(double res[2], double a[2], double b[2]);
void vector2_multiply(double res[2], double a[2], double b);
void vector2_divide(double res[2], double a[2], double b);

double mahalanobis2(const double x[2], const double cov[2][2]
		    , const double o[2]);

double mahalanobis2(const double dx[2], const double cov[2][2]);

double mahalanobis2_inv(const double x[2], const double inv[2][2]
			, const double o[2]);
double mahalanobis2_inv(const double dx[2], const double inv[2][2]);

/////////////////////////////////////////////////////////////////////////
// INLINED VERSION
////////////////////////////////////////////////////////////////////////

inline void matrix2_set0(double a[2][2]){
  a[0][0] = 0;
  a[0][1] = 0;
  a[1][0] = 0;
  a[1][1] = 0;
}

inline void matrix2_set(double a[2][2], const double b[2][2]){
  a[0][0] = b[0][0];
  a[0][1] = b[0][1];
  a[1][0] = b[1][0];
  a[1][1] = b[1][1];
}

inline void matrix2_add(double res[2][2],
			const double a[2][2], const double b[2][2]){
  res[0][0] = a[0][0] + b[0][0];
  res[0][1] = a[0][1] + b[0][1];
  res[1][0] = a[1][0] + b[1][0];
  res[1][1] = a[1][1] + b[1][1];
}

inline void matrix2_subtract(double res[2][2],
			     const double a[2][2], const double b[2][2]){
  res[0][0] = a[0][0] - b[0][0];
  res[0][1] = a[0][1] - b[0][1];
  res[1][0] = a[1][0] - b[1][0];
  res[1][1] = a[1][1] - b[1][1];
}

inline void matrix2_multiply(double res[2][2],
                             const double a[2][2], 
			     const double b[2][2]){
   
  double a00 = a[0][0];
  double a10 = a[1][0];
  double b00 = b[0][0];
  double b01 = b[0][1];

  res[0][0] = a00*b00 + a[0][1]*b[1][0];
  res[1][0] = a10*b00 + a[1][1]*b[1][0];
  res[0][1] = a00*b01 + a[0][1]*b[1][1];
  res[1][1] = a10*b01 + a[1][1]*b[1][1];
}
//res = b*a*b'
inline void matrix2_multiply2(double res[2][2],
			     const double a[2][2], const double b[2][2]){
  //(b00*a00+b01*a10)*b00+(b00*a01+b01*a11)*b01 
  //(b00*a00+b01*a10)*b10+(b00*a01+b01*a11)*b11
  //(b10*a00+b11*a10)*b00+(b10*a01+b11*a11)*b01 
  //(b10*a00+b11*a10)*b10+(b10*a01+b11*a11)*b11

  double t1,t2,t3,t4;
  double aux1,aux2;
  aux1 = b[0][0]*a[0][0] + b[0][1]*a[1][0];
  aux2 = b[0][0]*a[0][1] + b[0][1]*a[1][1];

  t1 = aux1*b[0][0] + aux2*b[0][1];
  t2 = aux1*b[1][0] + aux2*b[1][1];

  aux1 = b[1][0]*a[0][0] + b[1][1]*a[1][0];
  aux2 = b[1][0]*a[0][1] + b[1][1]*a[1][1];

  t3 = aux1*b[0][0] + aux2*b[0][1];
  t4 = aux1*b[1][0] + aux2*b[1][1];

  res[0][0] = t1;  res[0][1] = t2;
  res[1][0] = t3;  res[1][1] = t4;
}

// c = b'*a*b
inline double matrix2_multiply2(const double a[2][2], const double b[2]){
  //(b0*a00+b1*a10)*b0+(b0*a01+b1*a11)*b1

  double c = (b[0]*a[0][0]+b[1]*a[1][0])*b[0] +
             (b[0]*a[0][1]+b[1]*a[1][1])*b[1];

  return c;
}

inline void matrix2_divide(double res[2][2],
			   const double a[2][2], const double b[2][2]){
  double b_inv[2][2];
  matrix2_inverse(b_inv,b);
  matrix2_multiply(res,a,b_inv);
}

inline void matrix2_add(double res[2][2],
			const double a[2][2], double b){
  res[0][0] = a[0][0] + b;
  res[0][1] = a[0][1] + b;
  res[1][0] = a[1][0] + b;
  res[1][1] = a[1][1] + b;
}

inline void matrix2_subtract(double res[2][2],
			     const double a[2][2], double b){
  res[0][0] = a[0][0] - b;
  res[0][1] = a[0][1] - b;
  res[1][0] = a[1][0] - b;
  res[1][1] = a[1][1] - b;
}

inline void matrix2_multiply(double res[2][2],
			     const double a[2][2], double b){
  res[0][0] = a[0][0] * b;
  res[0][1] = a[0][1] * b;
  res[1][0] = a[1][0] * b;
  res[1][1] = a[1][1] * b;
}

inline void matrix2_divide(double res[2][2],
			   const double a[2][2], double b){
  matrix2_multiply(res,a,1.0/b);
}

inline void matrix2_multiply(double res[2],
			     const double a[2][2], 
			     const double b[2]){
  double b1 = b[0];
  double b2 = b[1];

  res[0] = a[0][0]*b1 + a[0][1]*b2;
  res[1] = a[1][0]*b1 + a[1][1]*b2;
}

inline void matrix2_multiply(double res[2],
			     const double a[2], 
			     const double b[2][2]){
  res[0] = b[0][0]*a[0] + b[1][0]*a[1];
  res[1] = b[0][1]*a[0] + b[1][1]*a[1];
}


inline void matrix2_transpose(double res[2][2],
			      const double a[2][2]){
  
  double a10 = a[1][0];

  res[0][0] = a[0][0];
  res[1][0] = a[0][1];
  res[0][1] = a10;
  res[1][1] = a[1][1];
}

inline void matrix2_inverse(double res[2][2],
			    const double a[2][2]){

  double a00 = a[0][0];
  double det_inv = 1.0/(a00*a[1][1] - a[0][1]*a[1][0]);

  res[0][0] = +a[1][1]*det_inv;
  res[1][0] = -a[1][0]*det_inv;
  res[0][1] = -a[0][1]*det_inv;
  res[1][1] = +a00*det_inv;
}

inline double matrix2_det(const double a[2][2]){
  double det = a[0][0]*a[1][1] - a[0][1]*a[1][0];
  return det;
}

inline double matrix2_norm(const double a[2][2]){
  double eigs[2];

  matrix2_eigenvalues(eigs,a);
  //  if(eigs[0]>eigs[1]) return eigs[0];
  return eigs[1];
}


inline void matrix2_eigenvalues(double res[2], const double a[2][2]){
  double aux1, aux2;

  //a^2-2*a*d+d^2  +4*b*c == (a-d)*(a-d) + 4*b*c

  aux1 = a[0][0] + a[1][1];
  aux2 = a[0][0] - a[1][1];
  aux2 = sqrt(4*a[0][1]*a[1][0] + aux2*aux2);

  res[0] = 0.5*(aux1 - aux2);
  res[1] = 0.5*(aux1 + aux2);
}

inline void vector2_add(double res[2], double a[2], double b[2]){
  res[0] = a[0] + b[0];
  res[1] = a[1] + b[1];
}
inline void vector2_subtract(double res[2], double a[2], double b[2]){
  res[0] = a[0] - b[0];
  res[1] = a[1] - b[1];
}
inline void vector2_multiply(double res[2], double a[2], double b){
  res[0] = a[0] * b;
  res[1] = a[1] * b;
}
inline void vector2_divide(double res[2], double a[2], double b){
  res[0] = a[0] / b;
  res[1] = a[1] / b;
}

inline double mahalanobis2(const double x[2], const double cov[2][2]
		    , const double o[2]){
  double inv[2][2];
  matrix2_inverse(inv, cov);
  return mahalanobis2_inv(x, inv, o);
}

inline double mahalanobis2(const double x[2], const double cov[2][2]){
  double inv[2][2];
  matrix2_inverse(inv, cov);
  return mahalanobis2_inv(x, inv);
}

inline double mahalanobis2_inv(const double x[2], const double inv[2][2]
		    , const double o[2]){

  double dx = x[0] - o[0];
  double dy = x[1] - o[1];

  return ( dx*( dx*inv[0][0] + dy*inv[1][0] ) +
	   dy*( dx*inv[0][1] + dy*inv[1][1] ));
}

inline double mahalanobis2_inv(const double dx[2], const double inv[2][2]){
  return ( dx[0]*( dx[0]*inv[0][0] + dx[1]*inv[1][0] ) +
	   dx[1]*( dx[0]*inv[0][1] + dx[1]*inv[1][1] ));
}

#endif
