/*
  This code is described in "Computational Geometry in C" (Second Edition),
  Chapter 7.  It is not written to be comprehensible without the
  explanation in that book.

  Written by Joseph O'Rourke.
  Last modified: December 1997
  Questions to orourke@cs.smith.edu.
  --------------------------------------------------------------------
  This code is Copyright 1997 by Joseph O'Rourke.  It may be freely
  redistributed in its entirety provided that this copyright notice is
  not removed.
  --------------------------------------------------------------------
*/
#include        <stdio.h>
#include        <math.h>

#define X       0
#define Y       1

#define DEBUG(format, args...) printf(format,##args)
//#define DEBUG(format, args...)


typedef enum { Pin, Qin, Unknown } tInFlag;

typedef double  VECT2[2];   /* type double point */

//Interfaces
int     ConvexIntersect(VECT2* out, 
                        const VECT2* const P, int n, 
                        const VECT2* const Q, int m );





/*---------------------------------------------------------------------
  Function prototypes.
  ---------------------------------------------------------------------*/
double  Dot( const VECT2 a, const VECT2 b );
int	AreaSign( const VECT2 a, const VECT2 b, const VECT2 c );
bool    Between( const VECT2 a, const VECT2 b, const VECT2 c );
void    Assign( VECT2 p, const VECT2 a );
void    SubVec( const VECT2 a, const VECT2 b, VECT2 c );
bool    LeftOn( const VECT2 a, const VECT2 b, const VECT2 c );
bool    Left(   const VECT2 a, const VECT2 b, const VECT2 c );
tInFlag InOut(const VECT2 p, tInFlag inflag, int aHB, int bHA );


char    SegSegInt( const VECT2 a, const VECT2 b, 
                   const VECT2 c, const VECT2 d, 
                   VECT2 p, VECT2 q );

char    ParallelInt( const VECT2 a, const VECT2 b, 
                     const VECT2 c, const VECT2 d, 
                     VECT2 p, VECT2 q );


/////////////////////////////////////////////////////////////////////
// Inlines
////////////////////////////////////////////////////////////////////

inline void Assign( VECT2 p, const VECT2 a ){
  p[0] = a[0]; p[1] = a[1];
}

/*---------------------------------------------------------------------
  Returns TRUE iff point c lies on the closed segement ab.
  Assumes it is already known that abc are collinear.
  ---------------------------------------------------------------------*/
inline bool    Between( const VECT2 a, const VECT2 b, const VECT2 c )
{
  /* If ab not vertical, check betweenness on x; else on y. */
  if ( a[X] != b[X] )
    return ((a[X] <= c[X]) && (c[X] <= b[X])) ||
      ((a[X] >= c[X]) && (c[X] >= b[X]));
  else
    return ((a[Y] <= c[Y]) && (c[Y] <= b[Y])) ||
      ((a[Y] >= c[Y]) && (c[Y] >= b[Y]));
}


/*---------------------------------------------------------------------
  Returns the dot product of the two input vectors.
  ---------------------------------------------------------------------*/
inline double  Dot(const  VECT2 a, const VECT2 b)
{
  double sum = a[0]*b[0] + a[1]*b[1];
  return  sum;
}




/*---------------------------------------------------------------------
  Prints out the double point of intersection, and toggles in/out flag.
  ---------------------------------------------------------------------*/
inline tInFlag InOut( const VECT2 p, tInFlag inflag, int aHB, int bHA )
{

  /* Update inflag. */
  if      ( aHB > 0)
    return Pin;
  else if ( bHA > 0)
    return Qin;
  else    /* Keep status quo. */
    return inflag;
}

/*
  Returns true iff c is strictly to the left of the directed
  line through a to b.
*/
inline bool Left(const VECT2 a, const VECT2 b, const VECT2 c )
{
  return  AreaSign( a, b, c ) > 0;
}

inline bool LeftOn(const VECT2 a, const VECT2 b, const VECT2 c )
{
  return  AreaSign( a, b, c ) >= 0;
}

inline bool Collinear( const VECT2 a, const VECT2 b, const VECT2 c )
{
  return  AreaSign( a, b, c ) == 0;
}
/*---------------------------------------------------------------------
  a - b ==> c.
  ---------------------------------------------------------------------*/
inline void SubVec( const VECT2 a, const VECT2 b, VECT2 c )
{
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
}


inline int AreaSign( const VECT2 a, const VECT2 b, const VECT2 c )
{
  double area2;

  area2 = ( b[0] - a[0] ) * ( c[1] - a[1] ) -
          ( c[0] - a[0] ) * ( b[1] - a[1] );

  DEBUG("%%AreaSign = %e -- (%.2f,%.2f) (%.2f,%.2f)\n"
       ,area2,a[0],a[1],b[0],b[1]);

  /* The area should be an integer. */
  if      ( area2 >  0.00001 ) return  1;
  else if ( area2 < -0.00001 ) return -1;
  else                         return  0;
}

/*---------------------------------------------------------------------
  SegSegInt: Finds the point of intersection p between two closed
  segments ab and cd.  Returns p and a char with the following meaning:
  'e': The segments collinearly overlap, sharing a point.
  'v': An endpoint (vertex) of one segment is on the other segment,
  but 'e' doesn't hold.
  '1': The segments intersect properly (i.e., they share a point and
  neither 'v' nor 'e' holds).
  '0': The segments do not intersect (i.e., they share no points).
  Note that two collinear segments that share just one point, an endpoint
  of each, returns 'e' rather than 'v' as one might expect.
  ---------------------------------------------------------------------*/
char	SegSegInt( const VECT2 a, const VECT2 b, 
                   const VECT2 c, const VECT2 d, 
                   VECT2 p, VECT2 q )
{
  double  s, t;       /* The two parameters of the parametric eqns. */
  double num, denom;  /* Numerator and denoninator of equations. */
  char code = '?';    /* Return char characterizing intersection. */

  /*printf("%%SegSegInt: a,b,c,d: (%d,%d), (%d,%d), (%d,%d), (%d,%d)\n",
    a[X],a[Y], b[X],b[Y], c[X],c[Y], d[X],d[Y]);*/

  denom = a[X] * ( d[Y] - c[Y] ) +
          b[X] * ( c[Y] - d[Y] ) +
          d[X] * ( b[Y] - a[Y] ) +
          c[X] * ( a[Y] - b[Y] );

  /* If denom is zero, then segments are parallel: handle separately. */
  if (denom == 0.0)
    return  ParallelInt(a, b, c, d, p, q);

  num =    a[X] * ( d[Y] - c[Y] ) +
           c[X] * ( a[Y] - d[Y] ) +
           d[X] * ( c[Y] - a[Y] );
  if ( (num == 0.0) || (num == denom) ) code = 'v';
  s = num / denom;
  /*printf("num=%lf, denom=%lf, s=%lf\n", num, denom, s);*/

  num = -( a[X] * ( c[Y] - b[Y] ) +
	   b[X] * ( a[Y] - c[Y] ) +
	   c[X] * ( b[Y] - a[Y] ) );
  if ( (num == 0.0) || (num == denom) ) code = 'v';
  t = num / denom;
  /*printf("num=%lf, denom=%lf, t=%lf\n", num, denom, t);*/

  if      ( (0.0 < s) && (s < 1.0) &&
	    (0.0 < t) && (t < 1.0) )
    code = '1';
  else if ( (0.0 > s) || (s > 1.0) ||
	    (0.0 > t) || (t > 1.0) )
    code = '0';

  p[X] = a[X] + s * ( b[X] - a[X] );
  p[Y] = a[Y] + s * ( b[Y] - a[Y] );

  return code;
}


char   ParallelInt( const VECT2 a, const VECT2 b, 
                    const VECT2 c, const VECT2 d, 
                    VECT2 p, VECT2 q )
{
  if ( !Collinear( a, b, c) )
    return '0';

  if ( Between( a, b, c ) && Between( a, b, d ) ) {
    Assign( p, c );
    Assign( q, d );
    return 'e';
  }
  if ( Between( c, d, a ) && Between( c, d, b ) ) {
    Assign( p, a );
    Assign( q, b );
    return 'e';
  }
  if ( Between( a, b, c ) && Between( c, d, b ) ) {
    Assign( p, c );
    Assign( q, b );
    return 'e';
  }
  if ( Between( a, b, c ) && Between( c, d, a ) ) {
    Assign( p, c );
    Assign( q, a );
    return 'e';
  }
  if ( Between( a, b, d ) && Between( c, d, b ) ) {
    Assign( p, d );
    Assign( q, b );
    return 'e';
  }
  if ( Between( a, b, d ) && Between( c, d, a ) ) {
    Assign( p, d );
    Assign( q, a );
    return 'e';
  }
  return '0';
}


/*---------------------------------------------------------------------
  ---------------------------------------------------------------------*/
#define addPoint(p)   Assign(outPoly[nOutPoly++],p)

#define Advance(a, aa, n, inside, v) {\
 if(inside){ addPoint(v); DEBUG("%8.2lf    %8.2lf    lineto\n", v[X], v[Y] );}\
 aa++; a = (a+1) % n;}


int ConvexIntersect(VECT2* outPoly,
                    const VECT2* P, int n, 
                    const VECT2* Q, int m )
     /* P has n vertices, Q has m vertices. */
{
  int     a, b;           /* indices on P and Q (resp.) */
  int     a1, b1;         /* a-1, b-1 (resp.) */
  VECT2 A, B;           /* directed edges on P and Q (resp.) */
  int     cross;          /* sign of z-component of A x B */
  int     bHA, aHB;       /* b in H(A); a in H(b). */
  VECT2 Origin = {0,0}; /* (0,0) */
  VECT2 p;              /* double point of intersection */
  VECT2 q;              /* second point of intersection */
  tInFlag inflag;         /* {Pin, Qin, Unknown}: which inside */
  int     aa, ba;         /* # advances on a & b indices (after 1st inter.) */
  bool    FirstPoint;     /* Is this the first point? (used to initialize).*/
  VECT2 p0;             /* The first point. */
  int     code;           /* SegSegInt return code. */ 
  int nOutPoly = 0;

  /* Initialize variables. */
  a = 0; b = 0; aa = 0; ba = 0;
  inflag = Unknown; FirstPoint = true;

  do {
    /* Computations of key variables. */
    a1 = (a + n - 1) % n;
    b1 = (b + m - 1) % m;

    SubVec( P[a], P[a1], A );
    SubVec( Q[b], Q[b1], B );

    cross = AreaSign( Origin, A, B );
    aHB   = AreaSign( Q[b1], Q[b], P[a] );
    bHA   = AreaSign( P[a1], P[a], Q[b] );
    DEBUG("%%cross=%d, aHB=%d, bHA=%d\n", cross, aHB, bHA );

    /* If A & B intersect, update inflag. */
    code = SegSegInt( P[a1], P[a], Q[b1], Q[b], p, q );
    DEBUG("%%SegSegInt: code = %c\n", code );
    if ( code == '1' || code == 'v' ) {
      if ( inflag == Unknown && FirstPoint ) {
	aa = ba = 0;
	FirstPoint = false;
	p0[X] = p[X]; p0[Y] = p[Y];
	DEBUG("%8.2lf %8.2lf moveto\n", p0[X], p0[Y] );
      }

      DEBUG("%8.2lf %8.2lf lineto\n", p[X], p[Y] );
      addPoint(p); 

      inflag = InOut( p, inflag, aHB, bHA );
      DEBUG("%%InOut sets inflag=%d\n", inflag);
    }

    /*-----Advance rules-----*/

    /* Special case: A & B overlap and oppositely oriented. */
    if ( ( code == 'e' ) && (Dot( A, B ) < 0) ){
      addPoint(p);
      addPoint(q);
      return nOutPoly;
    }

    /* Special case: A & B parallel and separated. */
    if ( (cross == 0) && ( aHB < 0) && ( bHA < 0 ) ){
      DEBUG("%%P and Q are disjoint.\n");
      return nOutPoly;
    }

    /* Special case: A & B collinear. */
    else if ( (cross == 0) && ( aHB == 0) && ( bHA == 0 ) ) {
      /* Advance but do not output point. */
      if ( inflag == Pin ){
	Advance( b, ba, m, false, Q[b] );
      }else{
	Advance( a, aa, n, false, P[a] );
      }
    }

    /* Generic cases. */
    else if ( cross >= 0 ) {
      if ( bHA > 0){
	Advance( a, aa, n, inflag == Pin, P[a] );
      }else{
	Advance( b, ba, m, inflag == Qin, Q[b] );
      }
    }
    else /* if ( cross < 0 ) */{
      if ( aHB > 0){
	Advance( b, ba, m, inflag == Qin, Q[b] );
      }else{
	Advance( a, aa, n, inflag == Pin, P[a] );
      }
    }

    DEBUG("%%After advances:a=%d, b=%d; aa=%d, ba=%d; inflag=%d\n"
            , a, b, aa, ba, inflag);

    /* Quit when both adv. indices have cycled, or one has cycled twice. */
  } while ( ((aa < n) || (ba < m)) && (aa < 2*n) && (ba < 2*m) );

  if ( !FirstPoint ) /* If at least one point output, close up. */
    DEBUG("%8.2lf %8.2lf lineto\n", p0[X], p0[Y] );

  /* Deal with special cases: not implemented. */
  if ( inflag == Unknown) 
    DEBUG("%%The boundaries of P and Q do not cross.\n");

  return nOutPoly;
}




#ifdef CONVEX_POLY_MAIN
//Main

/*-------------------------------------------------------------------
  Main Function
 -------------------------------------------------------------------*/

void    ClosePostscript(FILE *);
void	OutputPolygons(FILE *, VECT2* P, int n, VECT2* Q, int m);
void    OutputPolygon(FILE *f, const VECT2* P, int n);
int     ReadPoly(VECT2*);

#define PMAX    1000        /* Max # of pts in polygon */


int main()
{
  int     	n, m;
  VECT2*	P, *Q;

  int nOutPoly = 0;
  VECT2* outPoly;
 


  FILE *f = stdout;
  P       = new VECT2[PMAX];
  Q       = new VECT2[PMAX];
  outPoly = new VECT2[PMAX];


  n = ReadPoly( P );
  m = ReadPoly( Q );
  OutputPolygons(f,P,n,Q,m);
  nOutPoly = ConvexIntersect( outPoly, P, n, Q,m);

  fprintf(f,"2 2 setlinewidth\n");
  fprintf(f,"%%Polygon: Intersection\n");
  OutputPolygon(f,outPoly,nOutPoly);

  ClosePostscript(f);
}


//In/Out


/*
  Reads in the coordinates of the vertices of a polygon from stdin,
  puts them into P, and returns n, the number of vertices.
  Formatting conventions: etc.
*/
int   ReadPoly( VECT2* P )
{
  int   n = 0;
  int   nin;

  scanf("%d", &nin);
  while ( (n < nin) && (scanf("%lf %lf",&P[n][0],&P[n][1]) != EOF) ) {
    ++n;
  }

  return   n;
}


void   OutputPolygons(FILE *f, VECT2* P, int n, VECT2* Q, int m)
{
  int i;
  double xmin, ymin, xmax, ymax;

  /* Compute Bounding Box for Postscript header. */
  xmin = xmax = P[0][X];
  ymin = ymax = P[0][Y];
  for (i = 1; i < n; i++) {
    if      ( P[i][X] > xmax ) xmax = P[i][X];
    else if ( P[i][X] < xmin ) xmin = P[i][X];
    if      ( P[i][Y] > ymax ) ymax = P[i][Y];
    else if ( P[i][Y] < ymin ) ymin = P[i][Y];
  }
  for (i = 0; i < m; i++) {
    if      ( Q[i][X] > xmax ) xmax = Q[i][X];
    else if ( Q[i][X] < xmin ) xmin = Q[i][X];
    if      ( Q[i][Y] > ymax ) ymax = Q[i][Y];
    else if ( Q[i][Y] < ymin ) ymin = Q[i][Y];
  }


  /* PostScript header */
  printf("%%!PS\n");
  fprintf(f,"%%%%Creator: convconv.c (Joseph O'Rourke)\n");
  fprintf(f,"%%%%BoundingBox: %8.2f %8.2f %8.2f %8.2f\n",
	  xmin, ymin, xmax, ymax);
  fprintf(f,"%%%%EndComments\n");
  fprintf(f,".00 .00 setlinewidth\n");
  fprintf(f,"%8.2f %8.2f translate\n", -xmin+100, -ymin+100 );
  /* The +100 shifts the figure from the lower left corner. */

  fprintf(f,"\n%%Polygon P:\n");
  OutputPolygon(f,P,n);


  fprintf(f,"\n%%Polygon Q:\n");
  OutputPolygon(f,Q,m);

}

void   OutputPolygon(FILE *f, const VECT2* P, int n){
  int i;
  fprintf(f,"newpath\n");
  fprintf(f,"%8.2f\t%8.2f\tmoveto\n", P[0][X], P[0][Y]);
  for( i = 1; i <= n; i++ )
    fprintf(f,"%8.2f\t%8.2f\tlineto\n", P[i%n][X], P[i%n][Y]);
  fprintf(f,"closepath stroke\n");
}

void   ClosePostscript(FILE *f)
{
  fprintf(f,"closepath stroke\n");
  fprintf(f,"showpage\n%%%%EOF\n");
}

#endif
