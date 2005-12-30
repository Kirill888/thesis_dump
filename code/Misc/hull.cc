//////////////////////////////////////////////////////////
//
//  hull.cc
//
// Two in-place convex hull algorithm
// (Gift Wrap and QuickHull)
//
// (C) 2003 Pascal Massimino
// details at http://skal.planet-d.net/coding/hull.html
//////////////////////////////////////////////////////////

typedef double VECT2[2];

#include <stdlib.h>
#include <math.h>
static const VECT2 *Sort_Pts;  // bweuark! Damned qsort() API :(

//////////////////////////////////////////////////////////
// some utils

inline double Angle(const VECT2 P0, const VECT2 P1, const VECT2 P2)
{
    // "Angle" between P0->P1 and P0->P2.
    // actually returns: ||(P1-P0) ^ (P2-P0)||
  double dx21 = P2[0] - P1[0];
  double dy21 = P2[1] - P1[1];
  double dx10 = P1[0] - P0[0];
  double dy10 = P1[1] - P0[1];
  return (dy21*dx10 - dx21*dy10);
}

static int Compare(const VECT2 A, const VECT2 B) {
  if      (A[0]<B[0]) return  1;
  else if (A[0]>B[0]) return -1;
  else return 0;
}
static int Cmp_Lo(const void *a, const void *b) {
  return Compare( Sort_Pts[*(int*)a], Sort_Pts[*(int*)b] );
}
static int Cmp_Hi(const void *a, const void *b) { 
  return Compare( Sort_Pts[*(int*)b], Sort_Pts[*(int*)a] );
}

#define SWAP(i,j) { int Tmp = Ind[i]; Ind[i] = Ind[j]; Ind[j] = Tmp; }

//////////////////////////////////////////////////////////
// Gift Wrap algorithm
//////////////////////////////////////////////////////////

static int Sweep(int Nb, const VECT2 *Pts, int *Ind)
{
  int i, n;
  n = 1;
  for(i=n+1; i<Nb; ++i)
  {
    //    int Tmp;
      // search where to insert point #i into the chain Ind[0..n]
    while(n-->0)
      if ( Angle( Pts[Ind[i]], Pts[Ind[n]], Pts[Ind[n+1]] ) > 0.0 )
        break;

        // Triangle (n,n+1,n+2) will be clockwise.
    n += 2;
    SWAP(n,i);
  }
  return n;
}

int Convex_Hull_2D(int Nb, const VECT2 *Pts, int *Ind)
{  
  int n;
  if (Nb<=2) return Nb;

  Sort_Pts = Pts; // nasty. Only to overcome qsort()'s API

    // First sweep, to find lower boundary.
    // Points are sorted right to left.

  qsort(Ind, Nb, sizeof(int), Cmp_Lo);
  n = Sweep(Nb, Pts, Ind);


    // Second sweep, to find upper boundary
    // Actually, we sort the remaining [n..Nb] partition in
    // reverse order (left to right) -> The sweep loop is the same.

  Ind[Nb] = Ind[0]; // Close cycle with leftmost point
  qsort(Ind+n, Nb-n, sizeof(int), Cmp_Hi);
  n += Sweep(Nb+1-n, Pts, Ind+n);
  Ind[n] = Ind[Nb]; // restore

  return n;
}

//////////////////////////////////////////////////////////
// Quick Hull algorithm
//////////////////////////////////////////////////////////

static int QHull_Internal(int N, const VECT2 *Pts, int *Ind,
                          int iA, int iB)
{
  int n, nn, n2, m, Split1, Split2, iMid;
  int Mid_Pt;
  double dMax;

  if (N<=2) return N;

    // As middle point, search the farthest away from line [A-->B]

  Mid_Pt = -1;
  dMax = 0.0;
  for(n=1; n<N; ++n) {
    double d = fabs( Angle( Pts[Ind[n]], Pts[iA], Pts[iB] ) );
    if (d>=dMax) { dMax = d; Mid_Pt = n; }
  }

    // Partition against midpoint

  iMid = Ind[Mid_Pt];
  Ind[Mid_Pt] = Ind[N-1];

    //Ind = [A|...........|xxx] (B)
    //  n = [0|1.......N-2|N-1|

  nn = N-2;
  n  = 1;
  while(n<=nn) {
    double d = Angle( Pts[Ind[n]], Pts[iA], Pts[iMid] );
    if (d>=0.0) { SWAP(n,nn); nn--; }
    else n++;
  }
  Ind[N-1] = Ind[n];
  Ind[n] = iMid;
  Split1 = n++;

    //Ind = (A) [..(+)..| M |...........] (B)
    //  n =     [1......|Sp1|........N-1]

  nn = N-1;
  while(n<=nn) {
    double d = Angle( Pts[Ind[n]], Pts[iMid], Pts[iB] );
    if (d>=0.0) { SWAP(n,nn); nn--; }
    else n++;
  }
  Split2 = n;

    // Status:
    //Ind = (A) [....(+)...| M |....(-)....|(trash)......] (B)
    //  N =     [1.........|Sp1|...........|Sp2.......N-1]

    // Recurse each sub-partition

  n  = QHull_Internal(Split1,        Pts, Ind       , iA, iMid);
  n2 = QHull_Internal(Split2-Split1, Pts, Ind+Split1, iMid, iB);
  m = Split1;
  while(n2-->0) {
    SWAP(n,m);
    m++; n++;
  }

  return n;
}

int Quick_Hull_2D(int Nb, const VECT2 *Pts, int *Ind)
{
  int n, nn, m, iA, iB;

  if (Nb<=2) return Nb;

  Sort_Pts = Pts; // nasty. Only to overcome qsort()'s API
  qsort(Ind, Nb, sizeof(int), Cmp_Lo);

    // first partitioning: classify points with respect to
    // the line joining the extreme points #0 and #Nb-1

  iA = Ind[0];
  iB = Ind[Nb-1];

  m = Nb-2;
  n = 1;
  while(n<=m) {
    double d = Angle( Pts[Ind[n]], Pts[iA], Pts[iB] );
    if (d>=0.0) { SWAP(n,m); m--; }
    else n++;
  }
  Ind[Nb-1] = Ind[n];
  Ind[n] = iB;

    // Partition is now:
    //  Ind = [ A|...(+)....[B]...(-)...|A ]
    //   n  = [0 |1.........[n].........|Nb]
    // We now process the two halves [A..(+)..B] and [B..(-)..A]

  m  = QHull_Internal(   n, Pts, Ind  , iA, iB); // 1st half [A..(+)..B]
  nn = QHull_Internal(Nb-n, Pts, Ind+n, iB, iA); // 2nd half [B..(-)..A]

  while(nn-->0) {
    SWAP(m,n);
    m++; n++;
  }
  return m;
}


#if 0
//////////////////////////////////////////////////////////

int main()
{
#define NB_PTS  100
  VECT2 Pts[NB_PTS];
  int Ind[NB_PTS+1];
  int n, i;
  for(i=0; i<NB_PTS; ++i) {
    Pts[i][0] = (rand() & 255)/255.0;
    Pts[i][1] = (rand() & 255)/255.0;
    Ind[i] = i;
  }

  n = Convex_Hull_2D(NB_PTS, (const VECT2*)Pts, Ind);
// n = Quick_Hull_2D(NB_PTS, (const VECT2*)Pts, Ind);

    // for 'gnuplot' display
  for(i=0; i<n; ++i)
    printf( "%f %f\n", Pts[Ind[i]][0], Pts[Ind[i]][1] );
    // close the cycle
  printf( "%f %f\n", Pts[Ind[0]][0], Pts[Ind[0]][1] );
  return 0;
}

//////////////////////////////////////////////////////////
#endif
