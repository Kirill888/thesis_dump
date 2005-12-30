#ifndef DEPEND
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#endif
#include "kdtree.h"

inline void swap(int &a, int &b){register int tmp = a; a = b; b = tmp;}
extern const double Inf;
extern const double NaN;

KDTree::~KDTree(){
  if(buffer != NULL){ free(buffer); }
  if(DATA != NULL){ free(DATA); }

  destroyTree(root);
}
void KDTree::destroyTree(KDTree::Node *root){
  if(root != NULL){
    destroyTree(root->left);
    destroyTree(root->right);

    delete root;
  }
}

//Init from a set of points, dim -- first dimension to split
void KDTree::init(const double *const* points, int npoints, int dim
                  , bool makePrivateCopy){
  int i;

  //Allocate memory to store points
  if(buffer != NULL){ free(buffer); }
  if(DATA != NULL){ free(DATA); }

  if(makePrivateCopy){
    buffer = (double*)  malloc(sizeof(double)*npoints*K);
  }else{
    buffer = NULL;
  }

  DATA   = (const double**) malloc(sizeof(double*)*K);

  //Copy points
  for(i = 0; i < K; ++i){
    if(makePrivateCopy){
      DATA[i] = buffer + i*npoints;
      memcpy(buffer + i*npoints, points[i], npoints*sizeof(double));
    }else{
      DATA[i] = points[i];
    }
  }

  int *ind = new int[npoints];
  for(i = 0; i < npoints; ++i) ind[i] = i;

  root = buildTree(ind, npoints, dim);

  delete[] ind;
}

KDTree::Node* KDTree::buildTree(int *ind, int n, int dim){
//   printf("Build Tree: n = %d, dim = %d, ind = %p\n",n, dim,ind);

  if(n == 1){
    return new Node(dim, ind[0]);
  }

  int k = n/2;

  split(ind, n, k, dim);

  Node *node = new Node(dim,ind[k]);
  int dim2 = (dim + 1)%K;

  node->left  = buildTree(ind,  k, dim2);
  node->right = buildTree(ind + k, n - k, dim2);

  return node; 
}


int KDTree::findNN(const double *query, double *dist2)const{
  *dist2 = Inf;
  double min[K];
  double max[K];
  int i;

  for(i = 0; i < K; ++i){ min[i] = -Inf; max[i] = Inf; }

  return findNN(root, query, min, max, dist2);
}

bool KDTree::checkIntersection(const double *query, double r2, const double *minBound, const double *maxBound)const{
  int i;
  double dist2 = 0;

  if(r2 == Inf) return true;

//   printf("chk: %g,%g (%g,%g)->(%g,%g)", query[0],query[1],minBound[0],minBound[1], maxBound[0], maxBound[1]);

  for(i = 0; dist2 < r2 && i < K; ++i){
    if(query[i] < minBound[i]){
      double dd = minBound[i] - query[i];
      dist2 += (dd*dd);
    }else if(query[i] > maxBound[i]){
      double dd = maxBound[i] - query[i];
      dist2 += (dd*dd);
    }else{
      //inside this dimension
    }
  }

//   printf(" dist2 %g %g\n",dist2,r2);

  return (dist2 < r2);
}

double KDTree::distance2(const double *query, int pivot)const{
  int i;
  double d2 = 0;

  for(i = 0; i < K; ++i){
    double dd = query[i] - DATA[i][pivot];
    d2 += (dd*dd);
  }

  return d2;
}

int KDTree::findNN(const Node* root,const double *query, const double *minBound, const double *maxBound, double* dist2)const{
  if(root == NULL){
    *dist2 = Inf;
    return -1;
  }

  int s     = root->dim;
  int pivot = root->ind;

#if 0
  printf("findNN: s = %d, pivot = %d, dist = %g\n",s,pivot, *dist2);
  int i;

  for(i = 0; i < K; ++i){
    printf("%10.3f -> %10.3f\t\t %10.3f, %10.3f %s\n", minBound[i], maxBound[i], DATA[i][pivot], query[i], i == s?"*":"");
  }

#endif

  const Node *n1;
  const Node *n2;

  const double *min1; 
  const double *min2;
  const double *max1;
  const double *max2;

  double minLeft[K];
  double maxRight[K];

  memcpy(maxRight , maxBound, K*sizeof(double));
  memcpy(minLeft  , minBound, K*sizeof(double));
  minLeft[s]  = DATA[s][pivot];
  maxRight[s] = DATA[s][pivot];

  if( query[s] <= DATA[s][pivot]){
    n1 = root->left; n2 = root->right;
    min1 = minBound; max1 = maxRight;
    min2 = minLeft;  max2 = maxBound;
  }else{
    n2 = root->left; n1 = root->right;
    min1 = minLeft;  max1 = maxBound;
    min2 = minBound; max2 = maxRight;
  }

  int nearest = -1;
  double maxDist2 = *dist2;

  if(n1 != NULL){
//     printf("Left: \n");
    nearest = findNN(n1,query,min1, max1, dist2);

    if(nearest >= 0 && *dist2 < maxDist2){
      maxDist2 = *dist2;
    }

//     printf("resl: %d %g %g\n", nearest, *dist2, maxDist2);
  }

  if( checkIntersection(query, maxDist2, min2, max2) ){

    double d2 = distance2(query, pivot);
//     printf("dist %g,%g\n", d2, maxDist2);

    if(d2 < *dist2){ 
      *dist2 = d2;
      nearest = pivot;

      if(d2 < maxDist2) maxDist2 = d2;

//       printf("set %d %g\n", pivot, *dist2);
    }

    if(n2 != NULL){
      d2 = maxDist2;
//       printf("Right: \n");

      int tmp1 = findNN(n2, query, min2, max2, &d2);

//       printf("resr: %d %g\n", tmp1, d2);

      if(tmp1 >= 0 && d2 < *dist2){
	*dist2 = d2;
	nearest = tmp1;
      }
    }

  }

  return nearest;

}



/*********************************************************************
 * Sorts the data in such a way that DATA[dim][  0 -> k-1 ] < DATA[dim][k]
 *                                   DATA[dim][k+1 ->  n  ] > DATA[dim][k]
 *
 * Based on function kth_smallest() (util.cc) by N. Devillard.

                Reference:

                  Author: Wirth, Niklaus 
                   Title: Algorithms + data structures = programs 
               Publisher: Englewood Cliffs: Prentice-Hall, 1976 
    Physical description: 366 p. 
                  Series: Prentice-Hall Series in Automatic Computation 

 */
void KDTree::split(int *ind, int n, int k /*usualy k=n/2*/, int dim){
    register int i,j,l,m ;
    register double x ;
    const double *a = DATA[dim];

    l=0 ; m=n-1 ;
    while (l<m) {
        x=a[ind[k]] ;
        i=l ;
        j=m ;
        do {
            while (a[ind[i]] < x ) i++ ;
            while (x < a[ind[j]] ) j-- ;
            if (i<=j) {
                swap(ind[i],ind[j]) ;
                i++ ; j-- ;
            }
        } while (i<=j) ;
        if (j<k) l=i ;
        if (k<i) m=j ;
    }
}
