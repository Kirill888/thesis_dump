#ifndef __KDTREE_H__
#define __KDTREE_H__

#ifndef DEPEND
#include <stdlib.h>
#endif

class KDTree{
 private:
  class Node{
  public:
    int dim;
    int ind;

    Node* left;
    Node* right;

    Node():dim(0),ind(-1),left(NULL),right(NULL){;}
    Node(int d, int i):dim(d),ind(i),left(NULL),right(NULL){;}
  };

  int K;

  int n_data;
  const double **DATA;
  double *buffer;

  Node *root;

  void split(int *ind, int n, int k /*usualy k=n/2*/, int dim);
  Node *buildTree(int *ind, int n, int dim);
  void destroyTree(Node*);

  int findNN(const Node* root,const double *query, const double *minBound, const double *maxBound, double* maxDist2)const;

  bool checkIntersection(const double *query, double r2, const double *minBound, const double *maxBound)const;
  double distance2(const double *query, int i)const;

 public:

  KDTree(int k):K(k),n_data(0), DATA(NULL), buffer(NULL){;}

  ~KDTree();

  //Init from a set of points
  void init(const double *const*points, int npoints, int dim = 0
           , bool makePrivateCopy = true);

  int findNN(const double *query, double *dist2)const;

  //Data access methods
  double get(int dim, int i)const{ return DATA[dim][i]; }

  //Shortcut methods for 2 and 3 dimension cases
  double X(int i)const{ return DATA[0][i]; }
  double Y(int i)const{ return DATA[1][i]; }
  double Z(int i)const{ return DATA[2][i]; }

  const double *getDim(int i)const{ return DATA[i]; }

};

#endif
