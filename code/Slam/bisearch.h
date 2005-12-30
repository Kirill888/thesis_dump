#ifndef __BISEARCH_H__
#define __BISEARCH_H__

#ifndef DEPEND
#include <stdlib.h>
#endif

class BinarySearch{

 private:

  struct BiNode{
    struct BiNode *left;
    struct BiNode *right;

    int ind;

    BiNode(){ left = right = NULL; ind = 0;}

    BiNode(struct BiNode *l, struct BiNode *r, int i){
      left  = l;
      right = r;
      ind   = i;
    }
  };

  struct BiNode *head;
  double const *search_array;
  int    size;

  void destroy_tree();
  void destroy_tree(struct BiNode *n);
  void build_tree();

  struct BiNode *build_tree(int start, int end);

 public:


  BinarySearch(){
    head = NULL;
    size = 0;
    search_array = NULL;
  }

  BinarySearch(const BinarySearch & other){
    size = other.size;
    search_array = other.search_array;

    build_tree();
  }

  const BinarySearch & operator=(const BinarySearch &other){
    if(this == &other) return *this;
    
    destroy_tree();

    size = other.size;
    search_array = other.search_array;

    build_tree();

    return *this;
  }

  BinarySearch(const double *a, int sz);

  ~BinarySearch(){
    destroy_tree(); 
  }

  int find(double v)const;
  void set(const double *a, int sz);


};


#endif
