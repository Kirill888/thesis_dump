#include "bisearch.h"

static int log2i(int a){
  int i = -1;

  while(a != 0){
    a = a >> 1;
    i += 1;
  }

  return i;
}

static int roundup_pw2(int a){
  int v = 1<<log2i(a);

  if(v == a) return a;
  else       return v<<1;

}

struct BinarySearch::BiNode *BinarySearch::build_tree(int start, int end){

  if( end - 1 <= start ){
    return new BiNode(NULL, NULL, start);

  }else{

    struct BiNode *n = new BiNode();
    int s, e;

    int np = roundup_pw2(end - start + 1);

    s = start;
    e = s + np/2 - 1;

    n->left = build_tree(s,e);
    n->ind  = e;

    s = e + 1;
    e = end;
      
    n->right = build_tree(s,e);

    return n;

  }
}

void BinarySearch::destroy_tree(struct BinarySearch::BiNode *n){
  if(n == NULL) return;

  destroy_tree(n->left);
  destroy_tree(n->right);

  delete n;
}

void BinarySearch::destroy_tree(){
  destroy_tree(head);
  head = NULL;
}

void BinarySearch::build_tree(){
  head = build_tree(0,size-1);
}

BinarySearch::BinarySearch(const double *a, int sz){
  search_array = a;
  size         = sz;

  build_tree();
}


void BinarySearch::set(const double *a, int sz){
  destroy_tree();

  search_array = a;
  size         = sz;

  if(search_array != NULL){
     build_tree();
  }else{
    head = NULL;
  }
}

int BinarySearch::find(double v)const{

  //  printf("Find %f\n", v);

  if(head == NULL) return -1;

  //  printf("  head is not null\n");

  if( v > search_array[size - 1]) return -1;

  struct BiNode *n = head;

  while(n->left != NULL){
     
    //     printf("  compare %f\n", search_array[n->ind]);

    if( v <= search_array[n->ind]){  //Go LEFT
      n = n->left;
    }else{                          //Go RIGHT
      n = n->right;
    }
  }

  if( v <= search_array[n->ind]){  //Found to the LEFT
    return n->ind;
  }else{
    return n->ind + 1;
  }
}
