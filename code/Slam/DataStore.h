#ifndef __DATA_STORE_H__
#define __DATA_STORE_H__

#ifndef DEPEND
#include <stdio.h>
#endif

#include "geometry.h"
#include "debug.h"
#include "memstat.h"

class ObservationInterface;

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
class ObservationStore{
 private:
  ObservationInterface **obs;
  int *obs_t;
  int n_data;
  int n_size;

  int size_increment;

  int scanOffset;

  void destroy();
  void make_copy(const ObservationStore &other);
  
 public:
  ObservationStore(){
    obs = NULL;
    obs_t = NULL;
    n_data = 0;
    n_size = 0;
    size_increment = 100;
    scanOffset = 0;
    resize(100);
  }

  ObservationStore(int initial_size, int increment){
    obs = NULL;
    obs_t = NULL;
    n_data = 0;
    n_size = 0;
    size_increment = increment;
    scanOffset = 0;
    resize(initial_size);
  }

  ObservationStore(const ObservationStore &other){
      make_copy(other);
  }
  
  const ObservationStore & operator=(const ObservationStore &rhs){
    if(this == &rhs) return *this;
    
    destroy();
    make_copy(rhs);
    return *this;
  }
  
  ~ObservationStore(){
      destroy();
  }
  

  void reset(){ destroy(); }
  void resize(int size);
     
  int numObs()const{return n_data;}
  int numScans()const{ 
    if(n_data == 0) return 0;

    return obs_t[n_data-1] - obs_t[0] + 1;
  }

  bool isFull()const{
    return n_data == n_size;
  }

  void startScan(){
    scanOffset = n_data; //Where the next obs will be added
  }

  int add(ObservationInterface *e, int t){
    if(isFull()) resize(n_size + size_increment);

    obs[n_data]   = e;
    obs_t[n_data] = t;

    n_data += 1;

    return (n_data-1);
  }

  int numObsLastScan()const{ return n_data - scanOffset; }
  int scan2Ind(int ind_scan)const{ return ind_scan + scanOffset;}
  const ObservationInterface *const* getLastScan()const{
    return obs + scanOffset;
  }

  const ObservationInterface * operator[](int i)const{return obs[i];}
  const ObservationInterface * get(int i)const{return obs[i];}
  const ObservationInterface *getLast()const{ return obs[n_data-1]; }

  void copyToArray(ObservationInterface *obs_out, int *t_out)const;

  int getScanId(int i)const{ return obs_t[i];}
  int getLastScanId()const{ return obs_t[n_data-1];}

  bool dumpToFile(const char* fname)const;
  bool dumpToFile(FILE *f)const;

  bool matlabDump(const char* fname, const char *varName)const;
  bool matlabDump(FILE *f, const char *varName)const;
};

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
class OdometryStoreNode{
 private: 
  RobotPose pose;
  OdometryStoreNode *parent;
  mutable int copyCount;
     
 public:

  OdometryStoreNode(){
    parent = NULL;
    copyCount = 1;

    MEM_STAT_ADD(odo);
  }

  OdometryStoreNode(RobotPose odo,  OdometryStoreNode *Parent){
    pose = odo;
    copyCount = 1;
    parent = Parent;

    MEM_STAT_ADD(odo);
  }

  ~OdometryStoreNode(){
    MEM_STAT_REMOVE(odo);
  }

  void makeCopy()const{
    copyCount++;
  }

  const RobotPose& getPose()const{return pose;}

  const RobotPose *getPose_p()const{return &pose;}
  operator const RobotPose*()const{return &pose;}

  operator const RobotPose&()const{return pose;}

  const OdometryStoreNode* getParent()const{return parent;}

  static void destroy(OdometryStoreNode *n);
};

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
class OdometryStore{
 private:
  OdometryStoreNode *head; 
  int n;

 public:
     
  OdometryStore(){
    head = NULL;
    n = 0;
  }

  OdometryStore(const OdometryStore &copy){
    if(copy.head != NULL){
      copy.head->makeCopy();
      head = copy.head;
      n = copy.n;
    }else{
      head = NULL;
      n = 0;
    }
  }

  ~OdometryStore(){
    OdometryStoreNode::destroy(head);
    //	DEBUG_error("OdometryStore::destructor (%p)\n",this);
  }

  void add(const RobotPose &odo){
    head = new OdometryStoreNode(odo, head);
    n += 1;
  }

  const RobotPose& getLastPose()const{
    return head->getPose();
  }

  const OdometryStoreNode* getLastNode()const{return head;}

  const OdometryStore& operator=(const OdometryStore &rhs){
    if(&rhs == this) return *this;
    
    OdometryStoreNode::destroy(head);
    if( rhs.head != NULL){
      rhs.head->makeCopy();
      head = rhs.head;
      n = rhs.n;
    }else{
      head = NULL;
      n = 0;
    }
	
    //	DEBUG_error("OdometryStore= (%p = %p)\n"
    //	            ,this, &lhs);

    return *this;
  }

  int numOdo()const{ return n;}

  OdometryStore getReverse()const;

  void copyToArray(RobotPose *odo_out,
                   int n_odo,
		   bool lastFirst = false)const;

  bool dumpToFile(const char* fname)const;
  bool dumpToFile(FILE* f)const;

  bool matlabDump(const char *fname, const char *varName)const;
  bool matlabDump(FILE *f, const char *varName)const;

  void reset(){
    OdometryStoreNode::destroy(head);
    head = NULL;
    n = 0;
  }

};

//-----------------------------------------------------------------
//
//-----------------------------------------------------------------
typedef void * GenericStackType;

class GenericStack{
 private:
  struct Entry{
    struct Entry *next;
    mutable int copyCount;
    GenericStackType data;

    Entry():next(NULL),copyCount(1){;}
    Entry(GenericStackType d, struct Entry *n):next(n),copyCount(1),data(d){;}
  };
  struct Entry *head;
  int n;

  void (*data_destructor)(GenericStackType);

 public:

  class Enumeration{
  private:
    const struct Entry *head;

  public:
    Enumeration(const struct Entry *e): head(e){;}

    bool hasMoreElements(){
      return head != NULL;
    }

    GenericStackType next(){
      GenericStackType tmp = head->data;
      head = head->next;
      return tmp;
    }
  };

  GenericStack():head(NULL),n(0),data_destructor(NULL){;}

  GenericStack(const GenericStack &other):head(NULL),n(0)
                          ,data_destructor(NULL){
    copy(&other);
  }

  GenericStack(const GenericStack *other):head(NULL),n(0)
                          ,data_destructor(NULL){
    copy(other);
  }

  GenericStack(void (*destructor)(GenericStackType) ):
	       head(NULL),n(0),data_destructor(destructor){;}

  const GenericStack& operator=(const GenericStack &rhs){
    if(&rhs != this){
       destroy();
       copy(&rhs);
    }

    return *this;
  }

  GenericStack* clone()const{ return new GenericStack(this); }

  ~GenericStack(){destroy();}

  void setDataDestructor(void (*destructor)(GenericStackType)){
    data_destructor = destructor;
  }

  void add(GenericStackType t){
    head = new struct Entry(t,head);
    n += 1;
  }

  void reset(){
    destroy();
    head = NULL;
    n = 0;
  }

  int numElements()const{return n;}

  GenericStackType getLast()const{
    return head->data;
  }

  void getAll(GenericStackType *out)const{
    struct Entry *e;
    int i;
    i = n - 1;
    for(e = head; e != NULL; e = e->next){
      out[i] = e->data;
      i -= 1;
    }
  }

  void getAllReverse(GenericStackType *out)const{
    struct Entry *e;
    int i;
    i = 0;
    for(e = head; e != NULL; e = e->next){
      out[i] = e->data;
      i += 1;
    }
  }

  Enumeration getElements()const{
    return Enumeration(head);
  }

 private:

  void destroy(){
    struct Entry *e = head;

    while(e != NULL){
      e->copyCount -= 1;
      if(e->copyCount > 0) return;

      struct Entry *tmp = e->next;
      if(data_destructor != NULL) data_destructor(e->data);
      delete e;

      e = tmp;
    }
  }

  void copy(const GenericStack* other){
    head = other->head;
    n = other->n;
    if(head != NULL) head->copyCount += 1;

    data_destructor = other->data_destructor;
  }

};


#endif






