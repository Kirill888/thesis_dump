#include <string.h>
#include "DataStore.h"
#include "map.h"

void ObservationStore::destroy(){
  if(obs != NULL){
    int i;

    for(i = 0; i < n_data; ++i) if(obs[i] != NULL) delete obs[i];

    delete[] obs;
    delete[] obs_t;

    obs = NULL;
    obs_t = NULL;

    n_data = n_size = 0;
  }
}

void ObservationStore::make_copy(const ObservationStore &other){
  int i;

  n_data = other.n_data;
  n_size = other.n_size;
  size_increment = other.size_increment;
  scanOffset = other.scanOffset;

  obs   = new ObservationInterface*[n_size];
  obs_t = new int[n_size];

  for(i = 0; i < n_data; ++i){
    obs[i]   = other[i]->clone();
    obs_t[i] = other.obs_t[i];
  }
}

void ObservationStore::resize(int size){
  if(obs != NULL){

    ObservationInterface **obs_n = new ObservationInterface*[size];
    int *obs_nt = new int[size];

    memcpy(obs_n,obs,n_data*sizeof(ObservationInterface*));
    memcpy(obs_nt,obs_t,n_data*sizeof(int));

    delete[] obs;
    delete[] obs_t;

    obs   = obs_n;
    obs_t = obs_nt;

  }else{
    obs   = new ObservationInterface*[size];
    obs_t = new int[size];
  }

  n_size = size;
}


void ObservationStore::copyToArray(ObservationInterface *obs_out,
				   int *t_out)const{
  int i;
  for(i = 0; i < n_data; ++i){
    obs_out[i]  = *obs[i];
    t_out[i] = obs_t[i];
  }
}

bool ObservationStore::dumpToFile(const char *fname)const{
  FILE *f = fopen(fname,"w");
  if(f == NULL) return false;
  bool res = dumpToFile(f);

  fclose(f);
  return res;

}

bool ObservationStore::dumpToFile(FILE *f)const{
  int i;

  for(i = 0; i < n_data; ++i){
    if(fprintf(f,"%d ", obs_t[i]) < 0) return false;
    if(!obs[i]->matlabDump(f)) return false;
    if(fprintf(f,"\n") < 0) return false;
  }

  return true;
}

bool ObservationStore::matlabDump(const char *fname, const char *varName)const{
  FILE *f = fopen(fname,"w");
  if(f == NULL) return false;
  bool res = matlabDump(f,varName);

  fclose(f);
  return res;

}

bool ObservationStore::matlabDump(FILE *f, const char *varName)const{
  if( n_data == 0){
    return fprintf(f,"%s = [];",varName) > 0;
  }else{
     
    if(fprintf(f,"%s = [ ...\n",varName) < 0) return false;
    if(!dumpToFile(f)) return false;

    return fprintf(f,"];\n") > 0;
  }
}


//////////////////////////////////////////////////////////////////////
// OdometryStore::
//////////////////////////////////////////////////////////////////////

void OdometryStoreNode::destroy(OdometryStoreNode *n){

  while( n != NULL){
    n->copyCount -= 1;

    if(n->copyCount > 0) return;

    OdometryStoreNode *tmp = n;
    n = n->parent;
    delete tmp;
  }
}



OdometryStore OdometryStore::getReverse()const{
  OdometryStore reverse;

  const OdometryStoreNode *n;

  for(n = head; n != NULL; n = n->getParent()){
    reverse.add(n->getPose());
  }

  return reverse;
}

void OdometryStore::copyToArray(RobotPose *odo_out,
				int n_odo,
				bool lastFirst)const{
  int i = 0;
  const OdometryStoreNode *n;

  if(lastFirst){
    for(n = head; n != NULL; n = n->getParent()){
      if(i >= n_odo){
	return;
      }

      odo_out[i] = n->getPose();
      i += 1;
    }
  }else{
    for(n = head; n != NULL; n = n->getParent()){
      i += 1;
      if( i > n_odo){
	return;
      }

      odo_out[n_odo - i] = n->getPose();
    }
  }
}

bool OdometryStore::dumpToFile(const char* fname)const{
  FILE *f = fopen(fname,"w");
  if(f == NULL) return false;
  bool res = dumpToFile(f);

  fclose(f);
  return res;
}

bool OdometryStore::dumpToFile(FILE* f)const{
  const OdometryStoreNode *n;

  for(n = head; n != NULL; n = n->getParent()){
    RobotPose p = n->getPose();

    if(fprintf(f,"%6.3f %6.3f %6.3f\n",
	       p.x , p.y , p.rot) < 0) return false;
  }

  return true;
}

bool OdometryStore::matlabDump(const char *fname, const char *varName)const{
  FILE *f = fopen(fname,"w");
  if(f == NULL) return false;
  bool res = matlabDump(f,varName);

  fclose(f);
  return res;
}

bool OdometryStore::matlabDump(FILE *f, const char *varName)const{
  bool res;

  res = fprintf(f,"%s = [...\n",varName) >0;
  res &= dumpToFile(f);
  res &= fprintf(f,"];\n") > 0;

  return res;
}

