#ifndef __INC_TEMPLATE_STORE_h
#define __INC_TEMPLATE_STORE_h

#include "DROSImage.h"
#include "DROSImageIO.h"

class TemplateStore
{
 public:
  TemplateStore();
  TemplateStore(char * template_repository_path);
  ~TemplateStore();
  double match(int id1, int id2); // id is the line number of the
                                  // template in the points file,
                                  // encoded as the template number
                                  // in the filename.
  bool IDExists(int id);
  int numberOfTemplates();
  int getClosestMatch(int id);
  

 protected:


 private:
  int m_number_of_ids;
  double ** m_match_array;
  double calculateNCC(int id1, int id2);
  char * m_repository;
  DROSImage * one;
  DROSImage * two;

};
 
#endif
