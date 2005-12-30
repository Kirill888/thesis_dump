#ifndef DEPEND
#include <sys/dir.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif

#include "TemplateStore.h"



TemplateStore::TemplateStore()
{
  m_number_of_ids = 0;
}

int TemplateStore::numberOfTemplates()
{
  return m_number_of_ids;
}

int TemplateStore::getClosestMatch(int id)
{
  int i;
  int match_number = -1;
  double match_value = -1.0;
  for (i=0;i<m_number_of_ids;i++)
  {
    if (i != id)
    {
      double val = match(id,i);
      //printf("%i, %i, %lf\n",id,i,val);
      if (val > match_value)
      {
	match_number = i;
	match_value = val;
      }
    }
  }
  return match_number;
}

TemplateStore::TemplateStore(char * template_repository_path)
{

  one = new DROSImage();
  two = new DROSImage();

  struct direct **files;
  m_repository = template_repository_path;
  int file_select();
  int count = scandir(template_repository_path, &files, NULL,NULL);
  if (count > 0)
  {
    int i;
    int pnm_count = 0;
    for (i=0;i<count;i++)
    {
      char * d_name = files[i]->d_name;
      if (strstr(d_name,".pnm") != NULL)
      {
	pnm_count++;
      }
    }
    m_number_of_ids = pnm_count;
  }
  else
  {
    m_number_of_ids = 0;
  }
  m_match_array = (double**)malloc(sizeof(double*)*m_number_of_ids);
  int i;
  for (i=0;i<m_number_of_ids;i++)
  {
    m_match_array[i] = (double*)malloc(sizeof(double)*(i+1));

    //Set to NaN
    memset(m_match_array[i],0xFF,sizeof(double)*(i+1));
  }

  //Clean up
  for (i=0;i<count;i++)
  {
    free(files[i]);
  }
  free(files);

}


TemplateStore::~TemplateStore()
{
  int i;
  for (i=0;i<m_number_of_ids;i++)
  {
    free(m_match_array[i]);
  }
  free(m_match_array);

  delete(one);
  delete(two);
}

double TemplateStore::calculateNCC(int id1, int id2)
{
  /// check the output from here to ensure that we do not need to add and abs function.



  char path_one[255], path_two[255];
  sprintf(path_one,"%s/template%i.pnm",m_repository,id1);
  sprintf(path_two,"%s/template%i.pnm",m_repository,id2);

  if (DILoad(path_one,one) < 0 || DILoad(path_two,two) < 0)
  {
    printf("failed to load templates\n");
    return -1.0;
  }

  double mean_one_r = 0;
  double mean_two_r = 0;
  double mean_one_g = 0;
  double mean_two_g = 0;
  double mean_one_b = 0;
  double mean_two_b = 0;
  int i;
  for (i=0;i<(one->Width()*one->Height()*one->BytesPerPixel());i++)
  {
    double mean_one = one->Data()[i];
    double mean_two = two->Data()[i];

    if (i%3 == 0)
    {
      // red channel
      mean_one_r += mean_one;
      mean_two_r += mean_two;
    }
    else if(i%3 == 1)
    {
      // green channel
      mean_one_g += mean_one;
      mean_two_g += mean_two;
    }
    else
    {
      // blue channel
      mean_one_b += mean_one;
      mean_two_b += mean_two;
    }
  }
  int pixelcount = (one->Height()*one->Width());
  mean_one_r /= pixelcount;
  mean_one_g /= pixelcount;
  mean_one_b /= pixelcount;
  mean_two_r /= pixelcount;
  mean_two_g /= pixelcount;
  mean_two_b /= pixelcount;
 
  double var_one_r = 0;
  double var_two_r = 0;
  double sum_one_image_r =0;

  double var_one_g = 0;
  double var_two_g = 0;
  double sum_one_image_g =0;

  double var_one_b = 0;
  double var_two_b = 0;
  double sum_one_image_b =0;

  for (i=0;i<(one->Width()*one->Height()*one->BytesPerPixel());i++)
  {

    if (i%3 == 0)
    {
      double normalised_one_r = one->Data()[i] - mean_one_r;
      double normalised_two_r = two->Data()[i] - mean_two_r;
      sum_one_image_r += normalised_one_r*normalised_two_r;
      var_one_r += normalised_one_r*normalised_one_r;
      var_two_r += normalised_two_r*normalised_two_r;
    }
    else if (i%3 == 1)
    {
      double normalised_one_g = one->Data()[i] - mean_one_g;
      double normalised_two_g = two->Data()[i] - mean_two_g;
      sum_one_image_g += normalised_one_g*normalised_two_g;
      var_one_g += normalised_one_g*normalised_one_g;
      var_two_g += normalised_two_g*normalised_two_g;
    }
    else
    {
      double normalised_one_b = one->Data()[i] - mean_one_b;
      double normalised_two_b = two->Data()[i] - mean_two_b;
      sum_one_image_b += normalised_one_b*normalised_two_b;
      var_one_b += normalised_one_b*normalised_one_b;
      var_two_b += normalised_two_b*normalised_two_b;      
    }
  }
  double corr_r = (sum_one_image_r) / sqrt(var_two_r*var_one_r);
  double corr_g = (sum_one_image_g) / sqrt(var_two_g*var_one_g);
  double corr_b = (sum_one_image_b) / sqrt(var_two_b*var_one_b);
  double corr = (fabs(corr_r) + fabs(corr_g) + fabs(corr_b)) / 3.0;

  return corr;
 
}

double TemplateStore::match(int id1, int id2)
{

  int ind1,ind2;
  if (id1 > id2)
  {
    ind1 = id1;
    ind2 = id2;
  }
  else
  {
    ind1 = id2;
    ind2 = id1;
  }

  if ( isnan(m_match_array[ind1][ind2]) )
  {
    m_match_array[ind1][ind2] = calculateNCC(id1,id2);
    //printf("NCC calculated as %lf\n",calculateNCC(id1,id2));
  }

  return m_match_array[ind1][ind2];
}

bool TemplateStore::IDExists(int id)
{
  if (id < m_number_of_ids)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}
