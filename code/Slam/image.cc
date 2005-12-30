#ifndef DEPEND
//#include <stdio.h>
#endif

#include "image.h"
#include "util.h"


void compute_window_offset(int* offset, int im_nr, int im_nc,
 		           int win_nr, int win_nc){
  int nc_2 = win_nc/2;
  int nr_2 = win_nr/2;

  int sz = win_nc*win_nr;

  int i,r,c;

  r = -nr_2;
  c = -nc_2;

  for(i = 0; i < sz; ++i){
    offset[i] = r*im_nc + c;

    c += 1;

    if(c > nc_2){
      c = -nc_2;
      r += 1;
    }
  }
}

void image_dilate(BYTE *image, const BYTE* image0, int nrows, int ncols,
	          const BYTE *kernel, int nr, int nc){
  int sz_kernel = nr*nc;
  int sz_image = nrows*ncols;
  int *offset = new int[sz_kernel];
  compute_window_offset(offset, nrows, ncols, nr,nc);

  BYTE* tmp = NULL;

  if(image == image0){
    tmp   = image;
    image = new BYTE[sz_image];
  }

  int i,j;
  int c,r;

  int nc_2 = nc/2;
  int nr_2 = nr/2;
  int max;


  int row_offset;
  int dr;

  for( r = 0, dr = nrows - 1; r < nrows; ++r, --dr){
    row_offset = r*ncols;
    int k_start = 0;
    int k_stop  = sz_kernel;

    if(nr_2 > r)         k_start = nc*(nr_2 - r);
    if( dr < nr_2) k_stop  = nc*(nr_2 + dr + 1);

    for(c = nc_2; c < ncols - nc_2; ++c){
      i = row_offset + c;

      max = 0;
      for(j = k_start; j < k_stop; ++j){
	int v = image0[i + offset[j]]*kernel[j];
	if(v > max) max = v;
      }
      image[i] = (BYTE) max;
    }

    //Process Sides
    for(c = 0; c < nc_2; ++c){           //Left side
      max = 0;
      i = row_offset + c;
      for(j = k_start; j < k_stop; ++j){
	int kc = j % nc; //Column of the kernel
	if(c + kc - nc_2 >= 0){
  	  int v = image0[i + offset[j]]*kernel[j];
  	  if(v > max) max = v;
	}
      }
      image[i] = (BYTE) max;
    }

    for(c = ncols - nc_2; c < ncols; ++c){ //Right Side
      max = 0;
      i = row_offset + c;
      for(j = k_start; j < k_stop; ++j){
	int kc = j % nc; //Column of the kernel
	if(c + kc - nc_2 < ncols){
  	  int v = image0[i + offset[j]]*kernel[j];
	  if(v > max) max = v;
	}
      }
      image[i] = (BYTE) max;
    }
  }

  if(tmp){
    memcpy(tmp, image, sizeof(BYTE)*sz_image);
    delete[] image;
  }

  delete[] offset;
}

//Translate mask to every pixel of the map, if  
//  ALL non-zero elements of the mask correspond to  non-zero elements 
//  of the image, copy these image pixels accross to new image.
void image_mask(BYTE *image, const BYTE* image0, int nrows, int ncols,
	        const BYTE *kernel, int nr, int nc){
  int sz_kernel = nr*nc;
  int sz_image = nrows*ncols;
  int *offset = new int[sz_kernel];
  compute_window_offset(offset, nrows, ncols, nr,nc);

  BYTE* tmp = NULL;

  if(image == image0){
    tmp   = image;
    image = new BYTE[sz_image];
  }

  int i,j;
  int c,r;

  int nc_2 = nc/2;
  int nr_2 = nr/2;


  //Init to zero
  memset(image, 0, sz_image*sizeof(BYTE));

  int row_offset;
  int ok ;

  for( r = nr_2; r < nrows - nr_2; ++r){
    row_offset = r*ncols;

    for(c = nc_2; c < ncols - nc_2; ++c){
      i = row_offset + c;

      ok = 1;
      for(j = 0; ok && j < sz_kernel; ++j){
	if(kernel[j] != 0){
	  ok = image0[offset[j] + i];
	}
      }

      if(ok){ //Mask fitted
	//Copy pixels accross
	for(j = 0; ok && j < sz_kernel; ++j){
	  if(kernel[j] != 0){
	    image[offset[j]+i] = image0[offset[j] + i];
	  }
	}
      }
    }
  }


  if(tmp){
    memcpy(tmp, image, sizeof(BYTE)*sz_image);
    delete[] image;
  }

  delete[] offset;
}


void image_convolve(BYTE *image,  const BYTE *image0, int nrows, int ncols,
	            const BYTE *kernel, int nr, int nc){
  int sz_kernel = nr*nc;
  int sz_image = nrows*ncols;
  int *offset = new int[sz_kernel];
  compute_window_offset(offset, nrows, ncols, nr,nc);

  BYTE * tmp = false;

  if(image == image0){
    tmp = image;
    image = new BYTE[sz_image];
  }

  int i,j;
  int c,r;

  int nc_2 = nc/2;
  int nr_2 = nr/2;
  int sum;


  int row_offset;
  int dr;

  for( r = 0, dr = nrows - 1; r < nrows; ++r, --dr){
    row_offset = r*ncols;
    int k_start = 0;
    int k_stop  = sz_kernel;

    if(nr_2 > r)         k_start = nc*(nr_2 - r);
    if( dr < nr_2) k_stop  = nc*(nr_2 + dr + 1);

    for(c = nc_2; c < ncols - nc_2; ++c){
      i = row_offset + c;

      sum = 0;
      for(j = k_start; j < k_stop; ++j){
	sum += image0[i + offset[j]]*kernel[j];
      }
      image[i] = (BYTE) sum;
    }

    //Process Sides
    for(c = 0; c < nc_2; ++c){           //Left side
      sum = 0;
      i = row_offset + c;
      for(j = k_start; j < k_stop; ++j){
	int kc = j % nc; //Column of the kernel
	if(c + kc - nc_2 >= 0){
  	  sum += image0[i + offset[j]]*kernel[j];
	}
      }
      image[i] = (BYTE) sum;
    }

    for(c = ncols - nc_2; c < ncols; ++c){ //Right Side
      sum = 0;
      i = row_offset + c;
      for(j = k_start; j < k_stop; ++j){
	int kc = j % nc; //Column of the kernel
	if(c + kc - nc_2 < ncols){
  	  sum += image0[i + offset[j]]*kernel[j];
	}
      }
      image[i] = (BYTE) sum;
    }

  }

  if(tmp){
    memcpy(tmp, image, sizeof(BYTE)*sz_image);
    delete[] image;
  }

  delete[] offset;
}

void image_getNonZeroBlob(BYTE *image, const BYTE* image0, int nrows, int ncols
			  ,int r,  int c){
  int i;

  int row_offset = r*ncols;
  int i_min, i_max;

  //Process Line  -- right
  for(i = row_offset + c; i < row_offset + ncols && image0[i] > 0; ++i){
    image[i] = image0[i];
  }
  i_max = i - 1;

  //Process Line -- left
  for(i = row_offset + c - 1; i >= row_offset  && image0[i] > 0; --i){
    image[i] = image0[i];
  }
  i_min = i + 1;

  int r_up = r + 1;   
  int r_down = r - 1; 

  //Go into recursion along the line
  for(i = i_min; i < i_max ; ++i){
    int col = i - row_offset;

    if(r_up < nrows && image0[i  + ncols] > 0 && 
                       image[i + ncols] == 0){
      image_getNonZeroBlob(image, image0, nrows, ncols, r_up, col);
    }

    if(r_down >= 0 && image0[i - ncols] > 0 &&
                      image[i - ncols] == 0){
      image_getNonZeroBlob(image, image0, nrows, ncols, r_down, col);
    }
  }
}

//bounds are set to: [r_min, c_min, r_max, c_max];
void image_getNonZeroBounds(int bounds[4], 
                            const BYTE* im, int nrows, int ncols){
  int r,c,i;
  int row_offset;
  bool empty;

  //Find first non-empty row.
  empty = true;
  for(r = 0; empty && r < nrows; ++r){
    row_offset = r*ncols;

    for(i = row_offset; empty && i < row_offset+ncols ; ++i){
      empty = (im[i] == 0);
    }
  }

  //Check if image is empty.
  if(r >= nrows){
    bounds[0] = nrows; bounds[2] = 0;
    bounds[1] = ncols; bounds[3] = 0;
    return;
  }

  //Set min row
  bounds[0] = r;

  //Find last non-empty row
  empty = true;
  for(r = nrows-1; empty && r > bounds[0]; --r){
    row_offset = r*ncols;

    for(i = row_offset; empty && i < row_offset + ncols; ++i){
      empty = (im[i] == 0);
    }
  }

  //Set max row
  bounds[2] = r;

  //Find first non-zero column
  empty = true;
  for(c = 0; empty && c < ncols; ++c){
    int i1 = bounds[0]*ncols;
    int i2 = bounds[2]*ncols;

    for(i = i1 + c; empty && i <= i2 + c; i += ncols){
      empty = (im[i] == 0);
    }
  }

  bounds[1] = c;

  //Find last non-zero column
  empty = true;
  for(c = ncols - 1; empty && c > bounds[1]; --c){
    int i1 = bounds[0]*ncols;
    int i2 = bounds[2]*ncols;

    for(i = i1 + c; empty && i <= i2 + c; i += ncols){
      empty = (im[i] == 0);
    }
  }

  bounds[3] = c;
}

//bounds = [r_min, c_min, r_max, c_max];
BYTE* image_subImage(const BYTE* im0, int *nrows, int *ncols, int bounds[4]){
  int nc = bounds[3] - bounds[1] + 1;
  int nr = bounds[2] - bounds[0] + 1;

  int bytesPerLine = nc*sizeof(BYTE);
  BYTE* im = new BYTE[nc*nr];

  int r0,r1;
  int nc0 = *ncols;
  int col0 = bounds[1];


  for(r0 = bounds[0], r1 = 0; r0 <= bounds[2]; ++r0, ++r1){
    memcpy(im + r1*nc, im0 + r0*nc0 + col0,  bytesPerLine);
  }

  *nrows = nr;  *ncols = nc;

  return im;
}
