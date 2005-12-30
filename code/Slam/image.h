#ifndef __IMAGE_H__
#define __IMAGE_H__

typedef unsigned char BYTE;

void image_convolve(BYTE *image, const BYTE* image0, int nrows, int ncols,
	            const BYTE *kernel, int nr, int nc);

void image_dilate(BYTE *image, const BYTE* image0, int nrows, int ncols,
	          const BYTE *kernel, int nr, int nc);

//Translate mask to every pixel of the map, if  
//  ALL non-zero elements of the mask correspond to  non-zero elements 
//  of the image, copy these image pixels accross to new image.
void image_mask(BYTE *image, const BYTE* image0, int nrows, int ncols,
	        const BYTE *mask, int nr, int nc);

//  image  should be set to all zero pixels, prior to the call
void image_getNonZeroBlob(BYTE *image, const BYTE* image0, int nrows, int ncols
               ,int start_row,  int start_col);

//bounds are set to: [r_min, c_min, r_max, c_max];
// if image is empty, bounds is set to [nrows ncols 0 0];
void image_getNonZeroBounds(int bounds[4], 
                            const BYTE* im, int nrows, int ncols);

BYTE* image_subImage(const BYTE* im0, int *nrows, int *ncols, int bounds[4]);
#endif
