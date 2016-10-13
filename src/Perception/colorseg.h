/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#ifndef MLR_libcolorseg_h
#define MLR_libcolorseg_h

#include <Core/array.h>

//----- patch analysis
void pch2img(byteA &img, const uintA &pch, floatA &pch_colormap);
void random_colorMap(floatA& pch_colormap, uint np);
uint incremental_patch_ids(uintA& pch);
void get_patch_colors(floatA& pch_col, byteA& img, uintA& pch, uint np);
void patch_color_statistics(arr& stats, const uintA& patches, const byteA& image);
void get_patch_centroids(arr& pch_cen, uintA& pch, uint np);

uint get_single_color_segmentation(uintA& segmentation,  // segmented image
				   const byteA& image,   // input image
				   float sigma = 0.75,   // (Gaussian!?) blurring factor
				   float k = 500,        // similarity threshold
				   int min = 200         // min. no. of pixels per segment
				   );

uint get_single_color_segmentation_rgb(uintA& segmentation,  // segmented image
				       byteA& rgb,           // avg. RGB values assigned to segments
				       const byteA& image,   // input image
				       float sigma = 0.75,   // (Gaussian!?) blurring factor
				       float k = 500,        // similarity threshold
				       int min = 200         // min. no. of pixels per segment
				       );



void colorize_patches(byteA& coloration, const uintA& patches, const arr& stats);

typedef mlr::Array<uintA> MultiSegmentations;
void get_multiple_color_segmentations(MultiSegmentations& segmentations,  // scale-hierarchy of segmented input
				      const byteA& image,                 // input image
				      const arr& sigma,
				      const arr& k,
				      const intA& min
				      );

#endif

