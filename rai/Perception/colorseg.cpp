/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "colorseg.h"

//#define RAI_libcolorseg
#ifdef RAI_libcolorseg

// Felzenszwalb's files
#include "libcolorseg/image.h"
#include "libcolorseg/segment-image.h"

// Unsupervised graph-cut-based segmentation (single-scale)
//
// INPUT
//    const byteA& image         - zi input image
//    float sigma                - a blurring factor to pre-smoothen the image
//    float k                    - pixel-similarity threshold
//    int min                    - minimum size of segments
//
// OUTPUT
//    uintA& segmentation        - segmented output image
//    uint                       - number of segments
//
// NOTE
//  The segment indices of the output are enumerated from 0...N, i.e.
//  the number of segments can be queried by
//    uint num_segments = segmentation.p[segmentation.maxIndex()];
//
uint get_single_color_segmentation(uintA& segmentation,  // segmented image
                                   const byteA& image,   // input image
                                   float sigma,          // (Gaussian!?) blurring factor
                                   float k,              // similarity threshold
                                   int min               // min. no. of pixels per segment
                                  ) {
  // wrap Felzenszwalb's data around Marc's array class data
  felzenszwalb::image<felzenszwalb::rgb>* image_pff = new felzenszwalb::image<felzenszwalb::rgb>(image.d1, image.d0, -1);
  image_pff->data = (felzenszwalb::rgb*) image.p;
  image_pff->access = new felzenszwalb::rgb*[image.d0]; // allocate row pointers
  for(uint i = 0; i < image.d0; i++)
    image_pff->access[i] = image_pff->data + (i * image.d1);

  // apply segmentation
  int num_segments = 0;
  felzenszwalb::image<unsigned int>* seg_pff = segment_image(image_pff, sigma, k, min, &num_segments);
  uint* data = seg_pff->data;

  //extract segmentation with consecutive indices
  segmentation.resize(image.d0, image.d1);
  intA lut(image.d0 * image.d1);                     // look-up table for consecutive segment enumeration
  lut = -1;
  int seg_counter = 0;
  for(uint i = 0; i < segmentation.N; i++) {
    int idx = data[i];
    if(lut(idx) == -1) lut(idx) = seg_counter++;
    segmentation.p[i] = (uint) lut(idx);
  }

  CHECK_EQ(seg_counter, num_segments, "LUT assignment: seg_counter != (num_segments+1)");

  // unwrap data and clean up
  image_pff->data = nullptr;
  delete image_pff->access;
  image_pff->access = nullptr;
  delete image_pff;
  delete seg_pff;

  return num_segments;
}

uint get_single_color_segmentation_rgb(uintA& segmentation,  // segmented image
                                       byteA& rgb,
                                       const byteA& image,   // input image
                                       float sigma,          // (Gaussian!?) blurring factor
                                       float k,              // similarity threshold
                                       int min               // min. no. of pixels per segment
                                      ) {
  // get the normal segmentation of the image
  int num_segments = get_single_color_segmentation(segmentation, image, sigma, k, min);

  // determine average RGB values for each segment/patch
  intA rgb_avg(num_segments, 3);
  rgb_avg = 0;
  intA segment_sizes(num_segments);
  segment_sizes = 0;

  int img_idx = 0, seg_idx = 0;
  for(uint i = 0; i < segmentation.N; i++) {
    seg_idx = segmentation.p[i];
    segment_sizes(seg_idx) += 1;
    rgb_avg(seg_idx, 0) += image.p[img_idx++];
    rgb_avg(seg_idx, 1) += image.p[img_idx++];
    rgb_avg(seg_idx, 2) += image.p[img_idx++];
  }

  std::cout << rgb_avg(seg_idx, 0) << std::endl;
  for(int i = 0; i < num_segments; i++) {
    if(segment_sizes(i) <= 0)
      HALT("segment_sizes(i) <= 0");

    rgb_avg(i, 0) /= segment_sizes(i);
    rgb_avg(i, 1) /= segment_sizes(i);
    rgb_avg(i, 2) /= segment_sizes(i);
  }

  // assign average colors
  img_idx = 0; seg_idx = 0;
  rgb.resize(image.d0, image.d1, 3);
  for(uint i = 0; i < segmentation.N; i++) {
    seg_idx = segmentation.p[i];
    rgb.p[img_idx++] = rgb_avg(seg_idx, 0);
    rgb.p[img_idx++] = rgb_avg(seg_idx, 1);
    rgb.p[img_idx++] = rgb_avg(seg_idx, 2);
  }

  return (uint) num_segments;
}

void get_patch_centroids(arr& pch_cen, uintA& pch, uint np) {
  uint x, y, Y=pch.d0, X=pch.d1;
  pch_cen.resize(np, 2);  pch_cen.setZero();
  uintA  pch_size(np);    pch_size.setZero();
  for(y=0; y<Y; y++) for(x=0; x<X; x++) {
      pch_cen(pch(y, x), 0) += (double)x;
      pch_cen(pch(y, x), 1) += (double)y;
      pch_size(pch(y, x))++;
    }
  for(uint i=0; i<np; i++)
    if(pch_size(i)) pch_cen[i]()/=(double)pch_size(i);
}

// Determine per patch color statistics: mean RGB + standard deviation
//
// INPUT
//
// OUTPUT
//  doubleA& stats                - N-by-6 matrix, #N patches, 1-3: RGB, 4-6: std. dev.
//
void patch_color_statistics(doubleA& stats, const uintA& patches, const byteA& image) {
  uint num_patches = patches.p[patches.maxIndex()]+1;
  uint num_pixels = image.d0*image.d1;
  if(image.d2 != 3)
    NIY;

  stats.resize(num_patches, 6);                          // Avg. R, G, B, and std dev
  stats = 0;

  // sum RGB values
  intA sum_rgb(num_patches, 3), patch_sizes(num_patches);
  sum_rgb = 0;
  patch_sizes = 0;
  int patch_id = -1, counter = 0;
  for(uint i = 0; i < num_pixels; i++) {
    patch_id = patches.p[i];
    patch_sizes(patch_id) += 1;
    for(int j = 0; j < 3; j++)
      sum_rgb(patch_id, j) += image.p[counter+j];
    counter+=3;
  }

  // mean RGB-values for each patch
  for(uint i = 0; i < num_patches; i++)
    if(patch_sizes(i) > 0)
      for(int j = 0; j < 3; j++)
        stats(i, j) = (double) sum_rgb(i, j) / (double) patch_sizes(i);

  // RGB standard deviation
  double d;
  counter = 0;
  for(uint i = 0; i < num_pixels; i++) {
    patch_id = patches.p[i];
    for(int j = 0; j < 3; j++) {
      d = image.p[counter+j] - stats(patch_id, j);
      stats(patch_id, 3+j) += (d*d);
    }
    counter += 3;
  }
  for(uint i = 0; i < num_patches; i++)
    if(patch_sizes(i) > 0)
      for(int j = 3; j < 6; j++)
        stats(i, j) = sqrt(stats(i, j) / (double) patch_sizes(i));
}

void get_patch_colors(floatA& pch_col, byteA& img, uintA& pch, uint np) {
  uint i, Y=img.d0, X=img.d1, N=img.d0*img.d1;
  pch.reshape(N);
  img.reshape(N, 3);
  pch_col.resize(np, 3);  pch_col.setZero();
  uintA  pch_size(np);    pch_size.setZero();
  for(i=0; i<N; i++) {
    pch_col(pch(i), 0) += (float)img(i, 0);
    pch_col(pch(i), 1) += (float)img(i, 1);
    pch_col(pch(i), 2) += (float)img(i, 2);
    pch_size(pch(i))++;
  }
  for(i=0; i<np; i++)
    if(pch_size(i)) pch_col[i]()/=(float)pch_size(i);
  img.reshape(Y, X, 3);
  pch.reshape(Y, X);
}

// Assign the average RGB values as color for each patch
//
// INPUT
//
// OUTPUT
//
void colorize_patches(byteA& coloration, const uintA& patches, const doubleA& stats) {
  int x = patches.d1, y = patches.d0;
  coloration.resize(y, x, 3);
  int counter = 0;
  for(uint i = 0; i < patches.N; i++)
    for(int j = 2; j >= 0; j--)
      coloration.p[counter++] = stats(patches.p[i], j);
}

// Unsupervised graph-cut-based segmentation (multi-scale)
//
// INPUT
//    const byteA& image         - zi input image
//    floatA& sigma              - an array of blurring factors to pre-smoothen the image
//    floatA& k                  - an array of pixel-similarity thresholds
//    intA& min                  - an array of minimum sizes of segments
//
// NOTE
//    "sigma.N == k.N == min.N"
//
// OUTPUT
//    MultiSegmentations& segmentation - scale hierarchy of segmented input image
//
// NOTE
//  The segment indices of the output are enumerated from 0...N, i.e.
//  the number of segments at scale level i can be queried by
//    uint num_segments = segmentations.p[i].p[segmentation.maxIndex()];
//
void get_multiple_color_segmentations(
  MultiSegmentations& segmentations,  // scale-hierarchy of segmented input
  const byteA& image,                 // input image
  const doubleA& sigma,
  const doubleA& k,
  const intA& min
) {
  // TODO phtread to speed up segmentation (one thread per level)
  if((sigma.d0 != k.d0) || (sigma.d0 != min.d0))
    HALT("size of sigma, k, and min has to be equal");

  uint num_levels = sigma.d0;
  segmentations.resize(num_levels);
  for(uint i = 0; i < num_levels; i++)
    get_single_color_segmentation(segmentations.p[i], image, sigma.p[i], k.p[i], min.p[i]);
}

uint incremental_patch_ids(uintA& pch) {
  uint i, N=pch.N;
  uintA pch_old(pch);
  uintA old_ids;
  int idx;
  for(i=0; i<N; i++) {
    idx = old_ids.findValue(pch_old.elem(i));
    if(idx<0) {
      idx = old_ids.N;
      old_ids.append(pch_old.elem(i));
    }
    pch.elem(i) = idx;
  }
  return old_ids.N;
}

void pch2img(byteA& img, const uintA& pch, floatA& pch_col) {
  uint i, N=pch.d0*pch.d1;
  if(pch_col.nd==2) {
    img.resize(N, 3);
    for(i=0; i<N; i++) {
      img(i, 0) = (byte)pch_col(pch.elem(i), 0);
      img(i, 1) = (byte)pch_col(pch.elem(i), 1);
      img(i, 2) = (byte)pch_col(pch.elem(i), 2);
    }
    img.reshape(pch.d0, pch.d1, 3);
  }
  if(pch_col.nd==1) {
    img.resize(N);
    for(i=0; i<N; i++)
      img(i) = (byte)(255.f*pch_col(pch.elem(i)));
    img.reshape(pch.d0, pch.d1);
  }
}

void random_colorMap(floatA& pch_col, uint np) {
  pch_col.resize(np, 3);
  rndUniform(pch_col, 0, 255);
}

#else

#include "../Core/util.h"
void pch2img(byteA& img, const uintA& pch, floatA& pch_colormap) {NICO}
void random_colorMap(floatA& pch_colormap, uint np) {NICO}
uint incremental_patch_ids(uintA& pch) {NICO}
void get_patch_colors(floatA& pch_col, byteA& img, uintA& pch, uint np) {NICO}
void get_patch_centroids(doubleA& pch_cen, byteA& img, uintA& pch, uint np) {NICO}

uint get_single_color_segmentation(uintA& segmentation,
                                   const byteA& image,
                                   float sigma,
                                   float k,
                                   int min
                                  ) {NICO}

uint get_single_color_segmentation_rgb(uintA& segmentation,
                                       byteA& rgb,
                                       const byteA& image,
                                       float sigma,
                                       float k,
                                       int min) {NICO}

void get_patch_centers(arr& centers, const uintA& patches) {NICO}

void get_patch_centroids(arr& pch_cen, uintA& pch, uint np) {NICO}

void patch_color_statistics(arr& stats, const uintA& patches, const byteA& image) {NICO}

void colorize_patches(byteA& coloration, const uintA& patches, const arr& stats) {NICO}

typedef rai::Array<uintA> MultiSegmentations;
void get_multiple_color_segmentations(MultiSegmentations& segmentations,
                                      const byteA& image,
                                      const arr& sigma,
                                      const arr& k,
                                      const intA& min
                                     ) {NIY}

#endif
