#ifndef COLORSPACE_H
#define COLORSPACE_H

#include <stdint.h>
#include "pixel_format.h"

void bgr2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);
void rgb2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);
// use num_pixels of input for y channel, memset uc and vc to 128 (-> resulting in only one plane of effective info)
void raw_fill(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);
void yuv_packed2planar(MLR::PixelFormat in_format, const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);

#endif // COLORSPACE_H
