#ifndef COLORSPACE_H
#define COLORSPACE_H

#include <stdint.h>

void bgr2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);
void rgb2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc, const unsigned int num_pixel);

#endif // COLORSPACE_H
