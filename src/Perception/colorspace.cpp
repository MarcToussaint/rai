#include "colorspace.h"

#define PREVCOL_SHIFT   10
#define PREVCOL_HALF    (1 << (PREVCOL_SHIFT - 1))
#define PREVCOL_FIX(x)  ((int) ((x) * (1<<PREVCOL_SHIFT) + 0.5))

/*
 * rgb to yuv conversion taken from icewing: http://icewing.sf.net/
 *
 * Copyright (C) 1999-2009
 * Frank Loemker, Applied Computer Science, Faculty of Technology,
 * Bielefeld University, Germany
 *
 * This file is part of iceWing, a graphical plugin shell.
 *
 * iceWing is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * iceWing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

/*********************************************************************
 Perform RGB to YUV conversion with intervall reduction.
 *********************************************************************/
static inline void rgbToYuvVis(const uint8_t rc, const uint8_t gc,
		const uint8_t bc, uint8_t *yc, uint8_t *uc, uint8_t *vc)

		{
	int y_shift = PREVCOL_FIX(0.299*219.0/255.0) * rc +
	PREVCOL_FIX(0.587*219.0/255.0) * gc +
	PREVCOL_FIX(0.114*219.0/255.0) * bc + PREVCOL_HALF;
	int y = y_shift >> PREVCOL_SHIFT;

	*uc = ((PREVCOL_FIX(0.564*224.0/255.0) * bc -
	PREVCOL_FIX(0.564*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
	*vc = ((PREVCOL_FIX(0.713*224.0/255.0) * rc -
	PREVCOL_FIX(0.713*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
	*yc = (y_shift + (16 << PREVCOL_SHIFT)) >> PREVCOL_SHIFT;
}

void bgr2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc,
		uint8_t *vc, const unsigned int num_pixel) {
#pragma omp parallel for schedule(guided, 256) num_threads(4)
	for (unsigned int i = 0; i < num_pixel; ++i) {
		const int pixel_index = i * 3;
		rgbToYuvVis(in_pixels[pixel_index + 2], in_pixels[pixel_index + 1],
				in_pixels[pixel_index], yc + i, uc + i, vc + i);
	}
}
void rgb2yuv(const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc,
		uint8_t *vc, const unsigned int num_pixel) {
#pragma omp parallel for schedule(guided, 256) num_threads(4)
	for (unsigned int i = 0; i < num_pixel; ++i) {
		const int pixel_index = i * 3;
		rgbToYuvVis(in_pixels[pixel_index], in_pixels[pixel_index + 1],
				in_pixels[pixel_index + 2], yc + i, uc + i, vc + i);
	}
}

void uyv444packed_yuv444planar(const uint8_t* const in_pixels, uint8_t* yc,
		uint8_t* uc, uint8_t *vc, const unsigned int num_pixel) {
	for (unsigned int i = 0; i < num_pixel; ++i) {
		const int pixel_index = i * 3;
		yc[i] = in_pixels[pixel_index + 1];
		uc[i] = in_pixels[pixel_index];
		vc[i] = in_pixels[pixel_index + 2];
	}
}

void yuv_packed2planar(MLR::PixelFormat in_format,
		const uint8_t* const in_pixels, uint8_t* yc, uint8_t* uc, uint8_t *vc,
		const unsigned int num_pixel) {
	switch (in_format) {
	case MLR::PIXEL_FORMAT_UYV444:
		uyv444packed_yuv444planar(in_pixels, yc, uc, vc, num_pixel);
		break;
	default:
		throw "pixel format not supported";
	}
}

