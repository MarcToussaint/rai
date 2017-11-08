/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/* image conversion */

#ifndef CONV_H
#define CONV_H

#include <climits>
//#include "image.h"
//#include "imutil.h"
#include "misc.h"

namespace felzenszwalb {
template<class T> class image;
typedef unsigned char uchar;

image<uchar> *imageRGBtoGRAY(image<rgb> *input);
image<rgb> *imageGRAYtoRGB(image<uchar> *input);
image<float> *imageUCHARtoFLOAT(image<uchar> *input);
image<float> *imageINTtoFLOAT(image<int> *input);
image<uchar> *imageFLOATtoUCHAR(image<float> *input, float min, float max);
image<uchar> *imageFLOATtoUCHAR(image<float> *input);
image<long> *imageUCHARtoLONG(image<uchar> *input);
image<uchar> *imageLONGtoUCHAR(image<long> *input, long min, long max);
image<uchar> *imageLONGtoUCHAR(image<long> *input);
image<uchar> *imageSHORTtoUCHAR(image<short> *input, short min, short max);
image<uchar> *imageSHORTtoUCHAR(image<short> *input);

}  // namespace felzenszwalb

#endif
