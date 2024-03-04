/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/*
 * pixel_format.hpp
 *
 *  Created on: Jun 13, 2014
 *      Author: ingo
 */

#ifndef PIXEL_FORMAT_HPP_
#define PIXEL_FORMAT_HPP_

/* for the yuv formats, also see http://www.ptgrey.com/support/kb/index.asp?a=4&q=313 */

namespace rai {
enum PixelFormat {
  PIXEL_FORMAT_RAW8,    // raw sensor-values, format depends on camera type
  PIXEL_FORMAT_RGB8,    // packed r-g-b, 8 bit each
  PIXEL_FORMAT_BGR8,    // packed b-g-r, 8 bit each
  PIXEL_FORMAT_YUV444_8,  // packed y-u-v, no sub-sampling, 8 bit each
  PIXEL_FORMAT_UYV444,  // packed u-y-v, no sub-sampling. (-> U0 Y0 V0 U1 Y1 V1 U2 Y2 V2 ...)
  PIXEL_FORMAT_UYV422   // packed u-y-v, 2x2 UV sub-sampling (-> U0 Y0 V0 Y1 U2 Y2 V2 Y3 U4 Y4 V4...)
};
}

#endif /* PIXEL_FORMAT_HPP_ */
