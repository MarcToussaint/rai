/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef AVUTIL_H
#define AVUTIL_H

/*
 * INTERNAL HEADER, DO NOT INCLUDE FROM OUTSIDE PERCEPTION!
 */

#ifdef HAVE_LIBAV
extern "C" {
#include <libavformat/avformat.h>
}

namespace rai {
/// call this before using any libav methods
void register_libav();

/// first tries to guess format from filename, on failure tries to use DEF_FORMAT,
/// if that does not work, HALTs
AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT);
}

#endif

#endif // AVUTIL_H
