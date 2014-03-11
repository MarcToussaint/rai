#ifndef AVUTIL_H
#define AVUTIL_H

/*
 * INTERNAL HEADER, DO NOT INCLUDE FROM OUTSIDE PERCEPTION!
 */

#ifdef HAVE_LIBAV
extern "C" {
#include <libavformat/avformat.h>
}

namespace MLR {
/// call this before using any libav methods
void register_libav();

/// first tries to guess format from filename, on failure tries to use DEF_FORMAT,
/// if that does not work, HALTs
AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT);
}

#endif

#endif // AVUTIL_H
