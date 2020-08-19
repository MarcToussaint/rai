/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "avutil.h"
#include <iostream>
#include "../Core/util.h"

#ifdef HAVE_LIBAV

Mutex libav_open_mutex;

namespace rai {
void register_libav() {
  Lock avlock(libav_open_mutex);
  av_register_all();
  avcodec_register_all();
}

AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT) {
  AVOutputFormat* fmt = av_guess_format(nullptr, filename, nullptr);
  if(!fmt) {
    std::cerr << "Could not determine container format from filename '" << filename << "', attempting " << DEF_FORMAT;
    fmt = av_guess_format(DEF_FORMAT, nullptr, nullptr);
    if(!fmt) {
std:cerr << "Could not open container format for " << DEF_FORMAT << endl;
      return nullptr;
    }
  }
  return fmt;
}

}

#endif // HAVE_LIBAV
