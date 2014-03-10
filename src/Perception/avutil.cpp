#include "avutil.h"
#include <iostream>
#include <Core/util.h>

#ifdef HAVE_LIBAV

Mutex libav_open_mutex;

namespace MLR {
void register_libav() {
    Lock avlock(libav_open_mutex);
    av_register_all();
    avcodec_register_all();
}

AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT) {
 AVOutputFormat* fmt = av_guess_format(NULL, filename, NULL);
    if(!fmt) {
        std::cerr << "Could not determine container format from filename '" << filename << "', attempting " << DEF_FORMAT;
        fmt = av_guess_format(DEF_FORMAT, NULL, NULL);
        if(!fmt) {
            std:cerr << "Could not open container format for " << DEF_FORMAT << endl;
            return NULL;
        }
    }
    return fmt;
}

}

#endif // HAVE_LIBAV
