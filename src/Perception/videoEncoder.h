/* minimalistic wrapper of the standard libav (ffmpeg) example for
   encoding/decoding vids */

#pragma once

#include <Core/array.h>

struct VideoEncoder_libav_simple{
  struct sVideoEncoder_libav_simple *s;

  VideoEncoder_libav_simple(const char* filename="z.avi", uint fps=30, uint qp=0);
  void addFrame(const byteA& rgb);
  void close();
};

struct VideoEncoder_x264_simple{
  struct sVideoEncoder_x264_simple *s;

  VideoEncoder_x264_simple(const char* filename="z.avi", uint fps=30, uint qp=0);
  void addFrame(const byteA& rgb);
  void close();
};

struct VideoEncoder_OpenCV{
  struct sVideoEncoder_OpenCV *s;

  VideoEncoder_OpenCV(const char* filename="z.avi", uint fps=30);
  void addFrame(const byteA& rgb);
  void close();
};
