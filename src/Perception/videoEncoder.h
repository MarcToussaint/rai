/* minimalistic wrapper of the standard libav (ffmpeg) example for
   encoding/decoding vids */

#pragma once

#include <Core/array.h>

/// Video encoder which creates a correct container, with metadata, of one
/// video stream in H264 format.
struct VideoEncoder_libav_simple{
  struct sVideoEncoder_libav_simple *s;

  VideoEncoder_libav_simple(const char* filename="z.avi", double fps=30, uint qp=0, bool is_rgb=false);
  void addFrame(const byteA& rgb);
  void close();
};

/// Simple encoder which directly uses libx264 to create a H264 bitstream
/// Has less metadata (in particular, fps and timing seems to be off), but
/// is more CPU efficient than the above.
struct VideoEncoder_x264_simple{
  struct sVideoEncoder_x264_simple *s;

  VideoEncoder_x264_simple(const char* filename="z.264", double fps=30, uint qp=0, bool is_rgb=false);
  void addFrame(const byteA& rgb);
  void close();
};

struct VideoEncoder_OpenCV{
  struct sVideoEncoder_OpenCV *s;

  VideoEncoder_OpenCV(const char* filename="z.avi", uint fps=30);
  void addFrame(const byteA& rgb);
  void close();
};
