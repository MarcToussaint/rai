/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/* minimalistic wrapper of the standard libav (ffmpeg) example for
   encoding/decoding vids */

#pragma once

#include "pixel_format.h"
#include "../Core/array.h"
#include <memory>

/// Video encoder which creates a correct container, with metadata, of one
/// video stream in H264 format.
struct VideoEncoder_libav_simple {
  unique_ptr<struct sVideoEncoder_libav_simple> self;

  VideoEncoder_libav_simple(const char* filename="z.avi", double fps=30, uint qp=0, bool is_rgb=false);
  void addFrame(const byteA& rgb);
  void close();
};

/// Simple encoder which directly uses libx264 to create a H264 bitstream
/// Has less metadata (in particular, fps and timing seems to be off), but
/// is more CPU efficient than the above.

#if 0
struct sVideoEncoder_x264_simple;
struct VideoEncoder_x264_simple {
  std::shared_ptr<sVideoEncoder_x264_simple> s;

  VideoEncoder_x264_simple(const char* filename="z.264", double fps=30, uint qp=0, rai::PixelFormat in_format=rai::PIXEL_FORMAT_BGR8);
  /** @deprecated Use constructor with explicit input format. Using this one allows only a choice between RGB and BGR. */
  explicit VideoEncoder_x264_simple(const char* filename, double fps, uint qp, bool is_rgb);
  void addFrame(const byteA& image);
  void close();
  const rai::String& name() const;
};
#endif

struct VideoEncoder_OpenCV {
  unique_ptr<struct sVideoEncoder_OpenCV> self;

  VideoEncoder_OpenCV(const char* filename="z.avi", uint fps=30);
  void addFrame(const byteA& rgb);
  void close();
};
