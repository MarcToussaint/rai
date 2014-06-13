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
namespace MLR {
enum PixelFormat {
	PIXEL_FORMAT_RGB8,	// packed r-g-b, 8 bit each
	PIXEL_FORMAT_BGR8,  // packed b-g-r, 8 bit each
	PIXEL_FORMAT_YUV444_8 // packed y-u-v, no sub-sampling, 8 bit each
};
}
struct VideoEncoder_x264_simple{
  struct sVideoEncoder_x264_simple *s;

  VideoEncoder_x264_simple(const char* filename="z.264", double fps=30, uint qp=0, MLR::PixelFormat in_format=MLR::PIXEL_FORMAT_BGR8);
  /** @deprecated Use constructor with explicit input format. Using this one allows only a choice between RGB and BGR. */
  VideoEncoder_x264_simple(const char* filename="z.264", double fps=30, uint qp=0, bool is_rgb=false);
  void addFrame(const byteA& image);
  void close();
  const MT::String& name() const;
};

struct VideoEncoder_OpenCV{
  struct sVideoEncoder_OpenCV *s;

  VideoEncoder_OpenCV(const char* filename="z.avi", uint fps=30);
  void addFrame(const byteA& rgb);
  void close();
};
