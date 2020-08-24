/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "videoEncoder.h"
#include "colorspace.h"
#include "../Core/util.h"

#ifdef X264_INSTALLED

extern "C" {
#include <x264.h>
}

struct sVideoEncoder_x264_simple {
  rai::String filename;
  double fps;
  bool isOpen;

  int i, out_size, size, x, y, outbuf_size, qp, pts;
  FILE* f;
  x264_picture_t pic_in, pic_out;
  x264_param_t params;
  x264_t* encoder;
  x264_nal_t* nals;
  bool first;
  rai::PixelFormat in_format;

  int frame_count;
  double encoding_time, video_time, scale_time;

  sVideoEncoder_x264_simple(const char* filename, double fps, uint qp, rai::PixelFormat in_format) :
    filename(filename), fps(fps), isOpen(false), i(0), out_size(0), x(0), y(0), outbuf_size(0), qp(qp), pts(0),
    f(nullptr), encoder(nullptr), nals(nullptr), first(false), in_format(in_format),
    frame_count(0), encoding_time(0.0), video_time(0.0), scale_time(0.0) {
    std::clog << "Encoder for " << filename << " expects pixel format " << in_format << std::endl;
  }
  ~sVideoEncoder_x264_simple() {
    close();
  }

  void open(uint width, uint height);
  void addFrame(const byteA& image);
  void close();
};

//==============================================================================

VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char* filename, double fps, uint qp, bool is_rgb) :
  s(new sVideoEncoder_x264_simple(filename, fps, qp, is_rgb ? rai::PIXEL_FORMAT_RGB8 : rai::PIXEL_FORMAT_BGR8)) {
}
VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char* filename, double fps, uint qp, rai::PixelFormat in_format) :
  s(new sVideoEncoder_x264_simple(filename, fps, qp, in_format)) {
}

void VideoEncoder_x264_simple::addFrame(const byteA& image) {
  if(!image.N) return;
  if(!self->isOpen) self->open(image.d1, image.d0);
  self->addFrame(image);
}
const rai::String& VideoEncoder_x264_simple::name() const {
  return self->filename;
}

void VideoEncoder_x264_simple::close() {
  std::clog << "Closing VideoEncoder264" << endl;
  if(self->isOpen)
    self->close();
}

//==============================================================================

void sVideoEncoder_x264_simple::open(uint width, uint height) {
  f = fopen(filename, "wb");
  if(!f) HALT("could not open "<< filename);

  pic_out.i_pts = 0;
  //pic_out.img.i_csp = X264_CSP_I444;

  int cspace = X264_CSP_NONE;
  switch(in_format) {
    default:
      cspace = X264_CSP_I444;
      break;
  }
  // could save a bit by using I420 here, but would make filling and converting code more complex

  x264_picture_alloc(&pic_in, cspace, width, height);
  std::clog << "i420 picture, planes=" << pic_in.img.i_plane ;
  for(int i = 0; i < pic_in.img.i_plane; ++i) {
    std::clog << ", stride " << i << " " << pic_in.img.i_stride[i];
  }
  std::clog << endl;

  x264_param_default_preset(&params, "ultrafast", nullptr);
  params.i_csp = cspace;
  params.i_threads = 0;
  params.i_width = width;
  params.i_height = height;
  params.i_fps_num = fps;
  params.i_fps_den = 1;
  params.rc.i_qp_constant = 0;

  encoder = x264_encoder_open(&params);
  if(!encoder) HALT("encoder could not be opened");

  // write header
  int nheader = 0;
  if(x264_encoder_headers(encoder, &nals, &nheader) < 0) {
    HALT("cannot create headers");
  }
  int header_size = nals[0].i_payload + nals[1].i_payload +nals[2].i_payload;
  if(!fwrite(nals[0].p_payload, header_size, 1, f)) {
    HALT("Cannot write headers");
  }

  isOpen=true;
}

void sVideoEncoder_x264_simple::addFrame(const byteA& image) {
  timespec end_csp, start_ts, end_ts, start_encode_ts, end_encode_ts;

  /* encode the image */
  //fflush(stdout);
  clock_gettime(CLOCK_REALTIME, &start_ts);
  const unsigned int num_pixel = image.d0 * image.d1;

  switch(in_format) {
    case rai::PIXEL_FORMAT_BGR8:
      bgr2yuv(image.p, pic_in.img.plane[0], pic_in.img.plane[1], pic_in.img.plane[2], num_pixel);
      break;
    case rai::PIXEL_FORMAT_RGB8:
      rgb2yuv(image.p, pic_in.img.plane[0], pic_in.img.plane[1], pic_in.img.plane[2], num_pixel);
      break;
    case rai::PIXEL_FORMAT_UYV444:
      yuv_packed2planar(in_format, image.p, pic_in.img.plane[0], pic_in.img.plane[1], pic_in.img.plane[2], num_pixel);
      break;
    case rai::PIXEL_FORMAT_RAW8:
      raw_fill(image.p, pic_in.img.plane[0], pic_in.img.plane[1], pic_in.img.plane[2], num_pixel);
      break;
    default:
      throw "input pixel format not supported, yet";
  }

  clock_gettime(CLOCK_REALTIME, &end_csp);
  double start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_csp.tv_sec + (end_csp.tv_nsec / 1e9);
  scale_time+=(end-start);

  int num_nals;
  pic_in.i_pts++; // = pic_out.i_pts++;
  clock_gettime(CLOCK_REALTIME, &start_encode_ts);
  out_size = x264_encoder_encode(encoder, &nals, &num_nals, &pic_in, &pic_out);
  if(out_size < 0) {
    HALT("Could not encode input image " << out_size);
  }
  clock_gettime(CLOCK_REALTIME, &end_encode_ts);

  ++frame_count;
  start = start_encode_ts.tv_sec + (start_encode_ts.tv_nsec / 1e9), end = end_encode_ts.tv_sec + (end_encode_ts.tv_nsec / 1e9);
  encoding_time+= (end - start);

  if(out_size > 0) {
    fwrite(nals[0].p_payload, out_size, 1, f);
  }
  clock_gettime(CLOCK_REALTIME, &end_ts);

  start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_ts.tv_sec + (end_ts.tv_nsec / 1e9);
  video_time+= (end - start);
}

void sVideoEncoder_x264_simple::close() {
  fclose(f);

  x264_encoder_close(encoder);
  //FIXME free image data

  cout <<" CLOSED ENCODER  file: " <<filename <<endl;
  double per_frame = (encoding_time/frame_count);
  cout << "Total encoding time: " << encoding_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (video_time/frame_count);
  cout << "Total video handling time: " << video_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (scale_time/frame_count);
  cout << "Video scaling time: " << scale_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
}

#else // X264_INSTALLED

// dummy implementation does nothing

struct sVideoEncoder_x264_simple {
};
VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char*, double, uint, bool) {
  NICO
}
VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char*, double, uint, rai::PixelFormat) {
  NICO
}
void VideoEncoder_x264_simple::addFrame(const byteA&) {}
void VideoEncoder_x264_simple::close() { }

#endif // X264_INSTALLED
