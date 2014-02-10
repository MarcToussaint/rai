#include "videoEncoder.h"
#include <Core/util.h>

#include <opencv2/opencv.hpp>
using namespace cv;

namespace {
#define PREVCOL_SHIFT   10
#define PREVCOL_HALF    (1 << (PREVCOL_SHIFT - 1))
#define PREVCOL_FIX(x)  ((int) ((x) * (1<<PREVCOL_SHIFT) + 0.5))

/*
 * rgb to yuv conversion taken from icewing: http://icewing.sf.net/
 *
 * Copyright (C) 1999-2009
 * Frank Loemker, Applied Computer Science, Faculty of Technology,
 * Bielefeld University, Germany
 *
 * This file is part of iceWing, a graphical plugin shell.
 *
 * iceWing is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * iceWing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */


/*********************************************************************
  Perform RGB to YUV conversion with intervall reduction.
*********************************************************************/
static inline void rgbToYuvVis (const uint8_t rc, const uint8_t gc, const uint8_t bc,
                                            uint8_t *yc, uint8_t *uc, uint8_t *vc)

{
    int y_shift = PREVCOL_FIX(0.299*219.0/255.0) * rc +
            PREVCOL_FIX(0.587*219.0/255.0) * gc +
            PREVCOL_FIX(0.114*219.0/255.0) * bc + PREVCOL_HALF;
    int y = y_shift >> PREVCOL_SHIFT;

    *uc = ((PREVCOL_FIX(0.564*224.0/255.0) * bc -
            PREVCOL_FIX(0.564*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
    *vc = ((PREVCOL_FIX(0.713*224.0/255.0) * rc -
            PREVCOL_FIX(0.713*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
    *yc = (y_shift + (16 << PREVCOL_SHIFT)) >> PREVCOL_SHIFT;
}

}

#ifdef HAVE_X264

extern "C" {
#include <x264.h>
}

struct sVideoEncoder_x264_simple{
  MT::String filename;
  double fps;
  bool isOpen;

  int i, out_size, size, x, y, outbuf_size, qp, pts;
  FILE *f;
  x264_picture_t pic_in, pic_out;
  x264_param_t params;
  x264_nal_t* nals;
  x264_t* encoder;
  bool first;

  int frame_count;
  double encoding_time, video_time, scale_time;

  sVideoEncoder_x264_simple(const char* filename, double fps, uint qp) :
      filename(filename), fps(fps), isOpen(false), i(0), out_size(0), x(0), y(0), outbuf_size(0), qp(qp),
      f(NULL), encoder(NULL), nals(NULL), frame_count(0), encoding_time(0.0), video_time(0.0), scale_time(0.0), pts(0), first(false)
  {}

  void open(uint width, uint height);
  void addFrame(const byteA& rgb);
  void close();
};


//==============================================================================

VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char* filename, uint fps, uint qp) {
    s = new sVideoEncoder_x264_simple(filename, fps, qp);
}

void VideoEncoder_x264_simple::addFrame(const byteA& rgb){
  if(!rgb.N) return;
  if(!s->isOpen) s->open(rgb.d1, rgb.d0);
  s->addFrame(rgb);
}

void VideoEncoder_x264_simple::close(){ std::clog << "Closing VideoEncoder264" << endl; if(s->isOpen) s->close(); }

//==============================================================================

void sVideoEncoder_x264_simple::open(uint width, uint height){
    f = fopen(filename, "wb");
    if (!f) HALT("could not open "<< filename);

    pic_out.i_pts = 0;
    //pic_out.img.i_csp = X264_CSP_I444;

    x264_picture_alloc(&pic_in, X264_CSP_I444, width, height);
    std::clog << "i420 picture, planes=" << pic_in.img.i_plane ;
    for(int i = 0; i < pic_in.img.i_plane; ++i) {
        std::clog << ", stride " << i << " " << pic_in.img.i_stride[i];
    }
    std::clog << endl;

    x264_param_default_preset(&params, "ultrafast", NULL);
    params.i_csp = X264_CSP_I444;
    params.i_threads = 0;
    params.i_width = width;
    params.i_height = height;
    params.i_fps_num = fps;
    params.i_fps_den = 1;
    params.rc.i_qp_constant = 0;

    encoder = x264_encoder_open(&params);
    if (!encoder) HALT("encoder could not be opened");

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

void sVideoEncoder_x264_simple::addFrame(const byteA& rgb){
  timespec end_csp, start_ts, end_ts, start_encode_ts, end_encode_ts;

  /* encode the image */
  //fflush(stdout);
  clock_gettime(CLOCK_REALTIME, &start_ts);
  uint8_t *rc, *gc, *bc;
  // BGR
  const uint8_t *in_pixels = rgb.p;
  uint8_t *yc = pic_in.img.plane[0];
  uint8_t *uc = pic_in.img.plane[1];
  uint8_t *vc = pic_in.img.plane[2];
  const unsigned int num_pixel = rgb.d0 * rgb.d1;

#pragma omp parallel for schedule(guided, 256) num_threads(4)
  for(int i = 0; i < num_pixel; ++i) {
      const int pixel_index = i*3;
      rgbToYuvVis(in_pixels[pixel_index+2], in_pixels[pixel_index+1], in_pixels[pixel_index], yc + i, uc + i, vc + i);
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

void sVideoEncoder_x264_simple::close(){
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

#else // HAVE_x264

// dummy implementation does nothing

struct sVideoEncoder_x264_simple{
};
VideoEncoder_x264_simple::VideoEncoder_x264_simple(const char*, uint, uint){
  NICO
}
void VideoEncoder_x264_simple::addFrame(const byteA&){}
void VideoEncoder_x264_simple::close(){ }

#endif // HAVE_LIBAV
