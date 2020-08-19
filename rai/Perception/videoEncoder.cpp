/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "videoEncoder.h"

#ifdef HAVE_LIBAV

#include "colorspace.h"
#include "../Core/util.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libavformat/avformat.h>
}
#include "avutil.h"

namespace {
const char* DEFAULT_CONTAINER = "avi";
}
using namespace rai;

struct sVideoEncoder_libav_simple {
  rai::String filename;
  double fps;
  bool isOpen;

  int i, out_size, num_pixels, size, outbuf_size, qp;
  AVCodec* codec;
  AVFormatContext* container_context;
  AVStream* video_stream;
  AVFrame* picture;
  uint8_t* outbuf, *picture_buf;
  bool is_rgb;

  int frame_count;
  double encoding_time, video_time, csp_time;

  sVideoEncoder_libav_simple() :
    fps(0), isOpen(false), i(0), out_size(0), num_pixels(0), outbuf_size(0), qp(0), codec(nullptr), picture(nullptr),
    outbuf(nullptr), picture_buf(nullptr), frame_count(0), encoding_time(0.0), video_time(0.0), csp_time(0.0), is_rgb(false)
  {}
  sVideoEncoder_libav_simple(const char* filename, double fps, uint qp, bool is_rgb) :
    filename(filename), fps(fps), isOpen(false), i(0), out_size(0), num_pixels(0), outbuf_size(0), qp(qp),
    codec(nullptr), picture(nullptr), outbuf(nullptr), picture_buf(nullptr), frame_count(0), encoding_time(0.0), video_time(0.0), csp_time(0.0), is_rgb(is_rgb)
  {}
  void open(uint width, uint height);
  void addFrame(const byteA& rgb);
  void close();
 private:
  void writeFrame();
};

//==============================================================================

VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char* filename, double fps, uint qp, bool is_rgb) : s(new sVideoEncoder_libav_simple(filename, fps, qp, is_rgb)) {
  std::clog << "Opening " << filename << " with fps=" << fps << ", qp=" << qp << ", " << (is_rgb ? "rgb " : "bgr") << " format" << endl;
}

void VideoEncoder_libav_simple::addFrame(const byteA& rgb) {
  if(!rgb.N) return;
  if(!self->isOpen) self->open(rgb.d1, rgb.d0);
  self->addFrame(rgb);
}

void VideoEncoder_libav_simple::close() { if(self->isOpen) self->close(); }

//==============================================================================

void sVideoEncoder_libav_simple::open(uint width, uint height) {
  register_libav();

  // prepare container context
  container_context = avformat_alloc_context();
  if(!container_context) {
    HALT("Allocation error for format context");
  }
  container_context->oformat = mt_guess_format(filename, DEFAULT_CONTAINER);
  if(!container_context->oformat) {
    HALT("Could not guess format for " << filename);
  }
  container_context->video_codec_id = CODEC_ID_H264;
  codec = avcodec_find_encoder(container_context->video_codec_id);
  if(!codec)
    HALT("codec not found");

  snprintf(container_context->filename, sizeof(container_context->filename), "%s", filename.p);
  video_stream = avformat_new_stream(container_context, codec);
  if(!video_stream) {
    HALT("Could not allocate video stream in container");
  }

  picture = avcodec_alloc_frame();

  /* put sample parameters */
  /* resolution must be a multiple of two */
  video_stream->codec->width = width;
  video_stream->codec->height = height;
  /* frames per second */
  video_stream->codec->time_base = av_d2q(1./fps, INT_MAX);
  video_stream->codec->gop_size = 10; /* emit one intra frame every ten frames */
  video_stream->codec->max_b_frames=1;
  video_stream->codec->pix_fmt = PIX_FMT_YUV444P;
  // apparently mp4 needs this
  if(!strcmp(container_context->oformat->name, "mp4"))
    video_stream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

  AVDictionary* opts = nullptr;
  char opt_str[4];
  sprintf(opt_str, "%d", 0);
  av_dict_set(&opts, "qp", opt_str, qp);
  av_dict_set(&opts, "preset", "ultrafast", 0);

  /* open it */
  if(avcodec_open2(video_stream->codec, codec, &opts) < 0)
    HALT("Encoder failed to open");

  // 1 should be URL_WRONLY
  HALT("URL_WRONLY wasn't defined. So I changed it to 1. Correct this!");
  if(avio_open(&container_context->pb, filename, 1) < 0) {
    HALT("Could not open " << filename);
  }
  avformat_write_header(container_context, &opts);

  /* alloc image and output buffer */
  num_pixels = width * height;
  size = num_pixels * 3; // 3 planes
  picture_buf = (byte*)malloc(size);
  if(!picture_buf)
    HALT("Could not allocate picture buffer");
  outbuf_size = size;
  outbuf = (byte*)malloc(outbuf_size);
  if(!outbuf)
    HALT("Could not allocate output buffer");

  // configure for three planes, not subsampled
  picture->data[0] = picture_buf;
  picture->data[1] = picture->data[0] + num_pixels;
  picture->data[2] = picture->data[1] + num_pixels;
  picture->linesize[0] = width; //num_pixels;
  picture->linesize[1] = width; //num_pixels;
  picture->linesize[2] = width; //num_pixels;
  picture->pts = 0;

  // done
  isOpen=true;
}

void sVideoEncoder_libav_simple::writeFrame() {
  if(out_size != 0) {
    // create a packet for container to write
    AVPacket pkt;
    av_init_packet(&pkt);

    pkt.pts= video_stream->codec->coded_frame->pts;
    if(video_stream->codec->coded_frame->key_frame)
      pkt.flags |= AV_PKT_FLAG_KEY;
    pkt.stream_index = video_stream->index;
    pkt.data = outbuf;
    pkt.size = out_size;

    av_write_frame(container_context, &pkt); // CHECK return code
    frame_count++;
    picture->pts++;
  }
}

void sVideoEncoder_libav_simple::addFrame(const byteA& rgb) {
  timespec start_ts, end_ts, start_encode_ts, end_encode_ts, end_csp_ts;

  clock_gettime(CLOCK_REALTIME, &start_ts);
  if(!is_rgb) {
    bgr2yuv(rgb.p, picture->data[0], picture->data[1], picture->data[2], num_pixels);
  } else {
    rgb2yuv(rgb.p, picture->data[0], picture->data[1], picture->data[2], num_pixels);
  }
  clock_gettime(CLOCK_REALTIME, &end_csp_ts);
  double start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_csp_ts.tv_sec + (end_csp_ts.tv_nsec / 1e9);
  csp_time+= (end - start);

  /* encode the image */
  clock_gettime(CLOCK_REALTIME, &start_encode_ts);
  out_size = avcodec_encode_video(video_stream->codec, outbuf, outbuf_size, picture);
  clock_gettime(CLOCK_REALTIME, &end_encode_ts);

  writeFrame();
  start = start_encode_ts.tv_sec + (start_encode_ts.tv_nsec / 1e9), end = end_encode_ts.tv_sec + (end_encode_ts.tv_nsec / 1e9);
  encoding_time+= (end - start);

  clock_gettime(CLOCK_REALTIME, &end_ts);

  start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_ts.tv_sec + (end_ts.tv_nsec / 1e9);
  video_time+= (end - start);
}

void sVideoEncoder_libav_simple::close() {
  /* get the delayed frames */
  do {
    out_size = avcodec_encode_video(video_stream->codec, outbuf, outbuf_size, nullptr);
    writeFrame();
  } while(out_size > 0);

  avcodec_close(video_stream->codec);
  free(picture_buf);
  free(outbuf);
  av_free(video_stream->codec);
  av_free(picture);

  av_write_trailer(container_context);
  /* free the streams */
  for(i = 0; i < container_context->nb_streams; i++) {
    av_freep(&container_context->streams[i]);
  }
  avio_close(container_context->pb);
  av_free(container_context);

  cout <<" CLOSED ENCODER  file: " <<filename <<endl;
  double per_frame = (encoding_time/frame_count);
  cout << "Total encoding time: " << encoding_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (video_time/frame_count);
  cout << "Total video handling time: " << video_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (csp_time/frame_count);
  cout << "Video scaling time: " << csp_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
}

#else // HAVE_LIBAV

// dummy implementation does nothing

struct sVideoEncoder_libav_simple {};
struct sVideoEncoder {};

VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char*, double, uint, bool) {
  NICO
}
void VideoEncoder_libav_simple::addFrame(const byteA&) {}
void VideoEncoder_libav_simple::close() {}

#endif // HAVE_LIBAV
