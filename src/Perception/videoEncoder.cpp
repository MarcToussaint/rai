#include "videoEncoder.h"
#include <Core/util.h>

#ifdef HAVE_LIBAV

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}

struct sVideoEncoder_libav_simple{
  static Mutex libav_open_mutex;
  MT::String filename;
  double fps;
  bool isOpen;

  int i, out_size, size, x, y, outbuf_size, qp;
  AVCodec *codec;
  AVCodecContext *c;
  FILE *f;
  AVFrame *picture;
  uint8_t *outbuf, *picture_buf;
  SwsContext *sws_ctx;

  int frame_count;
  double encoding_time, video_time, scale_time;

  sVideoEncoder_libav_simple() :
      fps(0), isOpen(false), i(0), out_size(0), x(0), y(0), outbuf_size(0), qp(0), codec(NULL), f(NULL), picture(NULL),
      outbuf(NULL), picture_buf(NULL), sws_ctx(NULL), frame_count(0), encoding_time(0.0), video_time(0.0), scale_time(0.0)
  {}
  sVideoEncoder_libav_simple(const char* filename, double fps, uint qp) :
      filename(filename), fps(fps), isOpen(false), i(0), out_size(0), x(0), y(0), outbuf_size(0), qp(qp),
      codec(NULL), f(NULL), picture(NULL), outbuf(NULL), picture_buf(NULL), sws_ctx(NULL), frame_count(0), encoding_time(0.0), video_time(0.0), scale_time(0.0)
  {}
  void open(uint width, uint height);
  void addFrame(const byteA& rgb);
  void close();
};

Mutex sVideoEncoder_libav_simple::libav_open_mutex;


//==============================================================================

VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char* filename, uint fps, uint qp) {
  s = new sVideoEncoder_libav_simple;
  s->filename = filename;
  s->fps = fps;
  s->qp = qp;
}

void VideoEncoder_libav_simple::addFrame(const byteA& rgb){
  if(!rgb.N) return;
  if(!s->isOpen) s->open(rgb.d1, rgb.d0); 
  s->addFrame(rgb);
}

void VideoEncoder_libav_simple::close(){ if(s->isOpen) s->close(); }

//==============================================================================

void sVideoEncoder_libav_simple::open(uint width, uint height){
  Lock avlock(libav_open_mutex);
  avcodec_register_all();

  //codec = avcodec_find_encoder(CODEC_ID_MPEG2VIDEO);
  codec = avcodec_find_encoder(CODEC_ID_H264);
  if (!codec) HALT("codec not found");

  c = avcodec_alloc_context3(codec);
  picture= avcodec_alloc_frame();

  /* put sample parameters */
  //c->bit_rate = 400000;
  /* resolution must be a multiple of two */
  c->width = width;
  c->height = height;
  /* frames per second */
  c->time_base= av_d2q(fps, INT_MAX);
  c->gop_size = 10; /* emit one intra frame every ten frames */
  c->max_b_frames=1;
  //c->pix_fmt = PIX_FMT_YUV420P;
  //c->pix_fmt = PIX_FMT_UYVY422;
  c->pix_fmt = PIX_FMT_BGR24;

  AVDictionary *opts = NULL;
  char opt_str[4];
  sprintf(opt_str,"%d", 0);
  av_dict_set(&opts, "qp", opt_str, qp);
//  av_dict_set(&opts, "preset", "superfast", 0);
  av_dict_set(&opts, "preset", "ultrafast", 0);

  /* open it */
  if (avcodec_open2(c, codec, &opts) < 0)
    HALT("Encoder failed to open");

  f = fopen(filename, "wb");
  if (!f) HALT("could not open "<< filename);

  /* alloc image and output buffer */
  size = c->width * c->height;
  picture_buf = (byte*)malloc((size * 3) / 2); /* size for YUV 420 */
  if(!picture_buf)
      HALT("Could not allocate picture buffer");
  outbuf_size = size*3; // way larger than needed, but hey, you never know what's coming
  outbuf = (byte*)malloc(outbuf_size);
  if(!outbuf)
      HALT("Could not allocate output buffer");

  picture->data[0] = picture_buf;
  picture->data[1] = picture->data[0] + size;
  picture->data[2] = picture->data[1] + size / 4;
  picture->linesize[0] = c->width;
  picture->linesize[1] = c->width / 2;
  picture->linesize[2] = c->width / 2;
  picture->pts = 0;

  //sws_ctx = sws_getContext(width, height, PIX_FMT_RGB24, width, height, c->pix_fmt, SWS_FAST_BILINEAR, NULL, NULL, NULL);
  //sws_ctx = sws_getContext(width, height, PIX_FMT_UYVY422, width, height, c->pix_fmt, SWS_FAST_BILINEAR, NULL, NULL, NULL);

  isOpen=true;
}

/*namespace {
void rgb2yuv(const char* y, const char* u, const char* v) {
    guchar *y=img->data[0], *u=img->data[1], *v=img->data[2];
                            for (x = img->width*img->height; x>0; x--) {
                                    prev_inline_rgbToYuvVis (*y, *u, *v, y, u, v);
                                    y++;
                                    u++;
                                    v++;
                            }
}
}
*/


void sVideoEncoder_libav_simple::addFrame(const byteA& rgb){
  timespec start_ts, end_ts, start_encode_ts, end_encode_ts, start_scale_ts, end_scale_ts;
  clock_gettime(CLOCK_REALTIME, &start_ts);
  int src_stride = rgb.d1*rgb.d2;
  clock_gettime(CLOCK_REALTIME, &start_scale_ts);
  //sws_scale(sws_ctx, &rgb.p, &src_stride, 0, c->height, picture->data, picture->linesize);
  sws_scale(sws_ctx, &rgb.p, &src_stride, 0, c->height, picture->data, picture->linesize);
  clock_gettime(CLOCK_REALTIME, &end_scale_ts);
  double start = start_scale_ts.tv_sec + (start_scale_ts.tv_nsec / 1e9), end = end_scale_ts.tv_sec + (end_scale_ts.tv_nsec / 1e9);
  scale_time+= (end - start);

  /* encode the image */
  //fflush(stdout);
  clock_gettime(CLOCK_REALTIME, &start_encode_ts);
  picture->data[0] = (uint8_t*)&rgb.p;
  out_size = avcodec_encode_video(c, outbuf, outbuf_size, picture);
  clock_gettime(CLOCK_REALTIME, &end_encode_ts);

  ++frame_count;
  start = start_encode_ts.tv_sec + (start_encode_ts.tv_nsec / 1e9), end = end_encode_ts.tv_sec + (end_encode_ts.tv_nsec / 1e9);
  encoding_time+= (end - start);
  picture->pts++;
//  printf("encoding frame %3d (size=%5d)\n", i, out_size);
  //fwrite(outbuf, 1, out_size, f);
  clock_gettime(CLOCK_REALTIME, &end_ts);

  start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_ts.tv_sec + (end_ts.tv_nsec / 1e9);
  video_time+= (end - start);
}

void sVideoEncoder_libav_simple::close(){
  /* get the delayed frames */
  for(; out_size; i++) {
    fflush(stdout);

    out_size = avcodec_encode_video(c, outbuf, outbuf_size, NULL);
//    printf("write frame %3d (size=%5d)\n", i, out_size);
    fwrite(outbuf, 1, out_size, f);
  }

  /* add sequence end code to have a real mpeg file */
  outbuf[0] = 0x00;
  outbuf[1] = 0x00;
  outbuf[2] = 0x01;
  outbuf[3] = 0xb7;
  fwrite(outbuf, 1, 4, f);
  fclose(f);

  avcodec_close(c);
  free(picture_buf);
  free(outbuf);
  av_free(c);
  av_free(picture);
//  printf("\n");
  cout <<" CLOSED ENCODER  file: " <<filename <<endl;
  double per_frame = (encoding_time/frame_count);
  cout << "Total encoding time: " << encoding_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (video_time/frame_count);
  cout << "Total video handling time: " << video_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
  per_frame = (scale_time/frame_count);
  cout << "Video scaling time: " << scale_time << " (" <<  per_frame << "s / " << (per_frame * 1000) << "ms per frame)" << endl;
}

#else // HAVE_LIBAV

// dummy implementation does nothing

struct sVideoEncoder_libav_simple{
};
VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char*, uint, uint){
  NICO
}
void VideoEncoder_libav_simple::addFrame(const byteA&){}
void VideoEncoder_libav_simple::close(){}

#endif // HAVE_LIBAV
