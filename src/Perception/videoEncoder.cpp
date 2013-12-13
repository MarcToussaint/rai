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
  uint fps;
  bool isOpen;

  AVCodec *codec;
  AVCodecContext *c;
  int i, out_size, size, x, y, outbuf_size, qp;
  FILE *f;
  AVFrame *picture;
  uint8_t *outbuf, *picture_buf;
  SwsContext *sws_ctx;

  sVideoEncoder_libav_simple():isOpen(false){}
  void open(uint width, uint height);
  void addFrame(const byteA& rgb);
  void close();
};

Mutex sVideoEncoder_libav_simple::libav_open_mutex;


//==============================================================================

VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char* filename, uint fps, uint qp){
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

void VideoEncoder_libav_simple::close(){ s->close(); }

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
  c->time_base= (AVRational){1, (int)fps};
  c->gop_size = 10; /* emit one intra frame every ten frames */
  c->max_b_frames=1;
  c->pix_fmt = PIX_FMT_YUV420P;

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

  sws_ctx = sws_getContext(width, height, PIX_FMT_RGB24, width, height, c->pix_fmt, SWS_BILINEAR, NULL, NULL, NULL);

  isOpen=true;
}

void sVideoEncoder_libav_simple::addFrame(const byteA& rgb){
  int src_stride = rgb.d1*rgb.d2;
  sws_scale(sws_ctx, &rgb.p, &src_stride, 0, c->height, picture->data, picture->linesize);

  /* encode the image */
  fflush(stdout);
  out_size = avcodec_encode_video(c, outbuf, outbuf_size, picture);
  picture->pts++;
//  printf("encoding frame %3d (size=%5d)\n", i, out_size);
  fwrite(outbuf, 1, out_size, f);
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
