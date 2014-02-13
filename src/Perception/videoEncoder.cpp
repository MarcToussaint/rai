#include "videoEncoder.h"
#include "colorspace.h"
#include <Core/util.h>

#ifdef HAVE_LIBAV

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
}

struct sVideoEncoder_libav_simple{
    static Mutex libav_open_mutex;
    MT::String filename;
    double fps;
    bool isOpen;

    int i, out_size, num_pixels, size, outbuf_size, qp;
    AVCodec *codec;
    AVCodecContext *c;
    FILE *f;
    AVFrame *picture;
    uint8_t *outbuf, *picture_buf;
    bool is_rgb;

    int frame_count;
    double encoding_time, video_time, csp_time;

    sVideoEncoder_libav_simple() :
        fps(0), isOpen(false), i(0), out_size(0), num_pixels(0), outbuf_size(0), qp(0), codec(NULL), f(NULL), picture(NULL),
        outbuf(NULL), picture_buf(NULL), frame_count(0), encoding_time(0.0), video_time(0.0), csp_time(0.0), is_rgb(false)
    {}
    sVideoEncoder_libav_simple(const char* filename, double fps, uint qp, bool is_rgb) :
        filename(filename), fps(fps), isOpen(false), i(0), out_size(0), num_pixels(0), outbuf_size(0), qp(qp),
        codec(NULL), f(NULL), picture(NULL), outbuf(NULL), picture_buf(NULL), frame_count(0), encoding_time(0.0), video_time(0.0), csp_time(0.0), is_rgb(is_rgb)
    {}
    void open(uint width, uint height);
    void addFrame(const byteA& rgb);
    void close();
};

Mutex sVideoEncoder_libav_simple::libav_open_mutex;


//==============================================================================

VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char* filename, uint fps, uint qp, bool is_rgb) {
    s = new sVideoEncoder_libav_simple(filename, fps, qp, is_rgb);
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

    codec = avcodec_find_encoder(CODEC_ID_H264);
    if (!codec)
        HALT("codec not found");

    c = avcodec_alloc_context3(codec);
    picture = avcodec_alloc_frame();

    /* put sample parameters */
    /* resolution must be a multiple of two */
    c->width = width;
    c->height = height;
    /* frames per second */
    c->time_base= av_d2q(fps, INT_MAX);
    c->gop_size = 10; /* emit one intra frame every ten frames */
    c->max_b_frames=1;
    c->pix_fmt = PIX_FMT_YUV444P;

    AVDictionary *opts = NULL;
    char opt_str[4];
    sprintf(opt_str,"%d", 0);
    av_dict_set(&opts, "qp", opt_str, qp);
    av_dict_set(&opts, "preset", "ultrafast", 0);

    /* open it */
    if (avcodec_open2(c, codec, &opts) < 0)
        HALT("Encoder failed to open");

    f = fopen(filename, "wb");
    if (!f) HALT("could not open "<< filename);

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
    picture->data[1] = picture->data[0] + size;
    picture->data[2] = picture->data[1] + size;
    picture->linesize[0] = c->width;
    picture->linesize[1] = c->width;
    picture->linesize[2] = c->width;
    picture->pts = 0;

    // done
    isOpen=true;
}


void sVideoEncoder_libav_simple::addFrame(const byteA& rgb){    
    timespec start_ts, end_ts, start_encode_ts, end_encode_ts, end_csp_ts;

    clock_gettime(CLOCK_REALTIME, &start_ts);
    if(!is_rgb) {
        bgr2yuv(rgb.p, picture->data[0], picture->data[1], picture->data[1], num_pixels);
    } else {
        rgb2yuv(rgb.p, picture->data[0], picture->data[1], picture->data[1], num_pixels);
    }
    clock_gettime(CLOCK_REALTIME, &end_csp_ts);
    double start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_csp_ts.tv_sec + (end_csp_ts.tv_nsec / 1e9);
    csp_time+= (end - start);

    /* encode the image */
    clock_gettime(CLOCK_REALTIME, &start_encode_ts);
    picture->data[0] = (uint8_t*)&rgb.p;
    out_size = avcodec_encode_video(c, outbuf, outbuf_size, picture);
    clock_gettime(CLOCK_REALTIME, &end_encode_ts);

    ++frame_count;
    start = start_encode_ts.tv_sec + (start_encode_ts.tv_nsec / 1e9), end = end_encode_ts.tv_sec + (end_encode_ts.tv_nsec / 1e9);
    encoding_time+= (end - start);
    picture->pts++;

    clock_gettime(CLOCK_REALTIME, &end_ts);

    start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_ts.tv_sec + (end_ts.tv_nsec / 1e9);
    video_time+= (end - start);
}

void sVideoEncoder_libav_simple::close(){
    /* get the delayed frames */
    for(; out_size; i++) {
        fflush(stdout);

        out_size = avcodec_encode_video(c, outbuf, outbuf_size, NULL);
        fwrite(outbuf, 1, out_size, f);
    }

    fclose(f);

    avcodec_close(c);
    free(picture_buf);
    free(outbuf);
    av_free(c);
    av_free(picture);

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

struct sVideoEncoder_libav_simple{
};
VideoEncoder_libav_simple::VideoEncoder_libav_simple(const char*, uint, uint, bool){
    NICO
}
void VideoEncoder_libav_simple::addFrame(const byteA&){}
void VideoEncoder_libav_simple::close(){}

#endif // HAVE_LIBAV
