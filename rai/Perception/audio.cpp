/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "audio.h"
#include <iostream>
using namespace std;

#include "../Core/util.h"

#ifdef HAVE_LIBAV
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}
#include "avutil.h"
using namespace rai;

#define DEFAULT_CONTAINER "wav"

class sAudioWriter_libav {
 private:
  AVCodec* codec;
  AVFormatContext* oc;
  AVStream* s;

 public:
  sAudioWriter_libav(const char* filename) : codec(nullptr), oc(nullptr), s(nullptr) {
    register_libav();

    oc = avformat_alloc_context();
    if(!oc) {
      HALT("Could not allocate format context");
    }
    oc->oformat = mt_guess_format(filename, DEFAULT_CONTAINER);
    if(!oc->oformat) {
      HALT("Could not guess format for " << filename);
    }
    oc->audio_codec_id = CODEC_ID_PCM_S16LE;
    codec = avcodec_find_encoder(oc->audio_codec_id);
    if(!codec)
      HALT("Audion codec not found");

    snprintf(oc->filename, sizeof(oc->filename), "%s", filename);
    s= avformat_new_stream(oc, codec);
    if(!s) {
      HALT("Could not allocate stream structure");
    }
    self->codec->sample_fmt = AV_SAMPLE_FMT_S16;
    self->codec->sample_rate = 48000;
    self->codec->channels = 2;

    // some formats want stream headers to be separate
    if(oc->oformat->flags & AVFMT_GLOBALHEADER)
      oc->flags |= CODEC_FLAG_GLOBAL_HEADER;

    /* open it */
    if(avcodec_open2(self->codec, codec, nullptr) < 0)
      HALT("Encoder failed to open");

    if(avio_open(&(oc->pb), filename, AVIO_FLAG_WRITE) < 0) {
      HALT("Could not open " << filename);
    }
    avformat_write_header(oc, nullptr);
  }
  ~sAudioWriter_libav() {
    avcodec_close(self->codec);
    avformat_free_context(oc);
  }

  void write(const byteA& audio_samples) {
    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = nullptr;

    AVFrame frame;
    frame.data[0] = audio_samples.p;
    frame.linesize[0] = audio_samples.d0;
    frame.nb_samples = audio_samples.d0 / 4; // 2 channels, 2 bytes per sample
    frame.pts = AV_NOPTS_VALUE;

    // note: allocates data, so not most efficient, but should do
    int got_packet, ret;

    if((ret = avcodec_encode_audio2(self->codec, &pkt, &frame, &got_packet)) != 0) {
      HALT("Could not encode audio frame: " << ret);
      return;
    }
    if(got_packet) {
      if((ret = av_write_frame(oc, &pkt)) != 0) {
        HALT("Error while writing audio frame: " << ret);
      }
    }
    av_free_packet(&pkt);
  }
};
#else
class sAudioRecorder_libav {};
#endif

AudioWriter_libav::AudioWriter_libav(const char* filename)
#ifdef HAVE_LIBAV
  : s(new sAudioWriter_libav(filename))
#endif
{
}
AudioWriter_libav::~AudioWriter_libav() {
#if HAVE_LIBAV
  delete(s);
#endif
}

void AudioWriter_libav::writeSamples_R48000_2C_S16_NE(const byteA& samples) {
#ifdef HAVE_LIBAV
  self->write(samples);
#else
  RAI_MSG("writeSamples not available, because LIBAV is missing");
#endif
}

// PULSEAUDIO-based audio grabbing implementation

#ifdef HAVE_PULSEAUDIO
#include <pulse/simple.h>
#include <pulse/error.h>

class sAudioPoller_PA {
 private:
  pa_simple* pa;
 public:
  sAudioPoller_PA(const char* appname, const char* dev) {
    pa_sample_spec ss;
    ss.format = PA_SAMPLE_S16NE;
    ss.rate = 48000;
    ss.channels = 2;

    int error;
    if(!(pa = pa_simple_new(nullptr, appname, PA_STREAM_RECORD, dev, "record", &ss, nullptr, nullptr, &error))) {
      HALT(": pa_simple_new() failed: " << pa_strerror(error));
    }
  }
  ~sAudioPoller_PA() {
    pa_simple_free(pa);
  }
  int read(byteA& buf) {
    int ret, error;
    if((ret = pa_simple_read(pa, buf.p, buf.d0, &error)) < 0) {
      RAI_MSG(pa_strerror(error));
      return -1;
    }
    return ret;
  }
};

#endif

AudioPoller_PA::AudioPoller_PA(const char* appname, const char* dev)
#ifdef HAVE_PULSEAUDIO
  : s(new sAudioPoller_PA(appname, dev))
#endif
{
}

AudioPoller_PA::~AudioPoller_PA() {
}

bool AudioPoller_PA::read(byteA& buf) {
#ifdef HAVE_PULSEAUDIO
  return(self->read(buf) >= 0);
#else
  RAI_MSG("AudioPoller_PA::read not available, libpulse missing");
  return false;
#endif
}

