/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef AUDIO_H
#define AUDIO_H

#include "../Core/array.h"

class AudioWriter_libav {
 private:
  class sAudioWriter_libav* s;

 public:
  AudioWriter_libav(const char* filename);
  virtual ~AudioWriter_libav();

  /// add audio samples with each sample in a signed 16bit value in the platform's native endianness
  void writeSamples_R48000_2C_S16_NE(const byteA& samples);
};

/// PulseAudio based audio grabber
class AudioPoller_PA {
 private:
  class sAudioPoller_PA* s;

 public:
  /// Pulseaudio audio grabber using given device name
  AudioPoller_PA(const char* appname="mt_audiopoller", const char* dev=nullptr);
  virtual ~AudioPoller_PA();

  /// read from the device until buf is filled. returns true on success, false on EOF
  bool read(byteA& buf);
};

#endif // AUDIO_H
