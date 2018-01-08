#pragma once

#include "video.h"
#include <Core/thread.h>

struct VideoEncoderModule: Thread{
  VAR(byteA, img)

  VideoEncoderModule();

  virtual void open();
  virtual void close();
  virtual void step();

  uint fps;
  struct VideoEncoder_libav_simple *video;
  ofstream timeTagFile;
};
