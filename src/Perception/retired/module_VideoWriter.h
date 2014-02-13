#pragma once

#include "video.h"
#include <Core/module.h>

struct VideoEncoderModule: Module{
  ACCESS(byteA, img)

  VideoEncoderModule();

  virtual void open();
  virtual void close();
  virtual void step();

  uint fps;
  struct VideoEncoder_libav_simple *video;
  ofstream timeTagFile;
};
