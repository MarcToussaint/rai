/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

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
