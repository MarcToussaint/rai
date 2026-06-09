/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include "../Core/array.h"

struct OpencvCamera : Thread {
  unique_ptr<struct sOpencvCamera> self;

  int cameraID;
  bool flip_bgr=false;
  Var<byteA> image;

  OpencvCamera(const char *_name="default", int _cameraID=0);
  ~OpencvCamera();

  void open();
  void step();
  void close();
};
