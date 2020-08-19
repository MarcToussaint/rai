/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"

struct RosCom_Spinner : Thread {
  bool useRos;
  RosCom_Spinner(const char* nodeName="RAInode");
  ~RosCom_Spinner();
  void open() {}
  void step();
  void close() {}
};

