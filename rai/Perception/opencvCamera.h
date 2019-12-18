/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Core/array.h>

struct OpencvCamera : Thread {
  ptr<struct sOpencvCamera> s;
  Var<byteA> rgb;
  std::map<int, double> properties; bool set(int prop, double status);
  OpencvCamera(const Var<byteA>& _rgb);
  ~OpencvCamera();
  void open();
  void step();
  void close();
};
