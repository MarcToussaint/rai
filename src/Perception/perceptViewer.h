/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "percept.h"
#include "../Core/thread.h"

struct PerceptViewer : Thread {
  Var<PerceptL> percepts;
  Var<rai::Configuration> kin;
  PerceptL copy;
  MeshA modelCopy;
  struct OpenGL* gl;

  PerceptViewer(Var<PerceptL>& _percepts, Var<rai::Configuration> _kin);
  ~PerceptViewer();
  void open();
  void step();
  void close();
};
