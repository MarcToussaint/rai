/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include "percept.h"

struct PerceptViewer : Thread {
  Var<PerceptL> percepts;
  Var<rai::KinematicWorld> kin;
  PerceptL copy;
  MeshA modelCopy;
  struct OpenGL *gl;
  
  PerceptViewer(Var<PerceptL>& _percepts, Var<rai::KinematicWorld> _kin);
  ~PerceptViewer();
  void open();
  void step();
  void close();
};
