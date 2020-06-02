/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "../Gui/opengl.h"

struct OrsSceneGui {
  unique_ptr<struct sOrsSceneGui> self;

  OrsSceneGui(rai::Configuration& ors, OpenGL* gl=nullptr);

  void edit();

};
