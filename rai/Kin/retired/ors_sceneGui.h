/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// @file
/// @ingroup group_ors
/// @addtogroup group_ors
/// @{

#include "kin.h"
#include <Gui/opengl.h>

struct OrsSceneGui {
  struct sOrsSceneGui *s;
  
  OrsSceneGui(rai::KinematicWorld& ors, OpenGL *gl=NULL);
  
  void edit();
  
};

/// @}
