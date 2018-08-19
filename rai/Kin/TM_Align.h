/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_Align : Feature {
  int i, j;               ///< which shapes does it refer to?
  
  TM_Align(const rai::KinematicWorld& G, const char* iName=NULL, const char* jName=NULL);
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
};

