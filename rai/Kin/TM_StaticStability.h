/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_StaticStability : Feature {
  int i;               ///< which shapes does it refer to?
  double margin;
  
  TM_StaticStability(int iShape=-1, double _margin=.01);
  TM_StaticStability(const rai::KinematicWorld& G,
                     const char* iShapeName=NULL, double _margin=.01);
                     
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 4; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
};
