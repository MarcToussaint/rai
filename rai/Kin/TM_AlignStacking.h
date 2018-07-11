/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_AlignStacking : Feature {
  int i;               ///< which shapes does it refer to?
  TM_AlignStacking(int iShape=-1);
  
  TM_AlignStacking(const rai::KinematicWorld& G,
                   const char* iShapeName=NULL);
                   
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 2; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
};
