/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_AboveBox : Feature {
  int i, j;               ///< which shapes does it refer to?
  double margin;
  
  TM_AboveBox(int iShape=-1, int jShape=-1, double _margin=.01);
  TM_AboveBox(const rai::KinematicWorld& G,
              const char* iShapeName=NULL, const char* jShapeName=NULL, double _margin=.01);
              
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 4; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
  virtual Graph getSpec(const rai::KinematicWorld& K);
};
