/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMaps.h"

struct TM_GJK:Feature {
  int i, j;               ///< which shapes does it refer to?
//  rai::Vector vec1, vec2; ///< additional position or vector
  bool exact;
  bool negScalar;
  
  TM_GJK(const rai::Frame *s1, const rai::Frame *s2, bool exact, bool negScalar=false);
  TM_GJK(const rai::KinematicWorld& W, const char* s1, const char* s2, bool exact, bool negScalar=false);
  TM_GJK(const rai::KinematicWorld& W, const Graph& specs, bool exact);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& W);
  virtual uint dim_phi(const rai::KinematicWorld& G) { if(negScalar) return 1;  return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
};
