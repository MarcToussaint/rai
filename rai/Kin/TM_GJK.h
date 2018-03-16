/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMaps.h"

struct TM_GJK:TaskMap{
  int i, j;               ///< which shapes does it refer to?
//  mlr::Vector vec1, vec2; ///< additional position or vector
  bool exact;
  bool negScalar;

  TM_GJK(const mlr::Frame *s1, const mlr::Frame *s2, bool exact, bool negScalar=false);
  TM_GJK(const mlr::KinematicWorld& W, const char* s1, const char* s2, bool exact, bool negScalar=false);
  TM_GJK(const mlr::KinematicWorld& W, const Graph& specs, bool exact);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& W, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ if(negScalar) return 1;  return 3; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G);
};
