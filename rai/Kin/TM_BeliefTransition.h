/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_BeliefTransition : Feature {
  Feature *viewError;
  TM_BeliefTransition(Feature *viewError = NULL) : viewError(viewError) {}
  ~TM_BeliefTransition() { if(viewError) delete viewError; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("BeliefTransition"); }
};
