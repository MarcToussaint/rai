/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TM_FixSwichedObjects:Feature {
  TM_FixSwichedObjects() {}
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const rai::KinematicWorld& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("FixSwichedObjects"); }
};
