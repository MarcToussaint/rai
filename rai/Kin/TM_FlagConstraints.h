/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct TM_FlagConstraints : Feature {
  double g=1.; //gravity constant, usually 9.81
  TM_FlagConstraints() { g = rai::getParameter<double>("FlagConstraints/gravity", 1.); }
  virtual void phi(arr& y, arr& J, const WorldL& G);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const rai::KinematicWorld& K) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& Ktuple);
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("FlagConstraints"); }
};

//===========================================================================

struct TM_FlagCosts : Feature {
  TM_FlagCosts() {}
  virtual void phi(arr& y, arr& J, const WorldL& G);
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const rai::KinematicWorld& K) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& Ktuple);
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("TM_FlagCosts"); }
};
