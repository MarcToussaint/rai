/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "feature.h"

//===========================================================================

struct TM_qLimits:Feature {
  //TODO (danny) allow margin specification
  arr limits;
  
  TM_qLimits(const arr& _limits=NoArr) { if(!!_limits) limits=_limits; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("TM_qLimits"); }
};

//===========================================================================

struct LimitsConstraint:Feature {
  double margin;
  arr limits;
  LimitsConstraint(double _margin=.05):margin(_margin) {}
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual uint dim_phi(const rai::KinematicWorld& G) { return 1; }
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("LimitsConstraint"); }
  virtual Graph getSpec(const rai::KinematicWorld& K){ return Graph({{"feature", "LimitsConstraint"}}); }
};

