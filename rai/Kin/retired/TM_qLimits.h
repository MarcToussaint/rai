/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct LimitsConstraint:Feature {
  double margin;
  arr limits;
  LimitsConstraint(double _margin=.05):margin(_margin) {}
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("LimitsConstraint"); }
  virtual Graph getSpec(const rai::Configuration& K) { return Graph({{"feature", "LimitsConstraint"}}); }
};

