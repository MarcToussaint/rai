/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct TM_BeliefTransition : Feature {
  Feature* viewError;
  TM_BeliefTransition(Feature* viewError = nullptr) : viewError(viewError) {}
  ~TM_BeliefTransition() { if(viewError) delete viewError; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& G);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("BeliefTransition"); }
};
