/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

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
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const rai::Configuration& G) { HALT("can only be of order 1"); }
  virtual uint dim_phi(const ConfigurationL& G);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("FixSwichedObjects"); }
};
