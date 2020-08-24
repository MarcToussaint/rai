/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_netForce : Feature {
  int i;               ///< which shapes does it refer to?
  double gravity=9.81;
  bool transOnly=false;

  F_netForce(int iShape, bool _transOnly=false, bool _zeroGravity=false);
  F_netForce(const rai::Configuration& K, const char* iShapeName, bool _transOnly=false, bool _zeroGravity=false)
    : F_netForce(initIdArg(K, iShapeName), _transOnly, _zeroGravity) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& C);
  virtual uint dim_phi(const rai::Configuration& C);

  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("F_netForce-" <<K.frames(i)->name); }
};

