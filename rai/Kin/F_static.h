/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_netForce : Feature {
  double gravity=9.81;
  bool transOnly=false;

  F_netForce(bool _transOnly=false, bool _zeroGravity=false);

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& C){ if(transOnly) return 3;  return 6; }
};

