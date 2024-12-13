/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"
#include "F_qFeatures.h"

struct CubicSplineLeapCost : Feature {
  CubicSplineLeapCost(const uintA& _selectedFrames);
  uint dim_phi(const FrameL& F);
  void phi2(arr& y, arr& J, const FrameL& F);
};
