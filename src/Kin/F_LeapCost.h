#pragma once

#include "feature.h"
#include "F_qFeatures.h"

struct CubicSplineLeapCost : Feature {
  CubicSplineLeapCost(const uintA& _selectedFrames);
  uint dim_phi2(const FrameL& F);
  void phi2(arr& y, arr& J, const FrameL& F);
};
