#pragma once

#include <Core/array.h>

#include "spline.h"

namespace rai{

//==============================================================================

struct KLinearFilter{
  //params
  uint K;
  double threshold;
  //memory
  arr t, x;

  KLinearFilter(uint _K=2, double _threshold=10.) : K(_K), threshold(_threshold) {}

  void update(double t_now, const arr& x_now);
  arr get_x(double t_now);

  arr get_xDot(double t);
};

//==============================================================================

} //namespace
