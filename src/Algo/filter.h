#pragma once

#include <Core/array.h>

//===========================================================================

struct KLinearFilter{
  uint K;
  double threshold;

  arr t, x;

  //regression
  bool reg_is_good = false;
  double t_mean;
  arr x_mean, beta;

  KLinearFilter(uint _K=2, double _threshold=10.) : K(_K), threshold(_threshold) {}

  void update(double t_now, const arr& x_now);

  arr get_x(double t_now);

  arr get_xDot(double t);
};
