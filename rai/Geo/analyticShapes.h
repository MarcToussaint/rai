#pragma once

#include "geo.h"

//===========================================================================
//
// analytic distance functions
//

struct DistanceFunction_Sphere : ScalarFunction {
  rai::Transformation pose; double r;
  DistanceFunction_Sphere(const rai::Transformation& _pose, double _r);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_ssBox : ScalarFunction {
  rai::Transformation pose; double size_x, size_y, size_z, r;
  DistanceFunction_ssBox(const rai::Transformation& _pose, double _size_x, double _size_y, double _size_z, double _r=0.);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Cylinder : ScalarFunction {
  rai::Transformation pose; double size_z, r;
  DistanceFunction_Cylinder(const rai::Transformation& _pose, double _size_z, double _r);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Capsule : ScalarFunction {
  rai::Transformation pose; double size_z, r;
  DistanceFunction_Capsule(const rai::Transformation& _pose, double _size_z, double _r);
  double f(arr& g, arr& H, const arr& x);
};

extern ScalarFunction DistanceFunction_SSBox;
