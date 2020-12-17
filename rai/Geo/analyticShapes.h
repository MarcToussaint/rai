#pragma once

#include "geo.h"

//===========================================================================
//
// analytic distance functions
//

struct DistanceFunction_Sphere : ScalarFunction {
  rai::Transformation t; double r;
  DistanceFunction_Sphere(const rai::Transformation& _t, double _r);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_ssBox : ScalarFunction {
  rai::Transformation t; double dx, dy, dz, r;
  DistanceFunction_ssBox(const rai::Transformation& _t, double _dx, double _dy, double _dz, double _r=0.);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Cylinder : ScalarFunction {
  rai::Transformation t; double r, dz;
  DistanceFunction_Cylinder(const rai::Transformation& _t, double _r, double _dz);
  double f(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Capsule : ScalarFunction {
  rai::Transformation t; double r, dz;
  DistanceFunction_Capsule(const rai::Transformation& _t, double _r, double _dz);
  double f(arr& g, arr& H, const arr& x);
};

extern ScalarFunction DistanceFunction_SSBox;
