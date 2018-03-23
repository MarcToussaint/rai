#pragma once

#include <Core/array.h>
#include <Geo/geo.h>

struct Plane{
  arr mean, normal;
  arr inlierPoints;
  arr borderPoints;
  uintA borderTris;
  int label;
};

typedef rai::Array<Plane*> PlaneL;
typedef rai::Array<Plane> PlaneA;

struct CostFct_PlanePoints{
  const arr& n;
  const arr& m;
  const arr& X;
  const arr& transform;
  arr y;
  rai::Quaternion r;

  CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform);

  double f();
  arr df_transform();

  ScalarFunction f_transform();
};



void glDrawPlanes(const PlaneA& planes);

void glDrawPlanes(void *p);
