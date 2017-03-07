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

typedef mlr::Array<Plane*> PlaneL;
typedef mlr::Array<Plane> PlaneA;

struct CostFct_PlanePoints{
  const arr& n;
  const arr& m;
  const arr& X;
  const arr& transform;
  arr y;
  mlr::Quaternion r;

  CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform);

  double f();
  arr df_transform();

  ScalarFunction f_transform();
};



void glDrawPlanes(const PlaneA& planes);

void glDrawPlanes(void *p);
