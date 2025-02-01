/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Geo/mesh.h"

void computeOptimalSSBox(rai::Mesh& mesh, arr& x, rai::Transformation& t, const arr& X, uint trials=10, int verbose=0);

void minimalConvexCore(arr& core, const arr& points, double radius, int verbose=0);

void minimalConvexCore2(arr& core, const arr& points, double radius, int verbose=0);
void minimalConvexCore3(arr& core, const arr& points, double radius, int verbose=0);
double sphereReduceConvex(rai::Mesh& M, double radius, int verbose=0);

void optimalSphere(arr& core, uint num, const arr& points, double& radius, int verbose=0);
