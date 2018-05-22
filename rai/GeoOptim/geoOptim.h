/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include <Geo/mesh.h>

void computeOptimalSSBox(rai::Mesh& mesh, arr& x, rai::Transformation& t, const arr& X, uint trials=10, int verbose=0);
