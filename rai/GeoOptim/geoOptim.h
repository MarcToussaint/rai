#pragma once

#include <Core/array.h>
#include <Geo/mesh.h>

void computeOptimalSSBox(mlr::Mesh& mesh, arr& x, mlr::Transformation& t, const arr& X, uint trials=10, int verbose=0);
