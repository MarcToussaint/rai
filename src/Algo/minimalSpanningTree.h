/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

struct DoubleEdge { uint i, j; double w; };

std::tuple<double, uintA> minimalSpanningTree(uint num_vertices, const rai::Array<DoubleEdge>& edges);

