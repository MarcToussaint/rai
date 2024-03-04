#pragma once

#include "../Core/array.h"

struct DoubleEdge{ uint i,j; double w; };

std::tuple<double, uintA> minimalSpanningTree(uint num_vertices, const rai::Array<DoubleEdge>& edges);

