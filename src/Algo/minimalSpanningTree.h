#pragma once

#include "../Core/array.h"

struct DoubleEdge{ uint i,j; double w; };

uintA minimalSpanningTree(uint num_vertices, rai::Array<DoubleEdge>& edges);

