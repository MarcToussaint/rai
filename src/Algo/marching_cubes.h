#pragma once

#include "../Core/array.h"

namespace rai {

std::tuple<arr, uintA> marching_cubes(const floatA& grid_values, const arr& size);

}
