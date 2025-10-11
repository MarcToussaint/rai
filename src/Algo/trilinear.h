#pragma once

#include "../Core/array.h"

namespace rai {

template<class T>
rai::Array<T> trilinear_interpolate(const arr& pts, const rai::Array<T>& grid_values, const arr& grid_res);

} //namespace
