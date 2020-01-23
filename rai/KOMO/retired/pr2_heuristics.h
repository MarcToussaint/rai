/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

namespace rai {
struct Configuration;
};

arr pr2_reasonable_W(const rai::Configuration& world);
uintA pr2_get_shapes(const rai::Configuration& world);
