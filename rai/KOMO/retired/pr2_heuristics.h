/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

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
