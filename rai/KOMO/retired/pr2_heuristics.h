/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>

namespace rai {
struct KinematicWorld;
};

arr pr2_reasonable_W(const rai::KinematicWorld& world);
uintA pr2_get_shapes(const rai::KinematicWorld& world);
