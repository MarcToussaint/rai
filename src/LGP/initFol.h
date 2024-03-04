/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/kin.h"
#include "../Logic/folWorld.h"

struct KOMO;

namespace rai {

void initFolStateFromKin(struct FOL_World& L, const Configuration& K);

}//namespace
