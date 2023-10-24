/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"

//===========================================================================

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir,
                          const char* boxName, const arr& boxSize,
                          const char* gripperName, const char* palmName, const char* tableName, bool pre=false);

void addBoxPlaceObjectives(KOMO& komo, double time,
                           rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* tableName,
                           const char* gripperName, const char* palmName, double margin=-.02, bool pre=false);


