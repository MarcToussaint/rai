/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Kin/kin.h>

rai::String validatePath(const rai::Configuration& _K, const arr& q_now, const StringA& joints, const arr& q, const arr& tau);

std::pair<arr, arr> computePath(const rai::Configuration& K, const arr& target_q, const StringA& target_joints= {}, const char* endeff=nullptr, double up=.2, double down=.8);

void mirrorDuplicate(std::pair<arr, arr>& path);
