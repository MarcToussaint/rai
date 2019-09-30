#pragma once

#include <Kin/kin.h>

rai::String validatePath(const rai::Configuration& _K, const arr& q_now, const StringA& joints, const arr& q, const arr& tau);

std::pair<arr,arr> computePath(const rai::Configuration& K, const arr& target_q, const StringA& target_joints={}, const char* endeff=NULL, double up=.2, double down=.8);

void mirrorDuplicate(std::pair<arr,arr>& path);
