#pragma once

#include "../Kin/kin.h"
#include "../Logic/folWorld.h"

struct KOMO;

namespace rai {

void initFolStateFromKin(struct FOL_World& L, const Configuration& K);

}//namespace
