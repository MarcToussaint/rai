#pragma once

#include <KOMO/komo.h>

enum BoundType{ BD_all=-1, BD_symbolic=0, BD_pose=1, BD_seq=2, BD_path=3 };

void skeleton2Bound(KOMO& komo, BoundType boundType, const Skeleton& S, const rai::KinematicWorld& startKinematics, const rai::KinematicWorld& parentEffKinematics, bool collisions);
