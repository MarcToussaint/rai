#pragma once

#include <KOMO/komo.h>

enum BoundType{ BD_all=-1,
                BD_symbolic=0,
                BD_pose,
                BD_seq,
                BD_path,
                BD_seqPath,
                BD_seqVelPath,
                BD_max };

void skeleton2Bound(KOMO& komo,
                    BoundType boundType,
                    const Skeleton& S,
                    const rai::KinematicWorld& startKinematics,
                    const rai::KinematicWorld& parentEffKinematics,
                    bool collisions,
                    const arrA& waypoints={}
                    );
