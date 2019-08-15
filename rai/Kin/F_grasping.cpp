#include "F_grasping.h"
#include "TM_PairCollision.h"


void F_GraspOppose::phi(arr& y, arr& J, const rai::KinematicWorld& K){
    Value D1 = TM_PairCollision(i, k, TM_PairCollision::_vector, true)(K);
    Value D2 = TM_PairCollision(j, k, TM_PairCollision::_vector, true)(K);

    y = D1.y + D2.y;
    if(!!J) J = D1.J + D2.J;
}
