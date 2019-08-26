#include "F_pose.h"
#include "TM_default.h"

void F_Pose::phi(arr& y, arr& J, const rai::KinematicWorld& C){
    NIY;
}

void F_Pose::phi(arr& y, arr& J, const WorldL& Ctuple){
    arr yq, Jq;
    TM_Default tmp(TMT_pos, a);
    tmp.order = order;
    tmp.type = TMT_pos;
    tmp.Feature::__phi(y, J, Ctuple);
    tmp.type = TMT_quat;
    tmp.flipTargetSignOnNegScalarProduct=true;
    tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
    y.append(yq);
    if(!!J) J.append(Jq);
}

void F_PoseDiff::phi(arr& y, arr& J, const rai::KinematicWorld& C){
    NIY;
}

void F_PoseDiff::phi(arr& y, arr& J, const WorldL& Ctuple){
    arr yq, Jq;
    TM_Default tmp(TMT_posDiff, a, NoVector, b, NoVector);
    tmp.order = order;
    tmp.type = TMT_posDiff;
    tmp.Feature::__phi(y, J, Ctuple);
    tmp.type = TMT_quatDiff;
    tmp.flipTargetSignOnNegScalarProduct=true;
    tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
    y.append(yq);
    if(!!J) J.append(Jq);
}

void F_PoseRel::phi(arr& y, arr& J, const rai::KinematicWorld& C){
    NIY;
}

void F_PoseRel::phi(arr& y, arr& J, const WorldL& Ctuple){
    arr yq, Jq;
    TM_Default tmp(TMT_pos, a, NoVector, b, NoVector);
    tmp.order = order;
    tmp.type = TMT_pos;
    tmp.Feature::__phi(y, J, Ctuple);
    tmp.type = TMT_quat;
    tmp.flipTargetSignOnNegScalarProduct=true;
    tmp.Feature::__phi(yq, (!!J?Jq:NoArr), Ctuple);
    y.append(yq);
    if(!!J) J.append(Jq);
}



