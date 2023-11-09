#include "skeletonSymbol.h"
#include "../Core/util.h"

template<> const char* rai::Enum<rai::SkeletonSymbol>::names []= {
  //geometric:
  "touch", "above", "inside", "oppose", "restingOn",

  //pose constraints:
  "poseEq", "positionEq", "stableRelPose", "stablePose",

  //mode switches:
  "stable", "stableOn", "dynamic", "dynamicOn", "dynamicTrans", "quasiStatic", "quasiStaticOn", "downUp", "break", "stableZero",

  //interactions:
  "contact", "contactStick", "contactComplementary", "bounce", "push",

  //mode switches:
  "magic", "magicTrans",

  //integrated:
  "pushAndPlace",

  //grasps/placements:
  "topBoxGrasp", "topBoxPlace",

  "dampMotion",

  "identical",

  "alignByInt",

  "makeFree", "forceBalance",

  "relPosY",

  "touchBoxNormalX", "touchBoxNormalY", "touchBoxNormalZ",

  "boxGraspX", "boxGraspY", "boxGraspZ",

  "lift",

  "stableYPhi",
  "stableOnX",
  "stableOnY",

  "follow",
  "end",
  nullptr
};
