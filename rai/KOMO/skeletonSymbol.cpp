#include "skeletonSymbol.h"
#include "../Core/util.h"

template<> const char* rai::Enum<rai::SkeletonSymbol>::names []= {
  "touch", "above", "inside", "oppose",
  "poseEq", "stableRelPose", "stablePose",
  "stable", "stableOn", "dynamic", "dynamicOn", "dynamicTrans", "quasiStatic", "quasiStaticOn", "downUp", "break", "stableZero",
  "contact", "contactStick", "contactComplementary", "bounce", "push",
  "magic", "magicTrans",
  "topBoxGrasp", "topBoxPlace",
  "dampMotion",
  "identical",
  "alignByInt",
  "makeFree", "forceBalance",
  "relPosY",
  "touchBoxNormalX", "touchBoxNormalY", "touchBoxNormalZ",
  "boxGraspX", "boxGraspY", "boxGraspZ",
  "stableYPhi",
  "end",
  nullptr
};
