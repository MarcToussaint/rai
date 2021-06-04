#include "skeletonSymbol.h"
#include "../Core/util.h"

template<> const char* rai::Enum<rai::SkeletonSymbol>::names []= {
  "touch", "above", "inside", "oppose",
  "poseEq", "stableRelPose", "stablePose",
  "stable", "stableOn", "dynamic", "dynamicOn", "dynamicTrans", "quasiStatic", "quasiStaticOn", "downUp", "break",
  "contact", "contactStick", "contactComplementary", "bounce",
  "magic", "magicTrans",
  "topBoxGrasp", "topBoxPlace",
  "dampMotion",
  "identical",
  "alignByInt",
  "makeFree", "forceBalance",
  "touchBoxNormalX", "touchBoxNormalY", "touchBoxNormalZ",
  "end",
  nullptr
};
