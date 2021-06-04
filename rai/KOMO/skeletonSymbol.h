#pragma once

namespace rai {

enum SkeletonSymbol {
  SY_none=-1,

  //geometric:
  SY_touch, SY_above, SY_inside, SY_oppose,

  //pose constraints:
  SY_poseEq, SY_stableRelPose, SY_stablePose,

  //mode switches:
  SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn, SY_downUp, SY_break,

  //interactions:
  SY_contact, SY_contactStick, SY_contactComplementary, SY_bounce,

  //mode switches:
  SY_magic, SY_magicTrans,

  //grasps/placements:
  SY_topBoxGrasp, SY_topBoxPlace,

  SY_dampMotion,

  SY_identical,

  SY_alignByInt,

  SY_makeFree, SY_forceBalance,

  SY_touchBoxNormalX, SY_touchBoxNormalY, SY_touchBoxNormalZ,

  SY_end,
};

} //namespace
