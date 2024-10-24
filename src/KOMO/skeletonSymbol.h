/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

namespace rai {

enum SkeletonSymbol : int {
  SY_none=-1,

  //geometric:
  SY_touch, SY_above, SY_inside, SY_oppose, SY_restingOn,

  //pose constraints:
  SY_poseEq, SY_positionEq, SY_stableRelPose, SY_stablePose,

  //mode switches:
  SY_stable, SY_stableOn, SY_dynamic, SY_dynamicOn, SY_dynamicTrans, SY_quasiStatic, SY_quasiStaticOn, SY_downUp, SY_break, SY_stableZero,

  //interactions:
  SY_contact, SY_contactStick, SY_contactComplementary, SY_bounce, SY_push,

  //mode switches:
  SY_magic, SY_magicTrans,

  //integrated:
  SY_pushAndPlace,

  //grasps/placements:
  SY_topBoxGrasp, SY_topBoxPlace,

  SY_dampMotion,

  SY_identical,

  SY_alignByInt,

  SY_makeFree, SY_forceBalance,

  SY_relPosY,

  SY_touchBoxNormalX, SY_touchBoxNormalY, SY_touchBoxNormalZ,

  SY_boxGraspX, SY_boxGraspY, SY_boxGraspZ,

  SY_lift,

  SY_stableYPhi,
  SY_stableOnX,
  SY_stableOnY,

  SY_follow,

  SY_end,
};

} //namespace
