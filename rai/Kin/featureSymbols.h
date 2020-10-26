/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

enum FeatureSymbol {
  FS_none=-1,
  FS_position,
  FS_positionDiff,
  FS_positionRel,
  FS_quaternion,
  FS_quaternionDiff,
  FS_quaternionRel,
  FS_pose,
  FS_poseDiff,
  FS_poseRel,
  FS_vectorX,
  FS_vectorXDiff,
  FS_vectorXRel,
  FS_vectorY,
  FS_vectorYDiff,
  FS_vectorYRel,
  FS_vectorZ,
  FS_vectorZDiff,
  FS_vectorZRel,
  FS_scalarProductXX,
  FS_scalarProductXY,
  FS_scalarProductXZ,
  FS_scalarProductYX,
  FS_scalarProductYY,
  FS_scalarProductYZ,
  FS_scalarProductZZ,
  FS_gazeAt,

  FS_angularVel,

  FS_accumulatedCollisions,
  FS_jointLimits,
  FS_distance,
  FS_oppose,

  FS_qItself,
  FS_qControl,

  FS_aboveBox,
  FS_insideBox,

  FS_pairCollision_negScalar,
  FS_pairCollision_vector,
  FS_pairCollision_normal,
  FS_pairCollision_p1,
  FS_pairCollision_p2,

  FS_standingAbove,

  FS_physics,
  FS_contactConstraints,
  FS_energy,

  FS_transAccelerations,
  FS_transVelocities,

  FS_qQuaternionNorms,
};

namespace rai {
struct Configuration;
}
struct Feature;

ptr<Feature> symbols2feature(FeatureSymbol feat, const StringA& frames, const rai::Configuration& C, const arr& scale=NoArr, const arr& target=NoArr, int order=-1);

inline ptr<Feature> make_feature(FeatureSymbol feat, const StringA& frames, const rai::Configuration& C, const arr& scale=NoArr, const arr& target=NoArr, int order=-1){
  return symbols2feature(feat, frames, C, scale, target, order);
}

