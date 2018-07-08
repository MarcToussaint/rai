#include "taskMap.h"

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
  FS_scalarProductZ,
  FS_gazeAt,

  FS_accumulatedCollisions,
  FS_jointLimits,
  FS_distance,

  FS_qItself,

  FS_aboveBox,
  FS_insideBox,

  FS_standingAbove,
};


TaskMap *symbols2feature(FeatureSymbol feat, const StringA &symbols, const rai::KinematicWorld& world);
