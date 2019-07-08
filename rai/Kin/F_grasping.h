#pragma once

#include "feature.h"

struct F_GraspOppose : Feature {
  int i, j, k;               ///< which shapes does it refer to?

  F_GraspOppose(int iShape=-1, int jShape=-1, int kShape=-1);
  F_GraspOppose(const rai::KinematicWorld& K,
               const char* iShapeName=NULL, const char* jShapeName=NULL, const char* kShapeName=NULL)
      : i(initIdArg(K, iShapeName)), j(initIdArg(K, jShapeName)), k(initIdArg(K, kShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("GraspOppose-" <<G.frames(i)->name <<'-' <<G.frames(j)->name <<'-' <<G.frames(k)->name); }
};
