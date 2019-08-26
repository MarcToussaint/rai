/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once
#include "feature.h"

//===========================================================================

struct F_Pose : Feature {
  int a;
  F_Pose(int aShape) : a(aShape) {}
  F_Pose(const rai::KinematicWorld& C, const char* aShapeName=NULL)
      : F_Pose(initIdArg(C,aShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& C);
  virtual void phi(arr& y, arr& J, const WorldL& Ctuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 7; }
  virtual rai::String shortTag(const rai::KinematicWorld& C){ return STRING("F_Pose-" <<C.frames(a)->name ); }
};

//===========================================================================

struct F_PoseDiff : Feature {
  int a,b;
  F_PoseDiff(int aShape, int bShape) : a(aShape), b(bShape) {}
  F_PoseDiff(const rai::KinematicWorld& C, const char* aShapeName=NULL, const char* bShapeName=NULL)
      : F_PoseDiff(initIdArg(C,aShapeName), initIdArg(C,bShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& C);
  virtual void phi(arr& y, arr& J, const WorldL& Ctuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 7; }
  virtual rai::String shortTag(const rai::KinematicWorld& C){ return STRING("F_PoseDiff-" <<C.frames(a)->name <<'-' <<C.frames(b)->name); }
};

//===========================================================================

struct F_PoseRel : Feature {
  int a,b;
  F_PoseRel(int aShape, int bShape) : a(aShape), b(bShape) {}
  F_PoseRel(const rai::KinematicWorld& C, const char* aShapeName=NULL, const char* bShapeName=NULL)
      : F_PoseRel(initIdArg(C,aShapeName), initIdArg(C,bShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& C);
  virtual void phi(arr& y, arr& J, const WorldL& Ctuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 7; }
  virtual rai::String shortTag(const rai::KinematicWorld& C){ return STRING("F_PoseRel-" <<C.frames(a)->name <<'-' <<C.frames(b)->name); }
};
