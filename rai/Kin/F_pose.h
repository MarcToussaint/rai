/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_Position : Feature {
  F_Position() {}
  F_Position(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_Position"); }
};

//===========================================================================

struct F_PositionDiff : Feature {
  F_PositionDiff() {}
  F_PositionDiff(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_PositionDiff"); }
};

//===========================================================================

struct F_Vector : Feature {
  rai::Vector vec;
  F_Vector(const rai::Vector& _vec) : vec(_vec) {}
//  F_Vector(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_Vector"); }
};

//===========================================================================

struct F_VectorDiff : Feature {
  rai::Vector vec1, vec2;
  F_VectorDiff(const rai::Vector& _vec1, const rai::Vector& _vec2)  : vec1(_vec1), vec2(_vec2) {}
//  F_VectorDiff(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F){ NIY }
  virtual uint dim_phi2(const FrameL& F) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_VectorDiff"); }
};

//===========================================================================

struct F_ScalarProduct : Feature {
  rai::Vector vec1, vec2;
  F_ScalarProduct(const rai::Vector& _vec1, const rai::Vector& _vec2)  : vec1(_vec1), vec2(_vec2) {}
//  F_Vector(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_ScalarProduct"); }
};

//===========================================================================

struct F_Quaternion : Feature {
  F_Quaternion() {}
  F_Quaternion(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) { flipTargetSignOnNegScalarProduct=true; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 4; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_Quaternion"); }
};

//===========================================================================

struct F_QuaternionDiff : Feature {
  F_QuaternionDiff() {}
  F_QuaternionDiff(const uintA& _frameIDs, uint _order) : Feature(_frameIDs, _order) { flipTargetSignOnNegScalarProduct=true; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 4; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_QuaternionDiff"); }
};


//===========================================================================

struct F_Pose : Feature {
  uint a;
  F_Pose(uint aShape) : a(aShape) {}
  F_Pose(const rai::Configuration& C, const char* aShapeName=nullptr)
    : F_Pose(initIdArg(C, aShapeName)) {}

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 7; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_Pose-" <<C.frames(a)->name); }
};

//===========================================================================

struct F_PoseDiff : Feature {
  uint a, b;
  F_PoseDiff(int aShape, int bShape) : a(aShape), b(bShape) {}
  F_PoseDiff(const rai::Configuration& C, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : F_PoseDiff(initIdArg(C, aShapeName), initIdArg(C, bShapeName)) {}

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 7; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_PoseDiff-" <<C.frames(a)->name <<'-' <<C.frames(b)->name); }
};

//===========================================================================

struct F_PoseRel : Feature {
  int a, b;
  F_PoseRel(int aShape, int bShape) : a(aShape), b(bShape) {}
  F_PoseRel(const rai::Configuration& C, const char* aShapeName=nullptr, const char* bShapeName=nullptr)
    : F_PoseRel(initIdArg(C, aShapeName), initIdArg(C, bShapeName)) {}

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 7; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("F_PoseRel-" <<C.frames(a)->name <<'-' <<C.frames(b)->name); }
};

//===========================================================================

struct TM_Align : Feature {
  int i, j;               ///< which shapes does it refer to?

  TM_Align(const rai::Configuration& G, const char* iName=nullptr, const char* jName=nullptr);

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& G);
};

