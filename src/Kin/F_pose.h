/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_Zeros : Feature {
  uint dim;
  F_Zeros(uint dim):dim(dim) {}
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return dim; }
};

//===========================================================================

struct F_Position : Feature {
  rai::Vector relPos;
  F_Position(const rai::Vector& _relPos=0) : relPos(_relPos) {}
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_PositionDiff : Feature {
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_PositionRel : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

//===========================================================================

struct F_PositionDistance : Feature {
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
};

//===========================================================================

struct F_Vector : Feature {
  rai::Vector vec;
  F_Vector(const rai::Vector& _vec) : vec(_vec) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_VectorDiff : Feature {
  rai::Vector vec1, vec2;
  F_VectorDiff(const rai::Vector& _vec1, const rai::Vector& _vec2)  : vec1(_vec1), vec2(_vec2) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_VectorRel: Feature {
  rai::Vector vec;
  F_VectorRel(const rai::Vector& _vec)  : vec(_vec) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

//===========================================================================

struct F_Matrix: Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 9; }
};

struct F_MatrixDiff : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 9; }
};

//===========================================================================

struct F_ScalarProduct : Feature {
  rai::Vector vec1, vec2;
  F_ScalarProduct(const rai::Vector& _vec1, const rai::Vector& _vec2)  : vec1(_vec1), vec2(_vec2) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
};

//===========================================================================

struct F_Quaternion : Feature {
  F_Quaternion() { flipTargetSignOnNegScalarProduct = true; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 4; }
};

struct F_QuaternionDiff : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 4; }
};

struct F_QuaternionRel: Feature {
  F_QuaternionRel() { flipTargetSignOnNegScalarProduct = true; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 4; }
};

//===========================================================================

struct F_Pose : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 7; }
};

struct F_PoseDiff : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 7; }
};

struct F_PoseRel : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 7; }
};

//===========================================================================

struct F_LinVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  F_LinVel() { order=1; }
  Feature& setImpulseInsteadOfAcceleration() { impulseInsteadOfAcceleration=true; return *this; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 3; }
};

struct F_AngVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  F_AngVel() { order=1; }
  Feature& setImpulseInsteadOfAcceleration() { impulseInsteadOfAcceleration=true; return *this; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& G) { return 3; }
};

struct F_LinAngVel : Feature {
  bool impulseInsteadOfAcceleration=false;
  F_LinAngVel() { order=1; }
  Feature& setImpulseInsteadOfAcceleration() { impulseInsteadOfAcceleration=true; return *this; }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 6; }
};

//===========================================================================

struct F_NoJumpFromParent_OBSOLETE : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 7; }
};

