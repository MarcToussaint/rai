/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_qItself : Feature {
  bool relative_q0; ///< if true, absolute values are given relative to Joint::q0

  F_qItself(const uintA& _selectedFrames={}, bool relative_q0=false);
  void selectActiveJointPairs(const FrameL& F);

  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F);
 private:
  std::map<rai::Configuration*, uint> dimPhi;
};

//===========================================================================

struct F_q0Bias : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F);
};

//===========================================================================

struct F_qZeroVel : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F);
};

//===========================================================================

struct F_qLimits : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F);
};

//===========================================================================

struct F_qQuaternionNorms : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F);

  void setAllActiveQuats(const rai::Configuration& C);
};

//===========================================================================

struct F_qTime : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
};

//===========================================================================

rai::Array<rai::Joint*> getMatchingJoints(const ConfigurationL& Ktuple, bool zeroVelJointsOnly);
uintA getNonSwitchedFrames(const FrameL& A, const FrameL& B);
uintA getSwitchedFrames(const FrameL& A, const FrameL& B);
