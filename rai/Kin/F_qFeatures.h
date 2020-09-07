/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_qItself : Feature {
  enum PickMode { byJointNames, byFrameNames, byJointGroups, byExcludeJointNames, allActiveJoints };

//  uintA frameIDs; ///< optionally, select only a subset of joints, indicated by the BODIES! indices (reason: frame indices are stable across kinematic switches)
  bool moduloTwoPi; ///< if false, consider multiple turns of a joint as different q values (Default: true)
  bool relative_q0; ///< if true, absolute values are given relative to Joint::q0

  F_qItself(bool relative_q0=false);
  F_qItself(PickMode pickMode, const StringA& picks, const rai::Configuration& C, bool relative_q0=false);
  F_qItself(const uintA& _selectedFrames, bool relative_q0=false);

  virtual void phi(arr& y, arr& J, const rai::Configuration& C);
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const rai::Configuration& C);
  virtual uint dim_phi(const ConfigurationL& Ctuple);
  virtual uint dim_phi2(const FrameL& F);
  virtual void signature(intA& S, const rai::Configuration& C);
  virtual rai::String shortTag(const rai::Configuration& G);
 private:
  std::map<rai::Configuration*, uint> dimPhi;
};

//===========================================================================

struct F_qZeroVel : Feature {
  bool useChildFrame;

  F_qZeroVel(int iShape, bool _useChildFrame=false) : useChildFrame(_useChildFrame) { frameIDs = TUP(iShape); order=1; }
  F_qZeroVel(const rai::Configuration& K, const char* iShapeName=nullptr, bool _useChildFrame=false) : F_qZeroVel(initIdArg(K, iShapeName), _useChildFrame) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ctuple);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const rai::Configuration& G);
  virtual uint dim_phi2(const FrameL& F);

  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("qZeroVel"); }
};

//===========================================================================

struct F_qLimits2 : Feature {
  F_qLimits2(const uintA& frames) { frameIDs = frames; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("qLimits2"); }
};

//===========================================================================

struct F_qLimits : Feature {
  //TODO (danny) allow margin specification
  arr limits;

  F_qLimits(const arr& _limits=NoArr) { if(!!_limits) limits=_limits; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("qLimits"); }
};

//===========================================================================

struct F_qQuaternionNorms : Feature {
  F_qQuaternionNorms() { fs = FS_qQuaternionNorms; }
  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G);
  virtual void signature(intA& S, const rai::Configuration& C);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("QuaternionNorms"); }
};

//===========================================================================

rai::Array<rai::Joint*> getMatchingJoints(const ConfigurationL& Ktuple, bool zeroVelJointsOnly);
rai::Array<rai::Joint*> getSwitchedJoints(const rai::Configuration& G0, const rai::Configuration& G1, int verbose=0);
uintA getSwitchedBodies(const rai::Configuration& G0, const rai::Configuration& G1, int verbose=0);
uintA getNonSwitchedFrames(const ConfigurationL& Ktuple);

