/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

enum TM_qItself_PickMode { QIP_byJointNames, QIP_byBodyNames, QIP_byJointGroups, QIP_byExcludeJointNames };

struct TM_qItself:Feature {
  uintA selectedBodies; ///< optionally, select only a subset of joints, indicated by the BODIES! indices (reason: body indices are stable across kinematic switches)
  bool moduloTwoPi; ///< if false, consider multiple turns of a joint as different q values (Default: true)
  bool relative_q0; ///< if true, absolute values are given relative to Joint::q0
  
  TM_qItself(bool relative_q0=false);
  TM_qItself(TM_qItself_PickMode pickMode, const StringA& picks, const rai::KinematicWorld& G, bool relative_q0=false);
  TM_qItself(const uintA& _selectedBodies, bool relative_q0=false);
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G);
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual uint dim_phi(const WorldL& Ktuple);
  virtual rai::String shortTag(const rai::KinematicWorld& G);
private:
  std::map<rai::KinematicWorld*, uint> dimPhi;
};

//===========================================================================

struct TM_qZeroVels:Feature {
  TM_qZeroVels() { }
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) {NIY}
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G) {NIY}
  virtual uint dim_phi(const WorldL& Ktuple);
private:
  std::map<rai::KinematicWorld*, uint> dimPhi;
};

//===========================================================================

rai::Array<rai::Joint*> getMatchingJoints(const WorldL& Ktuple, bool zeroVelJointsOnly);
rai::Array<rai::Joint*> getSwitchedJoints(const rai::KinematicWorld& G0, const rai::KinematicWorld& G1, int verbose=0);
uintA getSwitchedBodies(const rai::KinematicWorld& G0, const rai::KinematicWorld& G1, int verbose=0);
uintA getNonSwitchedBodies(const WorldL& Ktuple);

