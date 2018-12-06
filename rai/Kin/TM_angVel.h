/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

//===========================================================================

struct TM_LinVel : Feature {
  int i;               ///< which shapes does it refer to?
  bool impulseInsteadOfAcceleration=false;

  TM_LinVel(int iShape=-1) : i(iShape) { order=1; }
  TM_LinVel(const rai::KinematicWorld& K, const char* iShapeName=NULL) : i(initIdArg(K,iShapeName)) { order=1; }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("TM_LinVel-" <<order <<'-' <<G.frames(i)->name); }
};


//===========================================================================

struct TM_AngVel : Feature {
  int i;               ///< which shapes does it refer to?
  bool impulseInsteadOfAcceleration=false;

  TM_AngVel(int iShape=-1) : i(iShape) { order=1; }
  TM_AngVel(const rai::KinematicWorld& K, const char* iShapeName=NULL) : i(initIdArg(K,iShapeName)) { order=1; }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("AngVel-" <<order <<'-' <<G.frames(i)->name); }
};

//===========================================================================

struct TM_LinAngVel : Feature {
  int i;               ///< which shapes does it refer to?

  TM_LinAngVel(int iShape=-1) : i(iShape) { order=1; }
  TM_LinAngVel(const rai::KinematicWorld& K, const char* iShapeName=NULL) : i(initIdArg(K,iShapeName)) { order=1; }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("LinAngVel-" <<order <<'-' <<G.frames(i)->name); }
};

//===========================================================================

struct TM_NoJumpFromParent : Feature {
  int i;               ///< which shapes does it refer to?

  TM_NoJumpFromParent(int iShape=-1) : i(iShape) { order=1; }
  TM_NoJumpFromParent(const rai::KinematicWorld& K, const char* iShapeName=NULL) : i(initIdArg(K,iShapeName)) { order=1; }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("TM_NoJumpToParent-" <<order <<'-' <<G.frames(i)->name); }
};
