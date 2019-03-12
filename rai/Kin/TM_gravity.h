/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_Gravity : Feature {
  double gravity=9.81;
  
  TM_Gravity();
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const WorldL& Ktuple);
  
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("Gravity"); }
};

struct TM_Gravity2 : Feature {
  double gravity=9.81;
  int i;               ///< which shapes does it refer to?

  TM_Gravity2(int iShape=-1);
  TM_Gravity2(const rai::KinematicWorld& K, const char* iShapeName=NULL) : TM_Gravity2(initIdArg(K,iShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("Gravity2-" <<G.frames(i)->name); }
};

struct TM_ZeroAcc : Feature {
  int i;               ///< which shapes does it refer to?

  TM_ZeroAcc(int iShape=-1);
  TM_ZeroAcc(const rai::KinematicWorld& K, const char* iShapeName=NULL) : TM_ZeroAcc(initIdArg(K,iShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G){ return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("ZeroAcc-" <<G.frames(i)->name); }
};

struct TM_ZeroQVel : Feature {
  int i;               ///< which shapes does it refer to?

  TM_ZeroQVel(int iShape=-1) : i(iShape) { order=1; }
  TM_ZeroQVel(const rai::KinematicWorld& K, const char* iShapeName=NULL) : TM_ZeroQVel(initIdArg(K,iShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("ZeroQVel-" <<G.frames(i)->name); }
};
