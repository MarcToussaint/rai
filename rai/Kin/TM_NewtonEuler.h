/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_NewtonEuler : Feature {
  int i;               ///< which shapes does it refer to?
  double gravity=9.81;
  
  TM_NewtonEuler(int iShape);
  TM_NewtonEuler(const rai::KinematicWorld& K, const char* iShapeName=NULL) : TM_NewtonEuler(initIdArg(K,iShapeName)){}
  
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const WorldL& Ktuple){ return 6; }
  
  virtual rai::String shortTag(const rai::KinematicWorld& K) { return STRING("NewtonEuler-" <<K.frames(i)->name); }
};


