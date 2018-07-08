/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_AngVel : Feature {
  int i;               ///< which shapes does it refer to?

  TM_AngVel(int iShape=-1)
    : i(iShape) {}

  TM_AngVel(const rai::KinematicWorld& K, const char* iShapeName=NULL)
    : i(initIdArg(K,iShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G){ NIY; }
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const rai::KinematicWorld& G);
  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("Vel_" <<G.frames(i)->name); }
};
