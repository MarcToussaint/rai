/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct F_pushed : Feature {
  int i;               ///< which shapes does it refer to?

  F_pushed(int iShape);
  F_pushed(const rai::KinematicWorld& K, const char* iShapeName) : F_pushed(initIdArg(K,iShapeName)){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K){ HALT("can't be here"); }
  virtual uint dim_phi(const rai::KinematicWorld& K){ HALT("can't be here"); }

  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const WorldL& Ktuple){ return 3; }

  virtual rai::String shortTag(const rai::KinematicWorld& K) { return STRING("static-" <<K.frames(i)->name); }
};


