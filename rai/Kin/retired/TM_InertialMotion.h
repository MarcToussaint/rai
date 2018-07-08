/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_InertialMotion : Feature {
  int i;
  double g,c;
  
  TM_InertialMotion(const rai::KinematicWorld &K, const char* i_name, double g=-9.81, double c=0.)
    : i(initIdArg(K, i_name)), g(g), c(c) {}
    
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& K) { HALT("can only be of higher order"); }
  
  virtual void phi(arr& y, arr& J, const WorldL& Ktuple);
  virtual uint dim_phi(const WorldL& Ktuple);
  
  virtual rai::String shortTag(const rai::KinematicWorld& G) { return STRING("InertialMotion"); }
};
