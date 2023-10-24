/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_InertialMotion : Feature {
  int i;
  double g, c;

  TM_InertialMotion(const rai::Configuration& K, const char* i_name, double g=-9.81, double c=0.)
    : i(initIdArg(K, i_name)), g(g), c(c) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple);

  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("InertialMotion"); }
};
