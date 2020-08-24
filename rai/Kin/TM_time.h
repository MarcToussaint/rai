/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_Time : Feature {
  TM_Time() {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K);
  virtual uint dim_phi(const rai::Configuration& K) { return 1; }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple) { return 1; }

  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Time"); }
};
