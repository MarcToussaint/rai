/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct TM_qZeroVels:Feature {
  TM_qZeroVels() { }

  virtual void phi(arr& y, arr& J, const rai::Configuration& G) {NIY}
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const rai::Configuration& G) {NIY}
  virtual uint dim_phi(const ConfigurationL& Ktuple);
private:
  std::map<rai::Configuration*, uint> dimPhi;
};

