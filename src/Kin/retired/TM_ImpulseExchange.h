/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_ImpulsExchange : Feature {
  int i, j;

  TM_ImpulsExchange(const rai::Configuration& K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)) {}

  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { return 6; }

  void phi(arr& y, arr& J, const rai::Configuration& K) { HALT(""); }

  rai::String shortTag(const rai::Configuration& K) { return STRING("ImpulseExchange"); }
};

struct TM_ImpulsExchange_weak : Feature {
  int i, j;

  TM_ImpulsExchange_weak(const rai::Configuration& K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)) {}

  void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  uint dim_phi(const rai::Configuration& K) { return 3; }

  void phi(arr& y, arr& J, const rai::Configuration& K) { HALT(""); }

  rai::String shortTag(const rai::Configuration& K) { return STRING("ImpulseExchange"); }
};
