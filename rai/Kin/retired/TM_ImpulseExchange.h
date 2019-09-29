/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_ImpulsExchange : Feature {
  int i,j;
  
  TM_ImpulsExchange(const rai::KinematicWorld &K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)) {}
    
  void phi(arr& y, arr& J, const WorldL& Ktuple);
  uint dim_phi(const rai::KinematicWorld& K) { return 6; }
  
  void phi(arr& y, arr& J, const rai::KinematicWorld& K) { HALT(""); }
  
  rai::String shortTag(const rai::KinematicWorld& K) { return STRING("ImpulseExchange"); }
};

struct TM_ImpulsExchange_weak : Feature {
  int i,j;
  
  TM_ImpulsExchange_weak(const rai::KinematicWorld &K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)) {}
    
  void phi(arr& y, arr& J, const WorldL& Ktuple);
  uint dim_phi(const rai::KinematicWorld& K) { return 3; }
  
  void phi(arr& y, arr& J, const rai::KinematicWorld& K) { HALT(""); }
  
  rai::String shortTag(const rai::KinematicWorld& K) { return STRING("ImpulseExchange"); }
};
