/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct TM_Gravity : Feature {
  double gravity=9.81;

  TM_Gravity();

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple);

  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Gravity"); }
};

struct TM_Gravity2 : Feature {
  double gravity=9.81;
  int i;               ///< which shapes does it refer to?

  TM_Gravity2(int iShape=-1);
  TM_Gravity2(const rai::Configuration& K, const char* iShapeName=nullptr) : TM_Gravity2(initIdArg(K, iShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Gravity2-" <<G.frames(i)->name); }
};

struct TM_ZeroAcc : Feature {
  int i;               ///< which shapes does it refer to?

  TM_ZeroAcc(int iShape=-1);
  TM_ZeroAcc(const rai::Configuration& K, const char* iShapeName=nullptr) : TM_ZeroAcc(initIdArg(K, iShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G) { NIY; }
  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const rai::Configuration& G) { return 3; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("ZeroAcc-" <<G.frames(i)->name); }
};

