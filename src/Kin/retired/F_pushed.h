/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "feature.h"

struct F_pushed : Feature {
  int i;               ///< which shapes does it refer to?

  F_pushed(int iShape);
  F_pushed(const rai::Configuration& K, const char* iShapeName) : F_pushed(initIdArg(K, iShapeName)) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& K) { HALT("can't be here"); }
  virtual uint dim_phi(const rai::Configuration& K) { HALT("can't be here"); }

  virtual void phi(arr& y, arr& J, const ConfigurationL& Ktuple);
  virtual uint dim_phi(const ConfigurationL& Ktuple) { return 6; }

  virtual rai::String shortTag(const rai::Configuration& K) { return STRING("pushed-" <<K.frames(i)->name); }
};

