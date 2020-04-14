/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_Max : Feature {
  ptr<Feature> f;
  bool neg;

  F_Max(const ptr<Feature>& f, bool neg=false) : f(f), neg(neg) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Max:"<<f->shortTag((G))); }
};

//===========================================================================

struct F_Norm : Feature {
  ptr<Feature> f;

  F_Norm(const ptr<Feature>& f) : f(f) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Norm:"<<f->shortTag((G))); }
};

//===========================================================================

struct F_Normalized : Feature {
  ptr<Feature> f;

  F_Normalized(const ptr<Feature>& f) : f(f) {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G) { return f->__dim_phi(G); }
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("Normalized:"<<f->shortTag((G))); }
};
