/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_Max : Feature {
  shared_ptr<Feature> f;
  bool neg;

  F_Max(const shared_ptr<Feature>& f, bool neg=false) : f(f), neg(neg) {}

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("Max:"<<f->shortTag(C)); }
};

//===========================================================================

struct F_Norm : Feature {
  shared_ptr<Feature> f;

  F_Norm(const shared_ptr<Feature>& f) : f(f) {}

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("Norm:"<<f->shortTag(C)); }
};

//===========================================================================

struct F_Normalized : Feature {
  shared_ptr<Feature> f;

  F_Normalized(const shared_ptr<Feature>& f) : f(f) {}

  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return f->dim(F); }
  virtual rai::String shortTag(const rai::Configuration& C) { return STRING("Normalized:"<<f->shortTag(C)); }
};
