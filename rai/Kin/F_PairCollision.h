/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_PairCollision : Feature {
  enum Type { _none=-1, _negScalar, _vector, _normal, _center, _p1, _p2 };

  Type type;
  bool neglectRadii=false;

  F_PairCollision(Type _type, bool _neglectRadii=false);
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){  if(type==_negScalar) return 1; return 3;  }
};
