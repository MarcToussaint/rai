/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

namespace rai {
struct PairCollision;
}

//===========================================================================

struct F_PairCollision : Feature {
  enum Type { _none=-1, _negScalar, _vector, _normal, _center, _p1, _p2 };

  Type type;
  bool neglectRadii=false;

  shared_ptr<rai::PairCollision> coll;

  F_PairCollision(Type _type=_negScalar, bool _neglectRadii=false)
    : type(_type), neglectRadii(_neglectRadii) {
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F);
};

//===========================================================================

struct F_PairFunctional : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 1; }
  std::shared_ptr<struct SweepingSDFPenetration> P;
  arr x;
  double d1, d2;
  arr g1, g2;
};

//===========================================================================

struct F_AccumulatedCollisions : Feature {
  double margin;
  bool selectAll=false;
  bool selectXor=false;
  F_AccumulatedCollisions(double _margin=.0, bool selAll=false, bool selXor=false) : margin(_margin), selectAll(selAll), selectXor(selXor) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F) { return 1; }
};

