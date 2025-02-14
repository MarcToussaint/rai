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
  virtual uint dim_phi(const FrameL& F);
};

//===========================================================================

struct F_PairNormalAlign : Feature {
  double dir = 1.;
  F_PairNormalAlign(double _dir=1.){ dir=_dir; }
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F){ return 3; }
};


//===========================================================================

struct F_PairFunctional : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
  std::shared_ptr<struct SweepingSDFPenetration> P;
  arr x;
  double d1, d2;
  arr g1, g2;
};

//===========================================================================

struct F_AccumulatedCollisions : Feature {
  double margin;
  F_AccumulatedCollisions(double _margin=.0) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi(const FrameL& F) { return 1; }
};

//===========================================================================

struct F_VelocityDistance : Feature {
  double margin;
  F_VelocityDistance(double margin=.05) : margin(margin) {}
  virtual arr phi(const FrameL& F);
  virtual uint dim_phi(const FrameL& F) {  return 3;  }
};

