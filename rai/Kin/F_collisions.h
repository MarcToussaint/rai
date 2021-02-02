/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct F_PairCollision : Feature {
  enum Type { _none=-1, _negScalar, _vector, _normal, _center, _p1, _p2 };

  Type type;
  bool neglectRadii=false;

  shared_ptr<struct PairCollision> coll;

  F_PairCollision(Type _type, bool _neglectRadii=false)
    : type(_type), neglectRadii(_neglectRadii) {
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){  if(type==_negScalar) return 1; return 3;  }
};

//===========================================================================

struct F_PairFunctional : Feature, GLDrawer {
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 1; }
  virtual void glDraw(struct OpenGL&);
protected:
  arr x;
  double d1, d2;
  arr g1, g2;
};

//===========================================================================

struct F_AccumulatedCollisions : Feature {
  double margin;
  F_AccumulatedCollisions(double _margin=.0) : margin(_margin) {}
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 1; }
};

