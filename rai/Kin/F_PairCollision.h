/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

struct F_PairCollision : Feature {
  enum Type { _none=-1, _negScalar, _vector, _normal, _center, _p1, _p2 };

  int i, j;               ///< which shapes does it refer to?
  Type type;
  bool neglectRadii=false;
  unique_ptr<struct PairCollision> coll;

  F_PairCollision(int _i, int _j, Type _type, bool _neglectRadii=false);
  F_PairCollision(const rai::Configuration& K, const char* s1, const char* s2, Type _type, bool neglectRadii=false);
  ~F_PairCollision();
  virtual void phi(arr& y, arr& J, const rai::Configuration& K);
  virtual uint dim_phi(const rai::Configuration& G) { if(type==_negScalar) return 1;  return 3; }
  virtual rai::String shortTag(const rai::Configuration& G);
  virtual rai::Graph getSpec(const rai::Configuration& K);
};
