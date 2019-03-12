/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "taskMaps.h"

struct TM_PairCollision : Feature {
  enum Type { _none=-1, _negScalar, _vector, _normal, _center, _p1, _p2 };

  int i, j;               ///< which shapes does it refer to?
  Type type;
  bool neglectRadii=false;
  struct PairCollision *coll=0;

  TM_PairCollision(int _i, int _j, Type _type, bool _neglectRadii=false);
  TM_PairCollision(const rai::KinematicWorld& K, const char* s1, const char* s2, Type _type, bool neglectRadii=false);
  ~TM_PairCollision();
  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& G) { if(type==_negScalar) return 1;  return 3; }
  virtual rai::String shortTag(const rai::KinematicWorld& G);
  virtual Graph getSpec(const rai::KinematicWorld& K);
};
