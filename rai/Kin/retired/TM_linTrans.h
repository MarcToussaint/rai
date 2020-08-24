/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "feature.h"

//===========================================================================

struct TM_LinTrans : Feature {
  ptr<Feature> map;
  arr A, a;

  TM_LinTrans(ptr<Feature> _map, const arr& A, const arr& a) : map(_map), A(A), a(a) {}
  ~TM_LinTrans() {}

  virtual void phi(arr& y, arr& J, const rai::Configuration& G);
  virtual uint dim_phi(const rai::Configuration& G);
  virtual rai::String shortTag(const rai::Configuration& G) { return STRING("LinTrans:"<<map->shortTag((G))); }
};
