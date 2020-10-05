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

  F_PairCollision(Type _type, bool _neglectRadii=false)
    : type(_type), neglectRadii(_neglectRadii) {
  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){  if(type==_negScalar) return 1; return 3;  }
};

//===========================================================================

//TODO: change naming: TMP_...

enum PTMtype {
  TMT_allP, //phi=sum over all proxies (as is standard)
  TMT_listedVsListedP, //phi=sum over all proxies between listed shapes
  TMT_allVsListedP, //phi=sum over all proxies against listed shapes
  TMT_allExceptListedP, //as above, but excluding listed shapes
  TMT_bipartiteP, //sum over proxies between the two sets of shapes (shapes, shapes2)
  TMT_pairsP, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  TMT_allExceptPairsP, //sum excluding these pairs
  TMT_vectorP //vector of all pair proxies (this is the only case where dim(phi)>1)
};

//===========================================================================

struct F_AccumulatedCollisions : Feature {
  PTMtype type;
  uintA shapes2;
  double margin;
  F_AccumulatedCollisions(PTMtype _type, uintA _shapes, double _margin=.0) : type(_type), margin(_margin) {  frameIDs=_shapes;  }
  virtual void phi2(arr& y, arr& J, const FrameL& F);
  virtual uint dim_phi2(const FrameL& F){ return 1; }
};

