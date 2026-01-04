/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/frame.h"
#include "pairCollision.h"
#include "../Kin/proxy.h"

namespace rai {

struct CoalInterface {
  struct CoalInterface_self* self=0;

  double cutoff=.01;
  uintAA excludes;
  // uintA collisions; //return values!
  Array<Proxy> collisions;
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  CoalInterface(const Array<Shape*>& geometries, const uintAA& _excludes);
  ~CoalInterface();

  void setActiveColliders(uintA geom_ids);

  void step(const arr& X, CollisionQueryMode mode); //TODO: this should have the query model as argument

 protected:
  friend CoalInterface_self;
  Proxy& addCollision(uint a, uint b);
};

//===========================================================================

struct PairCollision_Coal : PairCollision, NonCopyable {

  PairCollision_Coal(Shape* s1, Shape* s2,
                     const rai::Transformation& t1, const rai::Transformation& t2,
                     double _rad1=0., double _rad2=0.);
};


}

