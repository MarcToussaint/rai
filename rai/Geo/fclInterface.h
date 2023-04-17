/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

namespace rai {

struct FclInterface {
  struct FclInterface_self* self=0;
  
  double cutoff=-1.; //0 -> perform fine boolean collision check; >0 -> perform fine distance computations; <0 -> only broadphase
  uintA collisions; //return values!
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  FclInterface(const Array<shared_ptr<Mesh>>& geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X, double _cutoff=-2.);

protected:
  friend FclInterface_self;
  void addCollision(void* userData1, void* userData2);
};

}

