/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "mesh.h"

namespace fcl {
class CollisionObject;
class DynamicAABBTreeCollisionManager;
class BroadPhaseCollisionManager;
};

namespace rai {

struct FclInterface {
  Array<ptr<Mesh>> geometries;
  std::vector<fcl::CollisionObject*> objects;
  fcl::BroadPhaseCollisionManager* manager;

  double cutoff=0.;
  uintA collisions;
  uintA excludePairs;

  FclInterface(const Array<ptr<Mesh>>& _geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);

//private, called by collision callback
  void addCollision(void* userData1, void* userData2);
};

}

