/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

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
  Array<ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<fcl::CollisionObject*> objects;
  ptr<fcl::BroadPhaseCollisionManager> manager;

  double cutoff=0.;
  uintA collisions;
  uintA excludePairs;
  arr X_lastQuery;

  FclInterface(const Array<ptr<Mesh>>& _geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);

//private, called by collision callback
  void addCollision(void* userData1, void* userData2);
};

}

