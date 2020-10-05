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
  Array<ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<fcl::CollisionObject*> objects;
  shared_ptr<fcl::BroadPhaseCollisionManager> manager;

  double cutoff=0.; //0 -> perform fine boolean collision check; >0 -> perform fine distance computations; <0 -> only broadphase
  uintA collisions; //return values!
  arr X_lastQuery;  //memory to check whether an object has moved in consecutive queries

  FclInterface(const Array<ptr<Mesh>>& geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);

private: //called by collision callback
  void addCollision(void* userData1, void* userData2);
  static bool BroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_);
};

}

