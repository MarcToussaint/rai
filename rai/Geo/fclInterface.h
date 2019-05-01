#pragma once

#include "mesh.h"

namespace fcl{
  class CollisionObject;
  class DynamicAABBTreeCollisionManager;
};

namespace rai{

struct FclInterface{
  Array<ptr<Mesh>> geometries;
  std::vector<fcl::CollisionObject*> objects;
  fcl::DynamicAABBTreeCollisionManager* manager;

  double cutoff=0.;
  uintA collisions;

  FclInterface(const Array<ptr<Mesh>>& _geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);
};

}

