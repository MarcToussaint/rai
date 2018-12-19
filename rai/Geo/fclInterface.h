#pragma once

#include "geoms.h"

namespace fcl{
  class CollisionObject;
  class DynamicAABBTreeCollisionManager;
};

namespace rai{

struct FclInterface{
  Array<ptr<Geom>> geometries;
  std::vector<fcl::CollisionObject*> objects;
  fcl::DynamicAABBTreeCollisionManager* manager;

  double cutoff=0.;
  uintA collisions;

  FclInterface(const Array<ptr<Geom>>& _geometries, double _cutoff=0.);
  ~FclInterface();

  void step(const arr& X);
};

}

