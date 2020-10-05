/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "fclInterface.h"

#ifdef RAI_FCL

#include <fcl/broadphase/broadphase.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>

namespace rai {
struct ConvexGeometryData {
  arr plane_dis;
  intA polygons;
};
}

rai::FclInterface::FclInterface(const rai::Array<ptr<Mesh>>& geometries, double _cutoff)
  : cutoff(_cutoff) {
  convexGeometryData.resize(geometries.N);
  for(long int i=0; i<geometries.N; i++) {
    if(geometries(i)) {
      rai::Mesh& mesh = *geometries(i);
#if 0
      auto model = make_shared<fcl::BVHModel<fcl::OBBRSS>>();
      model->beginModel();
      for(uint i=0; i<mesh.T.d0; i++)
        model->addTriangle(fcl::Vec3f(&mesh.V(mesh.T(i, 0), 0)), fcl::Vec3f(&mesh.V(mesh.T(i, 1), 0)), fcl::Vec3f(&mesh.V(mesh.T(i, 2), 0)));
      model->endModel();
#elif 1
      mesh.computeNormals();
      std::shared_ptr<ConvexGeometryData> dat = make_shared<ConvexGeometryData>();
      dat->plane_dis = mesh.computeTriDistances();
      copy<int>(dat->polygons, mesh.T);
      dat->polygons.insColumns(0);
      for(uint i=0; i<dat->polygons.d0; i++) dat->polygons(i, 0) = 3;
      auto model = make_shared<fcl::Convex>((fcl::Vec3f*)mesh.Tn.p, dat->plane_dis.p, mesh.T.d0, (fcl::Vec3f*)mesh.V.p, mesh.V.d0, (int*)dat->polygons.p);
      convexGeometryData(i) = dat;
#else
      auto model = make_shared<fcl::Sphere>(mesh.getRadius());
#endif
      fcl::CollisionObject* obj = new fcl::CollisionObject(model, fcl::Transform3f());
      obj->setUserData((void*)(i));
      objects.push_back(obj);
    }
  }

//  manager = make_shared<fcl::IntervalTreeCollisionManager>();
  manager = make_shared<fcl::DynamicAABBTreeCollisionManager>();
//  manager = make_shared<fcl::SaPCollisionManager>();
//  manager = make_shared<fcl::NaiveCollisionManager>();
//  manager = new fcl::SpatialHashingCollisionManager();
  manager->registerObjects(objects);
  manager->setup();
}

rai::FclInterface::~FclInterface() {
  for(size_t i = 0; i < objects.size(); ++i)
    delete objects[i];
}

void rai::FclInterface::step(const arr& X) {
  CHECK_EQ(X.nd, 2, "");
  CHECK_EQ(X.d0, convexGeometryData.N, "");
  CHECK_EQ(X.d1, 7, "");

  for(auto* obj:objects) {
    uint i = (long int)obj->getUserData();
    if(i<X_lastQuery.d0 && maxDiff(X_lastQuery[i], X[i])<1e-8) continue;
    obj->setTranslation(fcl::Vec3f(X(i, 0), X(i, 1), X(i, 2)));
    obj->setQuatRotation(fcl::Quaternion3f(X(i, 3), X(i, 4), X(i, 5), X(i, 6)));
    obj->computeAABB();
  }
  manager->update();

  collisions.clear();
  manager->collide(this, BroadphaseCallback);
  collisions.reshape(collisions.N/2, 2);

  X_lastQuery = X;
}

void rai::FclInterface::addCollision(void* userData1, void* userData2) {
  uint a = (long int)userData1;
  uint b = (long int)userData2;
  collisions.resizeCopy(collisions.N+2);
  collisions.elem(-2) = a;
  collisions.elem(-1) = b;
}

bool rai::FclInterface::BroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_) {
  rai::FclInterface* self = static_cast<rai::FclInterface*>(cdata_);

  if(self->cutoff==0.) { //fine boolean collision query
    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::collide(o1, o2, request, result);
    if(result.isCollision()) self->addCollision(o1->getUserData(), o2->getUserData());
  } else if(self->cutoff>0.) { //fine distance query
    fcl::DistanceRequest request;
    fcl::DistanceResult result;
    fcl::distance(o1, o2, request, result);
    if(result.min_distance<self->cutoff) self->addCollision(o1->getUserData(), o2->getUserData());
  } else { //just broadphase
    self->addCollision(o1->getUserData(), o2->getUserData());
  }
  return false;
}

#else //RAI_FCL
rai::FclInterface::FclInterface(const Array<ptr<Mesh>>& _geometries, double _cutoff) { NICO }
rai::FclInterface::~FclInterface() { NICO }
void rai::FclInterface::step(const arr& X) { NICO }
#endif
