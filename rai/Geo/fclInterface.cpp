/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "fclInterface.h"

#ifdef RAI_FCL

#include <fcl/config.h>
#if FCL_MINOR_VERSION >= 6
#  include <fcl/fcl.h>
typedef fcl::CollisionObject<float> CollObject;
typedef fcl::Vector3<float> Vec3f;
typedef fcl::Quaternionf Quaternionf;
typedef fcl::BroadPhaseCollisionManager<float> BroadPhaseCollisionManager;
typedef fcl::DynamicAABBTreeCollisionManager<float> DynamicAABBTreeCollisionManager;
typedef fcl::CollisionRequest<float> CollisionRequest;
typedef fcl::CollisionResult<float> CollisionResult;
typedef fcl::DistanceRequest<float> DistanceRequest;
typedef fcl::DistanceResult<float> DistanceResult;
#else
#  include <fcl/broadphase/broadphase.h>
#  include <fcl/BVH/BVH_model.h>
#  include <fcl/distance.h>
#  include <fcl/collision.h>
#  include <fcl/collision_data.h>
typedef fcl::CollisionObject CollObject;
typedef fcl::Vec3f Vec3f;
typedef fcl::Quaternion3f Quaternionf;
typedef fcl::BroadPhaseCollisionManager BroadPhaseCollisionManager;
typedef fcl::DynamicAABBTreeCollisionManager DynamicAABBTreeCollisionManager;
typedef fcl::CollisionRequest CollisionRequest;
typedef fcl::CollisionResult CollisionResult;
typedef fcl::DistanceRequest DistanceRequest;
typedef fcl::DistanceResult DistanceResult;
#endif

namespace rai {
struct ConvexGeometryData {
  arr plane_dis;
  intA polygons;
};

struct FclInterface_self{
  Array<shared_ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<CollObject*> objects;
  shared_ptr<BroadPhaseCollisionManager> manager;

  static bool BroadphaseCallback(CollObject* o1, CollObject* o2, void* cdata_);
};

FclInterface::FclInterface(const Array<shared_ptr<Mesh>>& geometries, double _cutoff)
  : cutoff(_cutoff) {
  self = new FclInterface_self;
  
  self->convexGeometryData.resize(geometries.N);
  for(long int i=0; i<geometries.N; i++) {
    if(geometries(i)) {
      Mesh& mesh = *geometries(i);
#if 0
      auto model = make_shared<fcl::BVHModel<fcl::OBBRSS>>();
      model->beginModel();
      for(uint i=0; i<mesh.T.d0; i++)
        model->addTriangle(Vec3f(&mesh.V(mesh.T(i, 0), 0)), Vec3f(&mesh.V(mesh.T(i, 1), 0)), Vec3f(&mesh.V(mesh.T(i, 2), 0)));
      model->endModel();
#elif 1
      CHECK(!mesh.cvxParts.N, "NIY")
      //rai::Mesh mesh;
      //mesh.V = mesh_org.V;
      //mesh.makeConvexHull();
      mesh.computeNormals();
      std::shared_ptr<ConvexGeometryData> dat = make_shared<ConvexGeometryData>();
      dat->plane_dis = mesh.computeTriDistances();
      copy<int>(dat->polygons, mesh.T);
      dat->polygons.insColumns(0);
      for(uint i=0; i<dat->polygons.d0; i++) dat->polygons(i, 0) = 3;
#if FCL_MINOR_VERSION >= 7
      auto verts = make_shared<std::vector<fcl::Vector3<float>>>(mesh.V.d0);
      auto faces = make_shared<std::vector<int>>(mesh.T.N);
      for(uint i=0;i<verts->size();i++) (*verts)[i] = {(float)mesh.V(i,0), (float)mesh.V(i,1), (float)mesh.V(i,2)};
      for(uint i=0;i<faces->size();i++) (*faces)[i] = mesh.T.elem(i);
      auto model = make_shared<fcl::Convex<float>>(verts, mesh.T.d0, faces, true);
#else
      auto model = make_shared<fcl::Convex>((fcl::Vec3f*)mesh.Tn.p, dat->plane_dis.p, mesh.T.d0, (fcl::Vec3f*)mesh.V.p, mesh.V.d0, (int*)dat->polygons.p);
#endif
      self->convexGeometryData(i) = dat;
#else
      auto model = make_shared<fcl::Sphere>(mesh.getRadius());
#endif
      CollObject* obj = new CollObject(model, fcl::Transform3f());
      obj->setUserData((void*)(i));
      self->objects.push_back(obj);
    }
  }

//  manager = make_shared<fcl::IntervalTreeCollisionManager>();
  self->manager = make_shared<DynamicAABBTreeCollisionManager>();
//  manager = make_shared<fcl::SaPCollisionManager>();
//  manager = make_shared<fcl::NaiveCollisionManager>();
//  manager = new fcl::SpatialHashingCollisionManager();
  self->manager->registerObjects(self->objects);
  self->manager->setup();
}

FclInterface::~FclInterface() {
  for(size_t i = 0; i < self->objects.size(); ++i)
    delete self->objects[i];
  delete self;
}

void FclInterface::step(const arr& X, double _cutoff) {
  CHECK_EQ(X.nd, 2, "");
  CHECK_EQ(X.d0, self->convexGeometryData.N, "");
  CHECK_EQ(X.d1, 7, "");

  for(auto* obj:self->objects) {
    uint i = (long int)obj->getUserData();
    if(i<X_lastQuery.d0 && maxDiff(X_lastQuery[i], X[i])<1e-8) continue;
    obj->setTranslation(Vec3f(X(i, 0), X(i, 1), X(i, 2)));
    obj->setQuatRotation(Quaternionf(X(i, 3), X(i, 4), X(i, 5), X(i, 6)));
    obj->computeAABB();
  }
  self->manager->update();

  double defaultCutoff = cutoff;
  if(_cutoff>=0) cutoff = _cutoff;

  collisions.clear();
  self->manager->collide(this, FclInterface_self::BroadphaseCallback);
  collisions.reshape(-1, 2);

  if(_cutoff>=0) cutoff = defaultCutoff;

  X_lastQuery = X;
}

void FclInterface::addCollision(void* userData1, void* userData2) {
  uint a = (long int)userData1;
  uint b = (long int)userData2;
  collisions.resizeCopy(collisions.N+2);
  collisions.elem(-2) = a;
  collisions.elem(-1) = b;
}

bool FclInterface_self::BroadphaseCallback(CollObject* o1, CollObject* o2, void* cdata_) {
  FclInterface* fcl = static_cast<FclInterface*>(cdata_);

  if(fcl->cutoff==0.) { //fine boolean collision query
    CollisionRequest request;
    CollisionResult result;
    fcl::collide(o1, o2, request, result);
    if(result.isCollision()) fcl->addCollision(o1->getUserData(), o2->getUserData());
  } else if(fcl->cutoff>0.) { //fine distance query
    DistanceRequest request;
    DistanceResult result;
    fcl::distance(o1, o2, request, result);
    if(result.min_distance<fcl->cutoff) fcl->addCollision(o1->getUserData(), o2->getUserData());
  } else { //just broadphase
    fcl->addCollision(o1->getUserData(), o2->getUserData());
  }
  return false;
}
  
}//namespace

#else //RAI_FCL
rai::FclInterface::FclInterface(const rai::Array<shared_ptr<Mesh>>& _geometries, double _cutoff) { NICO }
rai::FclInterface::~FclInterface() { NICO }
void rai::FclInterface::step(const arr& X) { NICO }
#endif
