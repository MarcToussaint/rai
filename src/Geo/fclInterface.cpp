/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "fclInterface.h"

#ifdef RAI_FCL

#ifdef RAI_CCD //to ensure this is loaded locally, not from FCL
#  include "./ccd_rai/ccd.h"
#  include "./ccd_rai/quat.h"
#endif

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

struct FclInterface_self {
  Array<shared_ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<CollObject*> objects;
  rai::Array<CollObject*> activeColliders;
  shared_ptr<BroadPhaseCollisionManager> manager;

  static bool BroadphaseCallback(CollObject* o1, CollObject* o2, void* cdata_);
};

FclInterface::FclInterface(const Array<Shape*>& geometries, const uintAA& _excludes, QueryMode _mode)
  : mode(_mode), excludes(_excludes) {
  self = new FclInterface_self;

  self->convexGeometryData.resize(geometries.N);
  rai::Array<rai::Mesh> cvx_meshes(geometries.N);

  for(long int i=0; i<geometries.N; i++) {
    Shape* shape = geometries(i);
    if(shape) {
      std::shared_ptr<fcl::CollisionGeometry> geom;
      if(shape->type()==ST_capsule) {
        geom = make_shared<fcl::Capsule>(shape->size(-1), shape->size(-2));
      } else if(shape->type()==ST_cylinder) {
        geom = make_shared<fcl::Cylinder>(shape->size(-1), shape->size(-2));
      } else if(shape->type()==ST_sphere) {
        geom = make_shared<fcl::Sphere>(shape->size(-1));
      }else if(shape->type()==ST_box){
        geom = make_shared<fcl::Box>(shape->size(0), shape->size(1), shape->size(2));
      } else {
        CHECK(shape->sscCore().N, "for FCL broadphase, every shape with 'contact' enabled needs a convex core"); // <<shape->frame.name);
        CHECK_EQ(shape->radius(), shape->coll_cvxRadius, "should be equal");
        rai::Mesh& mesh = cvx_meshes(i);
        mesh.setSSCvx(shape->sscCore(), shape->coll_cvxRadius*1.05, 1); //make it a little larger... sphere approx
        // Mesh& mesh_org = shape->mesh();
        // CHECK(!mesh_org.cvxParts.N, "mesh '" <<shape->frame.name <<"' has convex decomposition - not implemented yet in FCL! -- please separate in separate frames")
        // // rai::Mesh& mesh = mesh_org;
        // rai::Mesh& mesh = cvx_mesh(i);
        // mesh.V = mesh_org.V;
        // mesh.makeConvexHull();
        if(!mesh.T.N) continue;
        mesh.computeTriNormals();
        std::shared_ptr<ConvexGeometryData> dat = make_shared<ConvexGeometryData>();
        dat->plane_dis = mesh.computeTriDistances();
        copy<int>(dat->polygons, mesh.T);
        dat->polygons.insColumns(0);
        for(uint i=0; i<dat->polygons.d0; i++) dat->polygons(i, 0) = 3;
#if FCL_MINOR_VERSION >= 7
        auto verts = make_shared<std::vector<fcl::Vector3<float>>>(mesh.V.d0);
        auto faces = make_shared<std::vector<int>>(mesh.T.N);
        for(uint i=0; i<verts->size(); i++)(*verts)[i] = {(float)mesh.V(i, 0), (float)mesh.V(i, 1), (float)mesh.V(i, 2)};
        for(uint i=0; i<faces->size(); i++)(*faces)[i] = mesh.T.elem(i);
        auto model = make_shared<fcl::Convex<float>>(verts, mesh.T.d0, faces, true);
#else
        geom = make_shared<fcl::Convex>((fcl::Vec3f*)mesh.Tn.p, dat->plane_dis.p, mesh.T.d0, (fcl::Vec3f*)mesh.V.p, mesh.V.d0, (int*)dat->polygons.p);
#endif
        self->convexGeometryData(i) = dat;
      }
      CollObject* obj = new CollObject(geom, fcl::Transform3f());
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

void FclInterface::setActiveColliders(uintA geom_ids){
  uint max_id = max(geom_ids);
  rai::Array<CollObject*> geomId2obj(max_id+1);
  geomId2obj.setZero();
  for(auto* obj:self->objects) {
    uint i = (long int)obj->getUserData();
    if(i<=max_id) geomId2obj(i) = obj;
  }
  for(uint i:geom_ids){
    if(geomId2obj(i)){
      self->activeColliders.append(geomId2obj(i));
    }
  }
  LOG(0) <<"#active colliders: " <<self->activeColliders.N <<endl;
}

void FclInterface::step(const arr& X) {
  CHECK_EQ(X.nd, 2, "");
  CHECK_GE(X.d0, self->convexGeometryData.N, "");
  CHECK_EQ(X.d1, 7, "");

  for(auto* obj:self->objects) {
    uint i = (long int)obj->getUserData();
    if(i<X_lastQuery.d0 && maxDiff(X_lastQuery[i], X[i])<1e-8) continue;
    double* T=&X(i, 0);
    obj->setTranslation(Vec3f(T[0], T[1], T[2]));
    obj->setQuatRotation(Quaternionf(T[3], T[4], T[5], T[6]));
    obj->computeAABB();
  }
  self->manager->update();

  collisions.clear();
  if(!self->activeColliders.N){ //collide all
    self->manager->collide(this, FclInterface_self::BroadphaseCallback);
  }else{
    for(auto* obj:self->activeColliders) {
      self->manager->collide(obj, this, FclInterface_self::BroadphaseCallback);
    }
  }

  collisions.reshape(-1, 2);

  X_lastQuery = X;
}

void FclInterface::addCollision(uint a, uint b) {
  collisions.resizeCopy(collisions.N+2);
  collisions.elem(-2) = a;
  collisions.elem(-1) = b;
}

bool FclInterface_self::BroadphaseCallback(CollObject* o1, CollObject* o2, void* cdata_) {
  FclInterface* fcl = static_cast<FclInterface*>(cdata_);

  uint a = (long int)o1->getUserData();
  uint b = (long int)o2->getUserData();
  if(a==b) return false;

  if(fcl->excludes.N) {
    if(a<b) {
      if(fcl->excludes(a).containsInSorted(b)) return false;
    } else {
      if(fcl->excludes(b).containsInSorted(a)) return false;
    }
  }

  if(fcl->mode==fcl->_broadPhaseOnly) {
    fcl->addCollision(a, b);
  } else if(fcl->mode==fcl->_binaryCollisionSingle || fcl->mode==fcl->_binaryCollisionAll) { //fine boolean collision query
    CollisionRequest request;
    CollisionResult result;
    fcl::collide(o1, o2, request, result);
    if(result.isCollision()) {
      fcl->addCollision(a, b);
      if(fcl->mode==fcl->_binaryCollisionSingle) return true; //can stop now
    }
  } else if(fcl->mode==fcl->_distanceCutoff) {
    CHECK(fcl->cutoff>=0., "")
    DistanceRequest request;
    DistanceResult result;
    fcl::distance(o1, o2, request, result);
    if(result.min_distance<fcl->cutoff) fcl->addCollision(a, b);
  } else {
    NIY;
  }
  return false;
}

}//namespace

#else //RAI_FCL
typedef int QueryMode;
rai::FclInterface::FclInterface(const Array<Shape*>& geometries, const uintAA& _excludes, QueryMode _mode){ NICO }
rai::FclInterface::~FclInterface() { NICO }
void rai::FclInterface::step(const arr& X) { NICO }
#endif


RUN_ON_INIT_BEGIN(fclInterface)
rai::Array<CollObject*>::memMove=true;
RUN_ON_INIT_END(fclInterface)
