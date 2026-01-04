/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "i_coal.h"

#ifdef RAI_COAL

#include <coal/config.hh>
#  include <coal/broadphase/broadphase.h>
#  include <coal/BVH/BVH_model.h>
#  include <coal/distance.h>
#  include <coal/collision.h>
#  include <coal/collision_data.h>

// typedef coal::CollisionObject CollisionObject;
// typedef coal::Vec3f Vec3f;
// typedef coal::Quaternion3f Quaternionf;
typedef coal::BroadPhaseCollisionManager BroadPhaseCollisionManager;
// typedef coal::DynamicAABBTreeCollisionManager DynamicAABBTreeCollisionManager;
// typedef coal::CollisionRequest CollisionRequest;
// typedef coal::CollisionResult CollisionResult;
// typedef coal::DistanceRequest DistanceRequest;
// typedef coal::DistanceResult DistanceResult;


namespace rai {

Transformation conv_Coal2Transformation(const coal::Transform3s& T) {
  Transformation X;
  NIY;
  // X.pos.set(pose.p.x, pose.p.y, pose.p.z);
  // X.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  return X;
}

coal::Transform3s conv_Transformation2Coal(const Transformation& t) {
  // coal::Transform3s T;
  // T.setTranslation(coal::Vec3s(t.pos.x, t.pos.y, t.pos.z));
  // T.setRotation(coal::Quats(t.rot.w, t.rot.x, t.rot.y, t.rot.z).toRotationMatrix() );
  // return T;
  return coal::Transform3s(coal::Quats(t.rot.w, t.rot.x, t.rot.y, t.rot.z), coal::Vec3s(t.pos.x, t.pos.y, t.pos.z));
}

//===========================================================================

struct ConvexGeometryData {
  arr plane_dis;
  intA polygons;
};

shared_ptr<coal::CollisionGeometry> create_CollisionGeometry(Shape* shape, bool from_ssc){
  shared_ptr<coal::CollisionGeometry> geom;
  if(from_ssc){

    if(shape->type()==ST_capsule) {
      geom = make_shared<coal::Capsule>(0., shape->size(-2));
    } else if(shape->type()==ST_sphere) {
      geom = make_shared<coal::Sphere>(0.);
    } else {
      rai::Mesh mesh;
      mesh.setConvex(shape->sscCore());
      if(mesh.T.N){
        std::shared_ptr<std::vector<coal::Vec3s>> verts = make_shared<std::vector<coal::Vec3s>>(mesh.V.d0);
        std::shared_ptr<std::vector<coal::Triangle32>> faces = make_shared<std::vector<coal::Triangle32>>(mesh.T.d0);
        for(uint i=0; i<verts->size(); i++) (*verts)[i] = {mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2)};
        for(uint i=0; i<faces->size(); i++) (*faces)[i] = {mesh.T(i, 0), mesh.T(i, 1), mesh.T(i, 2)};
        geom = make_shared<coal::ConvexTpl<coal::Triangle32>>(verts, mesh.V.d0, faces, mesh.T.d0);
      }else{
        LOG(-2) <<"this is not a proper cvx shape";
        geom = make_shared<coal::Sphere>(1.);
        // geom = make_shared<coal::Capsule>(0., shape->size(-2));
      }
    }

  }else{

    if(shape->type()==ST_capsule) {
      geom = make_shared<coal::Capsule>(shape->size(-1), shape->size(-2));
    } else if(shape->type()==ST_cylinder) {
      geom = make_shared<coal::Cylinder>(shape->size(-1), shape->size(-2));
    } else if(shape->type()==ST_sphere) {
      geom = make_shared<coal::Sphere>(shape->size(-1));
    }else if(shape->type()==ST_box){
      geom = make_shared<coal::Box>(shape->size(0), shape->size(1), shape->size(2));
    } else {
      CHECK(shape->sscCore().N, "for FCL broadphase, every shape with 'contact' enabled needs a convex core"); // <<shape->frame.name);
      CHECK_EQ(shape->radius(), shape->coll_cvxRadius, "should be equal");
      // rai::Array<rai::Mesh> cvx_meshes(geometries.N);
      rai::Mesh mesh; // = cvx_meshes(i);
      mesh.setSSCvx(shape->sscCore(), shape->coll_cvxRadius*1.05, 1); //make it a little larger... sphere approx
      // Mesh& mesh_org = shape->mesh();
      // CHECK(!mesh_org.cvxParts.N, "mesh '" <<shape->frame.name <<"' has convex decomposition - not implemented yet in FCL! -- please separate in separate frames")
      // // rai::Mesh& mesh = mesh_org;
      // rai::Mesh& mesh = cvx_mesh(i);
      // mesh.V = mesh_org.V;
      // mesh.makeConvexHull();
      if(mesh.T.N){
        // mesh.computeTriNormals();
        std::shared_ptr<std::vector<coal::Vec3s>> verts = make_shared<std::vector<coal::Vec3s>>(mesh.V.d0);
        std::shared_ptr<std::vector<coal::Triangle32>> faces = make_shared<std::vector<coal::Triangle32>>(mesh.T.d0);
        for(uint i=0; i<verts->size(); i++) (*verts)[i] = {mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2)};
        for(uint i=0; i<faces->size(); i++) (*faces)[i] = {mesh.T(i, 0), mesh.T(i, 1), mesh.T(i, 2)};
        geom = make_shared<coal::ConvexTpl<coal::Triangle32>>(verts, mesh.V.d0, faces, mesh.T.d0);
      }
    }
  }

  return geom;
}

struct CoalInterface_self : coal::CollisionCallBackBase{
  CoalInterface* coal = 0;
  CollisionQueryMode mode;

  Array<shared_ptr<struct ConvexGeometryData>> convexGeometryData;
  std::vector<coal::CollisionObject*> objects;
  rai::Array<coal::CollisionObject*> activeColliders;
  shared_ptr<BroadPhaseCollisionManager> manager;

  CoalInterface_self(CoalInterface *_coal) : coal(_coal) {}
  virtual ~CoalInterface_self() {}

  virtual bool collide(coal::CollisionObject* o1, coal::CollisionObject* o2);
};

CoalInterface::CoalInterface(const Array<Shape*>& geometries, const uintAA& _excludes)
  : excludes(_excludes) {
  self = new CoalInterface_self(this);

  self->convexGeometryData.resize(geometries.N);

  for(long int i=0; i<geometries.N; i++) {
    Shape* shape = geometries(i);
    if(shape) {
      // CHECK(!shape->geom, "collision geometry for this shape was already created");
      auto geom = create_CollisionGeometry(shape, false);
      if(geom){
        coal::CollisionObject* obj = new coal::CollisionObject(geom, coal::Transform3s());
        obj->setUserData((void*)(i));
        self->objects.push_back(obj);
      }
    }
  }

//  manager = make_shared<coal::IntervalTreeCollisionManager>();
  self->manager = make_shared<coal::DynamicAABBTreeCollisionManager>();
//  manager = make_shared<coal::SaPCollisionManager>();
//  manager = make_shared<coal::NaiveCollisionManager>();
//  manager = new coal::SpatialHashingCollisionManager();
  self->manager->registerObjects(self->objects);
  self->manager->setup();
}

CoalInterface::~CoalInterface() {
  for(size_t i = 0; i < self->objects.size(); ++i)
    delete self->objects[i];
  delete self;
}

void CoalInterface::setActiveColliders(uintA geom_ids){
  uint max_id = max(geom_ids);
  rai::Array<coal::CollisionObject*> geomId2obj(max_id+1);
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

void CoalInterface::step(const arr& X, CollisionQueryMode mode) {
  CHECK_EQ(X.nd, 2, "");
  CHECK_GE(X.d0, self->convexGeometryData.N, "");
  CHECK_EQ(X.d1, 7, "");

  for(auto* obj:self->objects) {
    uint i = (long int)obj->getUserData();
    if(i<X_lastQuery.d0 && maxDiff(X_lastQuery[i], X[i])<1e-8) continue;
    double* T=&X(i, 0);
    obj->setTranslation(coal::Vec3s(T[0], T[1], T[2]));
    obj->setRotation(coal::Quats(T[3], T[4], T[5], T[6]).toRotationMatrix() );
    obj->computeAABB();
  }
  self->manager->update();

  collisions.clear();
  self->mode = mode;
  if(!self->activeColliders.N){ //collide all
    self->manager->collide(self);
  }else{
    for(auto* obj:self->activeColliders) {
      self->manager->collide(obj, self);
    }
  }

  X_lastQuery = X;
}

Proxy& CoalInterface::addCollision(uint a, uint b) {
  Proxy& p = collisions.append();
  p.A = a;
  p.B = b;
  return p;
}

bool CoalInterface_self::collide(coal::CollisionObject* o1, coal::CollisionObject* o2) {
  uint a = (long int)o1->getUserData();
  uint b = (long int)o2->getUserData();
  if(a==b) return false;

  if(coal->excludes.N) {
    if(a<b) {
      if(coal->excludes(a).containsInSorted(b)) return false;
    } else {
      if(coal->excludes(b).containsInSorted(a)) return false;
    }
  }

  if(mode==_broadPhaseOnly) {
    Proxy& p = coal->addCollision(a, b);
    p.d = -0.;
    p.posA.set( o1->getTranslation().data() );
    p.posB.set( o2->getTranslation().data() );

  } else if(mode==_binaryCollisionSingle || mode==_binaryCollisionAll) { //fine boolean collision query
    coal::CollisionRequest request;
    coal::CollisionResult result;
    coal::collide(o1, o2, request, result);
    if(result.isCollision()) {
      coal->addCollision(a, b);
      if(mode==_binaryCollisionSingle) return true; //can stop now
    }

  } else if(mode==_distanceCutoff) {
    CHECK(coal->cutoff>=0., "")
    coal::DistanceRequest request;
    coal::DistanceResult result;
    coal::distance(o1, o2, request, result);
    if(result.min_distance<coal->cutoff) coal->addCollision(a, b);

  } else {
    coal::CollisionRequest request;
    coal::CollisionResult result;
    coal::collide(o1, o2, request, result);
    // cout <<"-- collision " <<a <<' ' <<b << " dist: " <<result.distance_lower_bound <<endl;
    // cout <<"   witness points: " <<result.nearest_points[0] << ' '<<result.nearest_points[1] <<endl;
    // cout <<"   normal: " <<result.normal <<endl;
    // if(result.isCollision()) {
    Proxy& p = coal->addCollision(a, b);
    p.d = result.distance_lower_bound;
    p.posA.set( result.nearest_points[0].data() );
    p.posB.set( result.nearest_points[1].data() );
    p.normal.set( result.normal.data() );
      // if(fcl->mode==fcl->_binaryCollisionSingle) return true; //can stop now
    // }
  }
  return false;
}

uintA support(const arr& V, const arr& dir, double eps=1e-8) {
  arr q = V*dir;
  q -= max(q);
  uintA M;
  for(uint i=0;i<q.N;i++) if(q.p[i]>-eps) M.append(i);
  return M;
}

PairCollision_Coal::PairCollision_Coal(Shape* s1, Shape* s2, const Transformation& t1, const Transformation& t2, double _rad1, double _rad2){
  rad1 = _rad1;
  rad2 = _rad2;

  coal::Transform3s T1 = conv_Transformation2Coal(t1);
  coal::Transform3s T2 = conv_Transformation2Coal(t2);

  if(!s1->geom) s1->geom = create_CollisionGeometry(s1, true);
  if(!s2->geom) s2->geom = create_CollisionGeometry(s2, true);
  if(!s1->geom){ LOG(0) <<"s1 is not a proper shape"; distance = 1e10; return; }
  if(!s2->geom){ LOG(0) <<"s2 is not a proper shape"; distance = 1e10; return; }

  coal::CollisionRequest request;
  coal::CollisionResult result;
  coal::collide(s1->geom.get(), T1, s2->geom.get(), T2, request, result);

  distance=result.distance_lower_bound;
  p1 = conv_eigen2arr(result.nearest_points[0]);
  p2 = conv_eigen2arr(result.nearest_points[1]);
  normal = -conv_eigen2arr(result.normal);

  // cout <<"coal d: " <<distance <<" |p1-p2|: " <<length(p1-p2) <<endl;

  if(s1->_sscCore && s2->_sscCore){
    {
      arr& V = *s1->_sscCore;
      arr dir = ~t1.rot.getMatrix() * normal;
      uintA M = support(V, -dir, 1e-8);
      for(uint i:M) simp1.append(V[i]);
      simp1.reshape(-1,3);
      t1.applyOnPointArray(simp1);
    }
    {
      arr& V = *s2->_sscCore;
      arr dir = ~t2.rot.getMatrix() * normal;
      uintA M = support(V, dir, 1e-8);
      for(uint i:M) simp2.append(V[i]);
      simp2.reshape(-1,3);
      t2.applyOnPointArray(simp2);
    }
  }else{
    NIY;
  }
}

}//namespace

RUN_ON_INIT_BEGIN(CoalInterface)
rai::Array<coal::CollisionObject*>::memMove=true;
RUN_ON_INIT_END(CoalInterface)

#else //RAI_COAL
rai::CoalInterface::CoalInterface(const Array<Shape*>& geometries, const uintAA& _excludes){ NICO }
rai::CoalInterface::~CoalInterface() { NICO }
void rai::CoalInterface::step(const arr& X, CollisionQueryMode mode) { NICO }
void rai::CoalInterface::setActiveColliders(uintA geom_ids){ NICO }
#endif

