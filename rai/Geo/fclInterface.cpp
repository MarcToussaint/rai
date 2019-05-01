#include "fclInterface.h"

#ifdef RAI_FCL

#include <fcl/broadphase/broadphase.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>

bool FclInterfaceBroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_);

rai::FclInterface::FclInterface(const rai::Array<ptr<Mesh> >& _geometries, double _cutoff)
  : geometries(_geometries), cutoff(_cutoff){
  for(long int i=0;i<geometries.N;i++){
    if(geometries(i)){
      rai::Mesh& mesh = *geometries(i);
      fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>();
      model->beginModel();
      for(uint i=0;i<mesh.T.d0;i++)
        model->addTriangle( fcl::Vec3f(&mesh.V(mesh.T(i,0),0)), fcl::Vec3f(&mesh.V(mesh.T(i,1),0)), fcl::Vec3f(&mesh.V(mesh.T(i,2),0)) );
      model->endModel();
      model->setUserData((void*)(i));
      fcl::CollisionObject* obj = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(model), fcl::Transform3f());
      obj->setUserData((void*)(i));
      objects.push_back(obj);
    }
  }

  manager = new fcl::DynamicAABBTreeCollisionManager();
  manager->registerObjects(objects);
  manager->setup();
}

rai::FclInterface::~FclInterface(){
  for(size_t i = 0; i < objects.size(); ++i)
    delete objects[i];
  delete manager;
}

void rai::FclInterface::step(const arr& X){
  CHECK_EQ(X.nd, 2, "");
  CHECK_EQ(X.d0, geometries.size(), "");
  CHECK_EQ(X.d1, 7, "");

  for(auto *obj:objects){
    uint i = (long int)obj->getUserData();
    obj->setTranslation(fcl::Vec3f(X(i,0), X(i,1), X(i,2)));
    obj->setQuatRotation(fcl::Quaternion3f(X(i,3), X(i,4), X(i,5), X(i,6)));
    obj->computeAABB();
  }
  manager->update();

  collisions.clear();
  manager->collide(this, FclInterfaceBroadphaseCallback);
  collisions.reshape(collisions.N/2,2);
}

bool FclInterfaceBroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_){
  rai::FclInterface* self = static_cast<rai::FclInterface*>(cdata_);

  if(self->cutoff==0.){
    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::collide(o1, o2, request, result);
    if(result.isCollision()){
      self->collisions.append(TUP( (long int)o1->getUserData(), (long int)o2->getUserData() ));
    }
  }else if(self->cutoff>0.){
    fcl::DistanceRequest request;
    fcl::DistanceResult result;
    double d = fcl::distance(o1, o2, request, result);
    if(d<self->cutoff){
      self->collisions.append(TUP( (long int)o1->getUserData(), (long int)o2->getUserData() ));
    }
  }else{
    self->collisions.append(TUP( (long int)o1->getUserData(), (long int)o2->getUserData() ));
  }
  return false;
}


#else //RAI_FCL
rai::FclInterface::FclInterface(const Array<ptr<Mesh> >& _geometries, double _cutoff){ NICO }
rai::FclInterface::~FclInterface(){ NICO }
  void rai::FclInterface::step(const arr& X){ NICO }
#endif
