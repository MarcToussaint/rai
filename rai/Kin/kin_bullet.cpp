#include "kin_bullet.h"

#ifdef RAI_BULLET

#include <Kin/frame.h>
#include <btBulletDynamicsCommon.h>

constexpr float gravity = -10.0f;
constexpr float initialY = 10.0f;
// TODO some combinations of coefficients smaller than 1.0
// make the ball go up higher / not lose height. Why?
constexpr float groundRestitution = 0.9f;
constexpr float sphereRestitution = 0.9f;
constexpr int maxNPoints = 500;

struct sBulletInterface{
  btDefaultCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  btBroadphaseInterface *overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld *dynamicsWorld;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  rai::Array<btRigidBody*> frameID_to_btBody;

  uint stepCount=0;
};

BulletInterface::BulletInterface(){
  self = new sBulletInterface;

  self->collisionConfiguration = new btDefaultCollisionConfiguration();
  self->dispatcher = new btCollisionDispatcher(self->collisionConfiguration);
  self->overlappingPairCache = new btDbvtBroadphase();
  self->solver = new btSequentialImpulseConstraintSolver;
  self->dynamicsWorld = new btDiscreteDynamicsWorld(self->dispatcher, self->overlappingPairCache, self->solver, self->collisionConfiguration);
  self->dynamicsWorld->setGravity(btVector3(0, 0, gravity));
}

BulletInterface::BulletInterface(const rai::KinematicWorld& K)
  : BulletInterface() {
  defaultInit(K);
}

BulletInterface::~BulletInterface(){
  for (int i = self->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = self->dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    self->dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }
  for (int i = 0; i < self->collisionShapes.size(); ++i) {
    delete self->collisionShapes[i];
  }
  delete self->dynamicsWorld;
  delete self->solver;
  delete self->overlappingPairCache;
  delete self->dispatcher;
  delete self->collisionConfiguration;
  self->collisionShapes.clear();
}

btRigidBody* BulletInterface::addGround(){
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0, 0, 0));
  btCollisionShape* groundShape;
  groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
  self->collisionShapes.push_back(groundShape);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* body = new btRigidBody(rbInfo);
  body->setRestitution(groundRestitution);
  self->dynamicsWorld->addRigidBody(body);
  return body;
}

btRigidBody* BulletInterface::addSphere(){
  btCollisionShape* colShape = new btSphereShape(btScalar(1.0));
  self->collisionShapes.push_back(colShape);
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, initialY));
  btVector3 localInertia(0, 0, 0);
  btScalar mass(1.0f);
  colShape->calculateLocalInertia(mass, localInertia);
  btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody *body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
                                        mass, myMotionState, colShape, localInertia));
  body->setRestitution(sphereRestitution);
  self->dynamicsWorld->addRigidBody(body);
  return body;
}

btRigidBody* BulletInterface::addFrame(const rai::Frame* f){
  CHECK(f->shape && f->shape->geom, "can only add frames with shapes");
  //create a colShape
  btCollisionShape* colShape = 0;
  arr& size = f->shape->geom->size;
  switch(f->shape->type()){
    case rai::ST_sphere:{
      double rad=1;
      if(size.N==1) rad=size(0);
      else rad=size(3);
      colShape =new btSphereShape(btScalar(rad));

    } break;
    case rai::ST_box:{
      colShape =new btBoxShape(btVector3(.5*size(0), .5*size(1), .5*size(2)));
    } break;
    default: HALT("NIY" <<f->shape->type());
  }
  self->collisionShapes.push_back(colShape);

  btTransform pose(btQuaternion(f->X.rot.x, f->X.rot.y, f->X.rot.z, f->X.rot.w),
                   btVector3(f->X.pos.x, f->X.pos.y, f->X.pos.z));
  btScalar mass(1.0f);
  if(f->inertia) mass = f->inertia->mass;
  btVector3 localInertia(0, 0, 0);
  colShape->calculateLocalInertia(mass, localInertia);
  btDefaultMotionState *motionState = new btDefaultMotionState(pose);
  btRigidBody *body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia));
  body->setRestitution(.9f);
  self->dynamicsWorld->addRigidBody(body);

  while(self->frameID_to_btBody.N<=f->ID) self->frameID_to_btBody.append(0);
  CHECK(!self->frameID_to_btBody(f->ID), "you already added a frame with ID" <<f->ID);
  self->frameID_to_btBody(f->ID) = body;
  return body;
}

void BulletInterface::addFrames(const FrameL& frames){
  for(const rai::Frame *f:frames) addFrame(f);
}

void BulletInterface::defaultInit(const rai::KinematicWorld& K){
  addGround();
  addFrames(K.frames);
}

void BulletInterface::step(double tau){
  self->stepCount++;
  self->dynamicsWorld->stepSimulation(tau);

  for (int j = self->dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; --j) {
    btCollisionObject *obj = self->dynamicsWorld->getCollisionObjectArray()[j];
    btRigidBody *body = btRigidBody::upcast(obj);
    btTransform trans;
    if (body && body->getMotionState()) {
      body->getMotionState()->getWorldTransform(trans);
    } else {
      trans = obj->getWorldTransform();
    }
    btVector3 origin = trans.getOrigin();
    if(j==1) std::cout <<self->stepCount <<' ' <<j <<' ' <<origin[2] <<std::endl;
  }
}

void BulletInterface::syncBack(FrameL& frames){
  for(uint i=0;i<self->frameID_to_btBody.N;i++){
    btRigidBody* b = self->frameID_to_btBody(i);
    rai::Frame *f = frames(i);
    if(f && b){
      btTransform trans;
      if (b && b->getMotionState()) {
        b->getMotionState()->getWorldTransform(trans);
      } else {
        NIY; //trans = obj->getWorldTransform();
      }
      btQuaternion q = trans.getRotation();
      btVector3& p = trans.getOrigin();
      f->X.pos.set(p.x(), p.y(), p.z());
      f->X.rot.set(q.w(), q.x(), q.y(), q.z());
    }
  }


}

#else
#endif
