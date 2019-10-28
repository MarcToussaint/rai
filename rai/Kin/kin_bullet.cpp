#include "kin_bullet.h"

#ifdef RAI_BULLET

#include <Kin/frame.h>
#include <btBulletDynamicsCommon.h>

constexpr float gravity = -10.0f;
constexpr float groundRestitution = 0.1f;
constexpr float objectRestitution = 0.1f;

struct BulletInterface_self{
  btDefaultCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  btBroadphaseInterface *overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld *dynamicsWorld;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  rai::Array<btRigidBody*> actors;
  rai::Array<rai::BodyType> actorTypes;

  uint stepCount=0;
};

BulletInterface::BulletInterface(bool verbose){
  self = new BulletInterface_self;

  if(verbose) LOG(0) <<"starting bullet engine ...";

  self->collisionConfiguration = new btDefaultCollisionConfiguration();
  self->dispatcher = new btCollisionDispatcher(self->collisionConfiguration);
  self->overlappingPairCache = new btDbvtBroadphase();
  self->solver = new btSequentialImpulseConstraintSolver;
  self->dynamicsWorld = new btDiscreteDynamicsWorld(self->dispatcher, self->overlappingPairCache, self->solver, self->collisionConfiguration);
  self->dynamicsWorld->setGravity(btVector3(0, 0, gravity));

  if(verbose) LOG(0) <<"... done starting bullet engine";
}

BulletInterface::BulletInterface(rai::Configuration& K, bool verbose)
  : BulletInterface() {
  defaultInit(K, verbose);
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

btRigidBody* BulletInterface::addFrame(rai::Frame* f, bool verbose){
  FrameL parts = {f};
  f->getRigidSubFrames(parts);
  bool hasShape=false;
  for(rai::Frame *p:parts) if(p->shape && p->getShape().type()!=rai::ST_marker){ hasShape=true; break; }

  rai::BodyType type = rai::BT_dynamic;
  if(hasShape){
    if(f->joint)   type = rai::BT_kinematic;
    if(f->inertia) type = f->inertia->type;
  }
  self->actorTypes(f->ID) = type;

  if(verbose) LOG(0) <<"adding link anchored at '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type);

  //-- create a bullet collision shape
  CHECK(f->shape && f->shape->_mesh, "can only add frames with meshes");
  btCollisionShape* colShape = 0;
  arr& size = f->shape->size;
  switch(f->shape->type()){
    case rai::ST_sphere:{
      colShape =new btSphereShape(btScalar(size.last()));
    } break;
    case rai::ST_box:{
      colShape =new btBoxShape(btVector3(.5*size(0), .5*size(1), .5*size(2)));
    } break;
    case rai::ST_ssBox:
    case rai::ST_ssCvx:{
#ifdef BT_USE_DOUBLE_PRECISION
      arr& V = f->shape->sscCore().V;
#else
      floatA V = convert<float>(f->shape->sscCore().V);
#endif
      colShape = new btConvexHullShape(V.p, V.d0, V.sizeT*V.d1);
      colShape->setMargin(f->shape->radius()+.01);
    } break;
    case rai::ST_mesh:{
#ifdef BT_USE_DOUBLE_PRECISION
      arr& V = f->shape->mesh().V;
#else
      floatA V = convert<float>(f->shape->mesh().V);
#endif
      colShape = new btConvexHullShape(V.p, V.d0, V.sizeT*V.d1);
    } break;
    default: HALT("NIY" <<f->shape->type());
  }
  self->collisionShapes.push_back(colShape);

  if(verbose) LOG(0) <<"adding link anchored at '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type);

  //-- create a bullet body
  rai::Transformation fX = f->get_X();
  btTransform pose(btQuaternion(fX.rot.x, fX.rot.y, fX.rot.z, fX.rot.w),
                   btVector3(fX.pos.x, fX.pos.y, fX.pos.z));
  btScalar mass(1.0f);
  btVector3 localInertia(0, 0, 0);
  if(type==rai::BT_dynamic){
    if(f->inertia) mass = f->inertia->mass;
    colShape->calculateLocalInertia(mass, localInertia);
  }else{
    mass=0.;
  }

  btDefaultMotionState *motionState = new btDefaultMotionState(pose);
  btRigidBody *body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia));
  body->setRestitution(objectRestitution);
  self->dynamicsWorld->addRigidBody(body);

  if(type==rai::BT_kinematic){
    body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
  }

  while(self->actors.N<=f->ID) self->actors.append(0);
  CHECK(!self->actors(f->ID), "you already added a frame with ID" <<f->ID);
  self->actors(f->ID) = body;
  return body;
}

void BulletInterface::addFrames(FrameL& frames, bool verbose){
  for(rai::Frame *f:frames) addFrame(f, verbose);
}

void BulletInterface::defaultInit(rai::Configuration& C, bool verbose){
  addGround();
  if(verbose) LOG(0) <<"creating Configuration within bullet ...";

  self->actors.resize(C.frames.N); self->actors.setZero();
  self->actorTypes.resize(C.frames.N); self->actorTypes.setZero();
  FrameL links = C.getLinks();
  addFrames(links, verbose);

  if(verbose) LOG(0) <<"... done creating Configuration within bullet";
}

void BulletInterface::step(double tau){
  self->stepCount++;
  self->dynamicsWorld->stepSimulation(tau);

//  for (int j = self->dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; --j) {
//    btCollisionObject *obj = self->dynamicsWorld->getCollisionObjectArray()[j];
//    btRigidBody *body = btRigidBody::upcast(obj);
//    btTransform trans;
//    if (body && body->getMotionState()) {
//      body->getMotionState()->getWorldTransform(trans);
//    } else {
//      trans = obj->getWorldTransform();
//    }
//    btVector3 origin = trans.getOrigin();
//    if(j==1) std::cout <<self->stepCount <<' ' <<j <<' ' <<origin[2] <<std::endl;
//  }
}

void BulletInterface::pushFullState(const FrameL& frames, const arr& vel){
  for(uint i=0;i<self->actors.N;i++){
    btRigidBody* b = self->actors(i);
    const rai::Frame *f = frames(i);
    if(f && b){
      rai::Transformation fX = f->get_X();
      btTransform pose(btQuaternion(fX.rot.x, fX.rot.y, fX.rot.z, fX.rot.w),
                       btVector3(fX.pos.x, fX.pos.y, fX.pos.z));
      b->setWorldTransform(pose);
      b->clearForces();
      b->setActivationState(ACTIVE_TAG);
      if(vel.N){
        b->setLinearVelocity(btVector3(vel(i,0),vel(i,1),vel(i,2)));
        b->setAngularVelocity(btVector3(vel(i,3),vel(i,4),vel(i,5)));
      }
    }
  }
}

void BulletInterface::pushKinematicStates(const FrameL& frames){
  for(uint i=0;i<self->actors.N;i++){
    btRigidBody* b = self->actors(i);
    rai::Frame *f = frames(i);
    if(f && b){
      rai::BodyType type = rai::BT_dynamic;
      if(f->joint) type = rai::BT_kinematic;
      if(f->inertia) type = f->inertia->type;

      if(type==rai::BT_kinematic){
        rai::Transformation fX = f->get_X();
        btTransform pose(btQuaternion(fX.rot.x, fX.rot.y, fX.rot.z, fX.rot.w),
                         btVector3(fX.pos.x, fX.pos.y, fX.pos.z));
        if(b->getMotionState()) {
          b->getMotionState()->setWorldTransform(pose);
        } else {
          NIY; //trans = obj->getWorldTransform();
        }
      }
    }
  }
}

void BulletInterface::pullDynamicStates(FrameL& frames, arr& vel){
  if(!!vel) vel.resize(frames.N,6).setZero();

  for(uint i=0;i<self->actors.N;i++){
    btRigidBody* b = self->actors(i);
    rai::Frame *f = frames(i);
    if(f && b){
      rai::BodyType type = rai::BT_dynamic;
      if(f->joint) type = rai::BT_kinematic;
      if(f->inertia) type = f->inertia->type;

      if(type==rai::BT_dynamic){
        btTransform pose;
        if (b && b->getMotionState()) {
          b->getMotionState()->getWorldTransform(pose);
        } else {
          NIY; //trans = obj->getWorldTransform();
        }
        const btQuaternion q = pose.getRotation();
        const btVector3& p = pose.getOrigin();
        f->setPosition({p.x(), p.y(), p.z()});
        f->setQuaternion({q.w(), q.x(), q.y(), q.z()});

        if(!!vel){
          const btVector3& v = b->getLinearVelocity();
          const btVector3& w = b->getAngularVelocity();
          vel[i] = ARR(v.x(), v.y(), v.z(), w.x(), w.y(), w.z());
        }
      }
    }
  }
}

void BulletInterface::saveBulletFile(const char* filename){
  //adapted from PhysicsServerCommandProcessor::processSaveBulletCommand

  FILE* f = fopen(filename, "wb");
  if (f){
    btDefaultSerializer* ser = new btDefaultSerializer();
    int currentFlags = ser->getSerializationFlags();
    ser->setSerializationFlags(currentFlags | BT_SERIALIZE_CONTACT_MANIFOLDS);

    self->dynamicsWorld->serialize(ser);
    fwrite(ser->getBufferPointer(), ser->getCurrentBufferSize(), 1, f);
    fclose(f);
    delete ser;
  }else{
    HALT("could not open file '" <<filename <<"' for writing");
  }
}

#else

BulletInterface::BulletInterface(bool verbose){ NICO }
BulletInterface::BulletInterface(rai::Configuration& K, bool verbose){ NICO }
BulletInterface::~BulletInterface(){ NICO }
btRigidBody* BulletInterface::addGround(){ NICO }
btRigidBody* BulletInterface::addFrame(rai::Frame* f, bool verbose){ NICO }
void BulletInterface::addFrames(FrameL& frames, bool verbose){ NICO }
void BulletInterface::defaultInit(rai::Configuration& K, bool verbose){ NICO }
void BulletInterface::step(double tau){ NICO }
void BulletInterface::pushFullState(const FrameL& frames, const arr& vel){ NICO }
void BulletInterface::pushKinematicStates(const FrameL& frames){ NICO }
void BulletInterface::pullDynamicStates(FrameL& frames, arr& vel){ NICO }
void BulletInterface::saveBulletFile(const char* filename){ NICO }

#endif

#ifdef RAI_BULLET
RUN_ON_INIT_BEGIN(kin_bullet)
rai::Array<btRigidBody*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_bullet)
#endif
