/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_bullet.h"

#ifdef RAI_BULLET

#include "frame.h"
#include <btBulletDynamicsCommon.h>

// ============================================================================

constexpr float gravity = -9.8f;
constexpr float groundRestitution = .1f;
constexpr float objectRestitution = .1f;

// ============================================================================

void btTrans2raiTrans(rai::Transformation& f, const btTransform& pose) {
  const btQuaternion q = pose.getRotation();
  const btVector3& p = pose.getOrigin();
  f.pos.set(p.x(), p.y(), p.z());
  f.rot.set(q.w(), q.x(), q.y(), q.z());
}

btTransform conv_raiTrans2btTrans(const rai::Transformation& fX) {
  btTransform pose(btQuaternion(fX.rot.x, fX.rot.y, fX.rot.z, fX.rot.w),
                   btVector3(fX.pos.x, fX.pos.y, fX.pos.z));
  return pose;
}

arr conv_btVec3_arr(const btVector3& v) {
  return ARR(v.x(), v.y(), v.z());
}

// ============================================================================

struct BulletInterface_self {
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  rai::Array<btRigidBody*> actors;
  rai::Array<rai::BodyType> actorTypes;

  uint stepCount=0;

  btRigidBody* addGround();
  btRigidBody* addLink(rai::Frame* f, int verbose);

  btCollisionShape* createCollisionShape(rai::Shape* s);
  btCollisionShape* createCompoundCollisionShape(rai::Frame* link, ShapeL& shapes);
};

// ============================================================================

BulletInterface::BulletInterface(rai::Configuration& C, int verbose) : self(nullptr) {
  self = new BulletInterface_self;

  if(verbose>0) LOG(0) <<"starting bullet engine ...";

  self->collisionConfiguration = new btDefaultCollisionConfiguration();
  self->dispatcher = new btCollisionDispatcher(self->collisionConfiguration);
  self->overlappingPairCache = new btDbvtBroadphase();
  self->solver = new btSequentialImpulseConstraintSolver;
  self->dynamicsWorld = new btDiscreteDynamicsWorld(self->dispatcher, self->overlappingPairCache, self->solver, self->collisionConfiguration);
  self->dynamicsWorld->setGravity(btVector3(0, 0, gravity));

  if(verbose>0) LOG(0) <<"... done starting bullet engine";

  self->addGround();

  if(verbose>0) LOG(0) <<"creating Configuration within bullet ...";

  self->actors.resize(C.frames.N); self->actors.setZero();
  self->actorTypes.resize(C.frames.N); self->actorTypes.setZero();
  FrameL links = C.getLinks();
  for(rai::Frame* a : links) self->addLink(a, verbose);

  if(verbose>0) LOG(0) <<"... done creating Configuration within bullet";
}

BulletInterface::~BulletInterface() {
  for(int i = self->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = self->dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if(body && body->getMotionState()) {
      delete body->getMotionState();
    }
    self->dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }
  for(int i = 0; i < self->collisionShapes.size(); ++i) {
    delete self->collisionShapes[i];
  }
  delete self->dynamicsWorld;
  delete self->solver;
  delete self->overlappingPairCache;
  delete self->dispatcher;
  delete self->collisionConfiguration;
  self->collisionShapes.clear();
}

void BulletInterface::step(double tau) {
  self->stepCount++;
  self->dynamicsWorld->stepSimulation(tau);
}

void BulletInterface::pullDynamicStates(FrameL& frames, arr& frameVelocities) {
  if(!!frameVelocities) frameVelocities.resize(frames.N, 2, 3).setZero();

  for(rai::Frame* f : frames) {
    if(self->actors.N <= f->ID) continue;
    btRigidBody* b = self->actors(f->ID);
    if(!b) continue;

    if(self->actorTypes(f->ID) == rai::BT_dynamic) {
      rai::Transformation X;
      btTransform pose;
      if(b->getMotionState()) {
        b->getMotionState()->getWorldTransform(pose);
      } else {
        NIY; //trans = obj->getWorldTransform();
      }
      btTrans2raiTrans(X, pose);
      f->set_X() = X;
      if(!!frameVelocities) {
        frameVelocities(f->ID, 0, {}) = conv_btVec3_arr(b->getLinearVelocity());
        frameVelocities(f->ID, 1, {}) = conv_btVec3_arr(b->getAngularVelocity());
      }
    }
  }
}

void BulletInterface::changeObjectType(rai::Frame* f, int _type) {
  rai::Enum<rai::BodyType> type((rai::BodyType)_type);
  if(self->actorTypes(f->ID) == type) {
    LOG(-1) <<"frame " <<*f <<" is already of type " <<type;
  }

  btRigidBody* a = self->actors(f->ID);
  if(!a) HALT("frame " <<*f <<"is not an actor");

  if(type==rai::BT_kinematic) {
    a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    a->setActivationState(DISABLE_DEACTIVATION);
  } else if(type==rai::BT_dynamic) {
    a->setCollisionFlags(a->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    a->setActivationState(DISABLE_DEACTIVATION);
  } else NIY;
  self->actorTypes(f->ID) = type;
}

void BulletInterface::pushKinematicStates(const FrameL& frames) {

  for(rai::Frame* f: frames) {
    if(self->actors.N <= f->ID) continue;
    if(self->actorTypes(f->ID)==rai::BT_kinematic) {
      btRigidBody* b = self->actors(f->ID);
      if(!b) continue; //f is not an actor

      CHECK(b->getMotionState(), "");
      b->getMotionState()->setWorldTransform(conv_raiTrans2btTrans(f->ensure_X()));
    }
  }
}

void BulletInterface::pushFullState(const FrameL& frames, const arr& frameVelocities) {
  for(rai::Frame* f : frames) {
    if(self->actors.N <= f->ID) continue;
    btRigidBody* b = self->actors(f->ID);
    if(!b) continue; //f is not an actor

    b->setWorldTransform(conv_raiTrans2btTrans(f->ensure_X()));
    b->setActivationState(ACTIVE_TAG);
    if(self->actorTypes(f->ID)==rai::BT_dynamic) {
      b->clearForces();
      if(!!frameVelocities && frameVelocities.N) {
        b->setLinearVelocity(btVector3(frameVelocities(f->ID, 0, 0), frameVelocities(f->ID, 0, 1), frameVelocities(f->ID, 0, 2)));
        b->setAngularVelocity(btVector3(frameVelocities(f->ID, 1, 0), frameVelocities(f->ID, 1, 1), frameVelocities(f->ID, 1, 2)));
      } else {
        b->setLinearVelocity(btVector3(0., 0., 0.));
        b->setAngularVelocity(btVector3(0., 0., 0.));
      }
    }
  }
  self->dynamicsWorld->stepSimulation(.01); //without this, two consequtive pushFullState won't work! (something active tag?)
}

btRigidBody* BulletInterface_self::addGround() {
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0, 0, 0));
  btCollisionShape* groundShape;
  groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
  collisionShapes.push_back(groundShape);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* body = new btRigidBody(rbInfo);
  body->setRestitution(groundRestitution);
  dynamicsWorld->addRigidBody(body);
  return body;
}

btRigidBody* BulletInterface_self::addLink(rai::Frame* f, int verbose) {
  //-- collect all shapes of that link
  FrameL parts = {f};
  f->getRigidSubFrames(parts);
  ShapeL shapes;
  for(rai::Frame* p: parts) if(p->shape && p->getShape().type()!=rai::ST_marker) shapes.append(p->shape);

  //-- decide on the type
  rai::BodyType type = rai::BT_static;
  if(shapes.N) {
    if(f->joint)   type = rai::BT_kinematic;
    if(f->inertia) type = f->inertia->type;
  }
  actorTypes(f->ID) = type;
  if(verbose>0) LOG(0) <<"adding link anchored at '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type);

  //-- create a bullet collision shape
  btCollisionShape* colShape = 0;
  if(shapes.N==1 && f == &shapes.scalar()->frame) {
    colShape = createCollisionShape(shapes.scalar());
  } else {
    colShape = createCompoundCollisionShape(f, shapes);
  }
  collisionShapes.push_back(colShape);

  //-- create a bullet body
  btTransform pose = conv_raiTrans2btTrans(f->ensure_X());
  btScalar mass(1.0f);
  btVector3 localInertia(0, 0, 0);
  if(type==rai::BT_dynamic) {
    if(f->inertia) mass = f->inertia->mass;
    colShape->calculateLocalInertia(mass, localInertia);
  } else {
    mass=0.;
  }

  btDefaultMotionState* motionState = new btDefaultMotionState(pose);
  btRigidBody* body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia));

  double fric=1.;
  if(shapes.N==1 && f == &shapes.scalar()->frame) {
    //try to read friction from attributes
    shapes.scalar()->frame.ats.get<double>(fric, "friction");
  }
  body->setFriction(fric);
  body->setRollingFriction(.01);
  body->setSpinningFriction(.01);
//  cout <<body->getContactStiffness() <<endl;
//  cout <<body->getContactDamping() <<endl;
//  body->setContactStiffnessAndDamping(1e4, 1e-1);
//  body->setContactStiffnessAndDamping(1e7, 3e4);
  body->setRestitution(objectRestitution);
  dynamicsWorld->addRigidBody(body);

  if(type==rai::BT_kinematic) {
    body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
  }

  while(actors.N<=f->ID) actors.append(0);
  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = body;
  return body;
}

void BulletInterface::saveBulletFile(const char* filename) {
  //adapted from PhysicsServerCommandProcessor::processSaveBulletCommand

  FILE* f = fopen(filename, "wb");
  if(f) {
    btDefaultSerializer* ser = new btDefaultSerializer();
    int currentFlags = ser->getSerializationFlags();
    ser->setSerializationFlags(currentFlags); // | BT_SERIALIZE_CONTACT_MANIFOLDS);

    self->dynamicsWorld->serialize(ser);
    fwrite(ser->getBufferPointer(), ser->getCurrentBufferSize(), 1, f);
    fclose(f);
    delete ser;
  } else {
    HALT("could not open file '" <<filename <<"' for writing");
  }
}

btCollisionShape* BulletInterface_self::createCollisionShape(rai::Shape* s) {
  btCollisionShape* colShape=0;
  arr& size = s->size;
  switch(s->type()) {
    case rai::ST_sphere: {
      colShape = new btSphereShape(btScalar(s->radius()));
    } break;
    case rai::ST_box: {
      colShape = new btBoxShape(btVector3(.5*size(0), .5*size(1), .5*size(2)));
    } break;
//    case rai::ST_capsule: {
//      colShape = new btCapsuleShape(btScalar(s->radius()), btScalar(size(0)));
//    } break;
    case rai::ST_capsule:
    case rai::ST_cylinder:
    case rai::ST_ssBox:
    case rai::ST_ssCvx:
//    {
//#ifdef BT_USE_DOUBLE_PRECISION
//      arr& V = s->sscCore().V;
//#else
//      floatA V = convert<float>(s->sscCore().V);
//#endif
//      colShape = new btConvexHullShape(V.p, V.d0, V.sizeT*V.d1);
//      colShape->setMargin(s->radius());
//    } break;
    case rai::ST_mesh: {
#ifdef BT_USE_DOUBLE_PRECISION
      arr& V = s->mesh().V;
#else
      floatA V = convert<float>(s->mesh().V);
#endif
      colShape = new btConvexHullShape(V.p, V.d0, V.sizeT*V.d1);
      colShape->setMargin(0.);
    } break;
    default: HALT("NIY" <<s->type());
  }
  return colShape;
}

btCollisionShape* BulletInterface_self::createCompoundCollisionShape(rai::Frame* link, ShapeL& shapes) {
  btCompoundShape* colShape = new btCompoundShape;
  for(rai::Shape* s:shapes) {
    colShape->addChildShape(conv_raiTrans2btTrans(s->frame.ensure_X()/link->ensure_X()), createCollisionShape(s));
  }
  return colShape;
}

#else

BulletInterface::BulletInterface(rai::Configuration& K, int verbose) { NICO }
BulletInterface::~BulletInterface() { NICO }
void BulletInterface::step(double tau) { NICO }
void BulletInterface::pushFullState(const FrameL& frames, const arr& vel) { NICO }
void BulletInterface::pushKinematicStates(const FrameL& frames) { NICO }
void BulletInterface::pullDynamicStates(FrameL& frames, arr& vel) { NICO }
void BulletInterface::saveBulletFile(const char* filename) { NICO }
void BulletInterface::changeObjectType(rai::Frame* f, int _type) { NICO }

#endif

#ifdef RAI_BULLET
RUN_ON_INIT_BEGIN(kin_bullet)
rai::Array<btRigidBody*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_bullet)
#endif
