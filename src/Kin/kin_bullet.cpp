/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_bullet.h"

#ifdef RAI_BULLET

#include "frame.h"

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>

//===========================================================================

constexpr float gravity = -9.81f;

//===========================================================================

void btTrans2raiTrans(rai::Transformation& f, const btTransform& pose) {
  const btQuaternion q = pose.getRotation();
  const btVector3& p = pose.getOrigin();
  f.pos.set(p.x(), p.y(), p.z());
  f.rot.set(q.w(), q.x(), q.y(), q.z());
}

btTransform conv_trans_btTrans(const rai::Transformation& X) {
  btTransform pose(btQuaternion(X.rot.x, X.rot.y, X.rot.z, X.rot.w),
                   btVector3(X.pos.x, X.pos.y, X.pos.z));
  return pose;
}

arr conv_btVec3_arr(const btVector3& v) {
  return arr{v.x(), v.y(), v.z()};
}

btVector3 conv_arr_btVec3(const arr& v) {
  CHECK_EQ(v.N, 3, "");
  return btVector3(v.elem(0), v.elem(1), v.elem(2));
}

btVector3 conv_vec_btVec3(const rai::Vector& v) {
  return btVector3(v.x, v.y, v.z);
}

btQuaternion conv_rot_btQuat(const rai::Quaternion& rot) {
  return btQuaternion(rot.x, rot.y, rot.z, rot.w);
}

//===========================================================================

struct MultiBodyInfo {
  btMultiBody* multibody;
  rai::Array<rai::Frame*> links;
  rai::Array<btMultiBodyJointMotor*> motors;
};

struct BulletInterface_self {
  btDefaultCollisionConfiguration* collisionConfiguration=0;
  btCollisionDispatcher* dispatcher=0;
  btBroadphaseInterface* broadphase=0;
  btSequentialImpulseConstraintSolver* solver=0;
  btDiscreteDynamicsWorld* dynamicsWorld=0;
  btMultiBodyConstraintSolver* mbsol=0;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  btSoftBodyWorldInfo softBodyWorldInfo;

  rai::Array<btCollisionObject*> actors;
  rai::Array<rai::BodyType> actorTypes;
  rai::Array<MultiBodyInfo> multibodies;

  rai::Bullet_Options opt;

  uint stepCount=0;

  void initPhysics();
  void initPhysics2();
  btRigidBody* addGround();
  btRigidBody* addLink(rai::Frame* f);
  btSoftBody* addSoft(rai::Frame* f);
  btMultiBody* addMultiBody(rai::Frame* base);
  void addExample();

  btCollisionShape* createSingleShape(rai::Shape* s);
  btCollisionShape* createCompoundCollisionShape(rai::Frame* link, ShapeL& shapes);
  btCollisionShape* createLinkShape(ShapeL& shapes, rai::BodyType& type, rai::Frame* f);
};

//===========================================================================

void BulletInterface_self::initPhysics() {
  if(opt.verbose>0) LOG(0) <<"starting bullet engine ...";
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  broadphase = new btDbvtBroadphase();
  //m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

  if(opt.softBody) {
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  } else if(opt.multiBody) {
    mbsol = new btMultiBodyConstraintSolver;
    dynamicsWorld = new btMultiBodyDynamicsWorld(dispatcher, broadphase, mbsol, collisionConfiguration);
    dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;
  } else {
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  }

  if(opt.yGravity) {
    dynamicsWorld->setGravity(btVector3(0, gravity, 0));
    softBodyWorldInfo.m_gravity.setValue(0, gravity, 0);
  } else {
    dynamicsWorld->setGravity(btVector3(0, 0, gravity));
    softBodyWorldInfo.m_gravity.setValue(0, 0, gravity);
  }

  if(opt.softBody) {
    softBodyWorldInfo.m_dispatcher = dispatcher;
    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_sparsesdf.Initialize();
    softBodyWorldInfo.air_density = (btScalar)1.2;
    softBodyWorldInfo.water_density = 0;
    softBodyWorldInfo.water_offset = 0;
    softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
  }

  if(opt.verbose>0) LOG(0) <<"... done starting bullet engine";
}

void BulletInterface_self::initPhysics2() {
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  broadphase = new btDbvtBroadphase();
  mbsol = new btMultiBodyConstraintSolver;
  dynamicsWorld = new btMultiBodyDynamicsWorld(dispatcher, broadphase, mbsol, collisionConfiguration);

  dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;
  dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
}

btRigidBody* BulletInterface_self::addGround() {
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0, 0, 0));
  btCollisionShape* groundShape;
  if(opt.yGravity) {
    groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
  } else {
    groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
  }
  collisionShapes.push_back(groundShape);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* body = new btRigidBody(rbInfo);
  if(opt.verbose>0) LOG(0) <<"bullet defaults: " <<body->getFriction() <<' ' <<body->getRestitution() <<' ' <<body->getContactStiffness() <<' ' <<body->getContactDamping();
  body->setFriction(opt.defaultFriction);
  body->setRestitution(opt.defaultRestitution);
  body->setContactStiffnessAndDamping(opt.contactStiffness, opt.contactDamping);
  dynamicsWorld->addRigidBody(body, 1, 1+2);
  return body;
}

btRigidBody* BulletInterface_self::addLink(rai::Frame* f) {
  ShapeL shapes;
  rai::BodyType type;
  btCollisionShape* colShape = createLinkShape(shapes, type, f);

  //-- create a bullet body
  btTransform pose = conv_trans_btTrans(f->ensure_X());
  btScalar mass(1.0f);
  btVector3 localInertia(0, 0, 0);
  if(type==rai::BT_dynamic) {
    if(f->inertia) {
      mass = f->inertia->mass;
      CHECK(f->inertia->com.isZero, "need zero com");
      CHECK(f->inertia->matrix.isDiagonal(), "need diagonal matrix");
      localInertia.setValue(f->inertia->matrix.m00, f->inertia->matrix.m11, f->inertia->matrix.m22);
    } else {
      colShape->calculateLocalInertia(mass, localInertia);
    }
  } else {
    mass=0.;
  }

  btDefaultMotionState* motionState = new btDefaultMotionState(pose);
  btRigidBody* body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia));

  //-- these are physics tweaks
  {
    double friction=opt.defaultFriction;
    for(auto s:shapes) if(s->frame.ats) s->frame.ats->get<double>(friction, "friction");
    if(friction>=0.) {
      if(opt.verbose>1) LOG(0) <<"setting friction of '" <<f->name <<"' to " <<friction;
      body->setFriction(friction);
    }
  }
  body->setRollingFriction(.01);
  body->setSpinningFriction(.01);
  {
    double restitution=opt.defaultRestitution;
    for(auto s:shapes) if(s->frame.ats) s->frame.ats->get<double>(restitution, "restitution");
    if(restitution>=0.) body->setRestitution(restitution);
  }
  body->setContactStiffnessAndDamping(opt.contactStiffness, opt.contactDamping);

  dynamicsWorld->addRigidBody(body);

  if(type==rai::BT_kinematic) {
    body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
  }

  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = body;
  return body;
}

btMultiBody* BulletInterface_self::addMultiBody(rai::Frame* base) {
  CHECK(!base->parent || (base->joint && base->joint->type==rai::JT_rigid) || (base->joint && base->inertia), "base needs to be either rigid or with inertia");
  //-- collect all links for that root
  FrameL F = {base};
  base->getPartSubFrames(F);
  FrameL links = {base};
  for(auto* f:F) { if(f->joint && !f->joint->isPartBreak) links.append(f); }
//  if(links.N==1){ addLink(base); return 0; } //actually just a single body, not multibody...
  intA parents(links.N);
  parents(0) = -1;
  for(uint i=1; i<links.N; i++) {
    rai::Frame* f = links(i);
    rai::Frame* p = f->parent;
    p = p->getUpwardLink();
    int idx = links.findValue(p);
    CHECK(idx>=0, "");
    parents(i) = idx;
  }

  if(opt.verbose>0) {
    LOG(0) <<"adding multibody with base '" <<base->name <<"' and links:";
    for(rai::Frame* f:links) cout <<f->name <<' ';
    cout <<"..." <<endl;
  }

  //   and decide on COM frames
  FrameL masses = links;
  for(uint i=0; i<links.N; i++) {
    if(links(i)->inertia) {
      //mass = link -- all good
    } else {
      for(rai::Frame* f:links(i)->children) {
        if(f->inertia) {
          masses(i) = f;
          break;
        }
      }
    }
  }

  btMultiBody* multibody = 0;
  auto world = dynamic_cast<btMultiBodyDynamicsWorld*>(dynamicsWorld);
  CHECK(world, "need a btMultiBodyDynamicsWorld");

  for(uint i=0; i<links.N; i++) {
    //create collision shape
    rai::Frame* linkJoint = links(i);
    rai::Frame* linkMass = masses(i);
    if(i==0) {
      linkJoint=0;
    } else {
      //CHECK(!linkJoint->inertia, "");
      //CHECK(linkMass->inertia, "");
      CHECK(linkJoint->joint, "");
    }
    ShapeL shapes;
    rai::BodyType type;
    btCollisionShape* colShape=createLinkShape(shapes, type, links(i));
    if(i>0) actorTypes(linkMass->ID) = rai::BT_dynamic;

    //get inertia
    btScalar mass(.1f);
    btVector3 localInertia(0.01, 0.01, 0.01);
    if(linkMass->inertia) {
      mass = linkMass->inertia->mass;
      CHECK(linkMass->inertia->com.isZero, "need zero com");
      CHECK(linkMass->inertia->matrix.isDiagonal(), "need diagonal matrix");
      localInertia.setValue(linkMass->inertia->matrix.m00, linkMass->inertia->matrix.m11, linkMass->inertia->matrix.m22);
    } else {
      if(colShape) colShape->calculateLocalInertia(mass, localInertia);
    }

    if(!i) { //base!!
      bool fixedBase = actorTypes(linkMass->ID)!=rai::BT_dynamic;
      multibody = new btMultiBody(links.N-1, mass, localInertia, fixedBase, false);
      multibody->setBaseWorldTransform(conv_trans_btTrans(linkMass->ensure_X()));
    } else { //link
      //get parent and coms
      //rai::Frame* parJoint = links(parents(i));
      rai::Frame* parMass = masses(parents(i));
      rai::Transformation relA = linkJoint->parent->ensure_X() / parMass ->ensure_X();
      rai::Transformation relB = linkMass->get_Q();
      //CHECK(relB.rot.isZero, ""); //check should hold only when all joint angles (linkJoint->Q) are zero
      switch(linkJoint->joint->type) {
        case rai::JT_hingeX: {
          btVector3 axis(1, 0, 0);
          multibody->setupRevolute(i-1, mass, localInertia, parents(i)-1, conv_rot_btQuat(-relA.rot), axis, conv_vec_btVec3(relA.pos), conv_vec_btVec3(relB.pos), true);
        } break;
        case rai::JT_transX: {
          btVector3 axis(1, 0, 0);
          CHECK(relA.pos.isZero, "offsets not handled properly - insert a pre-frame");
          CHECK(relB.pos.isZero, "offsets not handled properly - insert a pre-frame");
//          relA.pos.setZero(); relB.pos.setZero();
          multibody->setupPrismatic(i-1, mass, localInertia, parents(i)-1, conv_rot_btQuat(-relA.rot), axis, conv_vec_btVec3(relA.pos), conv_vec_btVec3(relB.pos), true);
        } break;
        case rai::JT_transY: {
          btVector3 axis(0, 1, 0);
          CHECK(relA.pos.isZero, "offsets not handled properly - insert a pre-frame");
          CHECK(relB.pos.isZero, "offsets not handled properly - insert a pre-frame");
//          relA.pos.setZero(); relB.pos.setZero();
          multibody->setupPrismatic(i-1, mass, localInertia, parents(i)-1, conv_rot_btQuat(-relA.rot), axis, conv_vec_btVec3(relA.pos), conv_vec_btVec3(relB.pos), true);
        } break;
        case rai::JT_transZ: {
          btVector3 axis(0, 0, 1);
          CHECK(relA.pos.isZero, "offsets not handled properly - insert a pre-frame");
          CHECK(relB.pos.isZero, "offsets not handled properly - insert a pre-frame");
//          relA.pos.setZero(); relB.pos.setZero();
          multibody->setupPrismatic(i-1, mass, localInertia, parents(i)-1, conv_rot_btQuat(-relA.rot), axis, conv_vec_btVec3(relA.pos), conv_vec_btVec3(relB.pos), true);
        } break;
        default: NIY;
      };
      if(linkJoint->joint->limits.N) {
        btMultiBodyConstraint* limitCons = new btMultiBodyJointLimitConstraint(multibody, i-1, linkJoint->joint->limits(0), linkJoint->joint->limits(1));
        world->addMultiBodyConstraint(limitCons);
      }
      if(linkJoint->joint->mimic) {
        btVector3 pivot(0, 1, 0);
        btMatrix3x3 frame(1, 0, 0, 0, 1, 0, 0, 0, 1);
        //HARD CODED: mimicer is the previous one
        btMultiBodyConstraint* gearCons = new btMultiBodyGearConstraint(multibody, i-1, multibody, i-2, pivot, pivot, frame, frame);
        gearCons->setGearRatio(-1); //why needed? already flipped in config..
        gearCons->setErp(0.1);
        gearCons->setMaxAppliedImpulse(50);
        world->addMultiBodyConstraint(gearCons);
      }
    }

    //add collider
    if(colShape) {
      btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(multibody, i-1); //-1 for root!
      col->setCollisionShape(colShape);
      col->setWorldTransform(conv_trans_btTrans(linkMass->ensure_X()));
      //-- these are physics tweaks
      {
        double friction=opt.defaultFriction;
        for(auto s:shapes) if(s->frame.ats) s->frame.ats->get<double>(friction, "friction");
        if(friction>=0.) {
          if(opt.verbose>1) LOG(0) <<"setting friction of '" <<linkJoint->name <<"' to " <<friction;
          col->setFriction(friction);
        }
      }
      col->setRollingFriction(.01);
      col->setSpinningFriction(.01);
      {
        double restitution=opt.defaultRestitution;
        for(auto s:shapes) if(s->frame.ats) s->frame.ats->get<double>(restitution, "restitution");
        if(restitution>=0.) col->setRestitution(restitution);
      }
      col->setContactStiffnessAndDamping(opt.contactStiffness, opt.contactDamping);

      dynamicsWorld->addCollisionObject(col, 2, 1+2);
      if(!i) multibody->setBaseCollider(col);
      else multibody->getLink(i-1).m_collider = col;
      actors(linkMass->ID) = col;
    }
  }

  multibody->finalizeMultiDof();

  multibody->setCanSleep(false);
  multibody->setHasSelfCollision(true);
  multibody->setUseGyroTerm(false);
  multibody->setLinearDamping(0.1f);
  multibody->setAngularDamping(0.9f);

  for(uint i=1; i<links.N; i++) {
    multibody->setJointPos(i-1, links(i)->joint->q0.scalar());
  }

  if(false) { //use bullet's internal forward kinematics to recompute all poses
    btAlignedObjectArray<btQuaternion> rot;
    btAlignedObjectArray<btVector3> pos;
    multibody->forwardKinematics(rot, pos);

    for(uint i=1; i<links.N; i++) {
      multibody->getLink(i-1).m_collider->setWorldTransform(multibody->getLink(i-1).m_cachedWorldTransform);
    }
  }

  world->addMultiBody(multibody);
  multibodies.append(MultiBodyInfo{multibody, links, {}});

  if(opt.verbose>0) {
    LOG(0) <<"... done with multibody with base '" <<base->name <<"'";
  }

  return multibody;
}

void BulletInterface_self::addExample() {
  int numLinks = 1;
  btVector3 basePosition(-0.4f, 3.f, 0.f);
  btVector3 linkHalfExtents(0.05, 0.37, 0.1);
  btTransform pose;

  //base
  btMultiBody* multibody = new btMultiBody(numLinks, 1., btVector3(0, 0, 0), true, false);
  pose.setIdentity();
  pose.setOrigin(basePosition);
  multibody->setBaseWorldTransform(pose);

  //init the links
  for(int i = 0; i < numLinks; ++i) {
    btVector3 hingeJointAxis(1, 0, 0);
    float linkMass = 1.f;
    btVector3 linkInertiaDiag(0.01f, 0.05f, 0.01f);
    btVector3 jointToLink(0, -linkHalfExtents[1], 0);
    btVector3 parentToJoint(0, -linkHalfExtents[1], 0);
    multibody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentToJoint, jointToLink, true);
  }

  multibody->finalizeMultiDof();
  auto world = dynamic_cast<btMultiBodyDynamicsWorld*>(dynamicsWorld);
  CHECK(world, "need a btMultiBodyDynamicsWorld");
  world->addMultiBody(multibody);

  multibodies.append(MultiBodyInfo{multibody, {}, {}});

  multibody->setCanSleep(false);
  multibody->setHasSelfCollision(true);
  multibody->setUseGyroTerm(true);
  multibody->setLinearDamping(0.1f);
  multibody->setAngularDamping(0.9f);

  multibody->setJointPos(0, 1.);

  {
    btAlignedObjectArray<btQuaternion> rot;
    btAlignedObjectArray<btVector3> pos;
    multibody->forwardKinematics(rot, pos);
  }

  for(int i = -1; i < multibody->getNumLinks(); ++i) {
    btCollisionShape* box = new btBoxShape(linkHalfExtents);
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(multibody, i);
    col->setCollisionShape(box);
    //col->setFriction(friction);
    world->addCollisionObject(col, 2, 1 + 2);
    if(i==-1) {
      col->setWorldTransform(btTransform(multibody->getWorldToBaseRot(), multibody->getBasePos()));
      multibody->setBaseCollider(col);
    } else {
      col->setWorldTransform(multibody->getLink(i).m_cachedWorldTransform);
      multibody->getLink(i).m_collider = col;
    }
    if(i==-1) actors(0) = col;
    else actors(3) = col;
  }

}

btSoftBody* BulletInterface_self::addSoft(rai::Frame* f) {
  //-- collect all shapes of that link
  CHECK_EQ(f->children.N, 0, "");
  //-- check inertia

  //-- decide on the type
  actorTypes(f->ID) = rai::BT_soft;
  if(opt.verbose>0) LOG(0) <<"adding link anchored at '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(rai::BT_soft);

  //-- create a bullet collision shape
  rai::Mesh& m = f->shape->mesh();

  btSoftBody* softbody = btSoftBodyHelpers::CreateRope(softBodyWorldInfo,
                         conv_arr_btVec3(m.V[0]),
                         conv_arr_btVec3(m.V[-1]),
                         m.V.d0-2,
                         1); //root fixed? tail fixed?
  softbody->m_cfg.piterations = 4;
  softbody->m_materials[0]->m_kLST = 0.5;
  softbody->setTotalMass(f->inertia->mass);
  auto world = dynamic_cast<btSoftRigidDynamicsWorld*>(dynamicsWorld);
  CHECK(world, "need a btSoftRigidDynamicsWorld");
  world->addSoftBody(softbody);

  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = softbody;
  return softbody;
}

btCollisionShape* BulletInterface_self::createSingleShape(rai::Shape* s) {
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
    case rai::ST_ssCylinder:
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
      floatA V = rai::convert<float>(s->mesh().V);
#endif
      colShape = new btConvexHullShape(V.p, V.d0, V.sizeT*V.d1);
//      rai::Mesh& m = s->mesh();
//      colShape = new btTriangleMeshShape();
//      colShape = new btTriangleIndexVertexArray(m.T.d0, m.T.p, 3*m.T.sizeT, V.d0, V.p, 3*V.sizeT);
      colShape->setMargin(0.);
    } break;
    default: HALT("NIY" <<s->type());
  }
  return colShape;
}

btCollisionShape* BulletInterface_self::createCompoundCollisionShape(rai::Frame* link, ShapeL& shapes) {
  btCompoundShape* colShape = new btCompoundShape;
  for(rai::Shape* s:shapes) {
    colShape->addChildShape(conv_trans_btTrans(s->frame.ensure_X()/link->ensure_X()), createSingleShape(s));
  }
  return colShape;
}

btCollisionShape* BulletInterface_self::createLinkShape(ShapeL& shapes, rai::BodyType& type, rai::Frame* f) {
  //-- collect all shapes of that link
  {
    rai::Frame* link=f->getUpwardLink();
    FrameL tmp = {link};
    link->getRigidSubFrames(tmp, false);
    for(rai::Frame* p: tmp) {
      if(p->shape
          && p->getShape().type()!=rai::ST_marker
          && p->getShape().type()!=rai::ST_camera
          && p->getShape().alpha()==1.) shapes.append(p->shape);
    }
  }

  //-- check inertia
  bool shapesHaveInertia=false;
  for(rai::Shape* s:shapes) if(s->frame.inertia) { shapesHaveInertia=true; break; }
  if(shapesHaveInertia && !f->inertia) {
    LOG(-1) <<"computing compound inertia for object frame '" <<f->name <<"' -- this should have been done earlier?";
    f->computeCompoundInertia();
    f->transformToDiagInertia();
#if 0 //we relocate that frame to have zero COM (bullet only accepts zero COM and diagonal inertia)
    if(!f->inertia->com.isZero) {
      CHECK(!f->shape || f->shape->type()==rai::ST_marker, "can't translate this frame if it has a shape attached");
      CHECK(!f->joint || f->joint->type==rai::JT_rigid || f->joint->type==rai::JT_free, "can't translate this frame if it has a joint attached");
      LOG(0) <<"translating frame '" <<f->name <<"' to accomodate for centered compound inertia for bullet";
      f->set_X()->pos += f->ensure_X().rot * f->inertia->com;
      for(rai::Frame* ch:f->children) ch->set_Q()->pos -= f->inertia->com;
      f->inertia->com.setZero();
//MISSING HERE: ALSO TRANSFORM THE INERTIA MATRIX TO BECOME DIAG!
    }
#endif
  }
  if(f->inertia && !f->inertia->com.isZero) {
    THROW("DON'T DO THAT! Bullet can only properly handle (compound) inertias if transformed to zero com and diagonal tensor")
  }

  //-- decide on the type
  type = rai::BT_static;
//  if(shapes.N) {
  if(f->joint)   type = rai::BT_kinematic;
  if(f->inertia) type = f->inertia->type;
//  }
  actorTypes(f->ID) = type;
  if(opt.verbose>0) LOG(0) <<"adding link '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type) <<" with " <<shapes.N <<" shapes";

  if(!shapes.N) return 0;

  //-- create a bullet collision shape
  btCollisionShape* colShape=0;
  if(shapes.N==1 && f == &shapes.scalar()->frame) {
    colShape = createSingleShape(shapes.scalar());
  } else {
    colShape = createCompoundCollisionShape(f, shapes);
  }
  collisionShapes.push_back(colShape);

  return colShape;
}

//===========================================================================

BulletInterface::BulletInterface(rai::Configuration& C, const rai::Bullet_Options& opt) : self(nullptr) {
  self = new BulletInterface_self;

  self->opt = opt;

  self->initPhysics();

  self->addGround();

  if(opt.verbose>0) LOG(0) <<"creating Configuration within bullet ...";

  self->actors.resize(C.frames.N).setZero();
  self->actorTypes.resize(C.frames.N).setZero();

  if(opt.multiBody) {
    FrameL parts = C.getParts();
    for(rai::Frame* f : parts) {
      bool asMultiBody=false;
      FrameL sub = f->getSubtree();
      for(rai::Frame* a:sub) if(a->joint) { asMultiBody=true; break; }

      if(asMultiBody) {
        self->addMultiBody(f);
        if(f->ats && (*f->ats)["motors"]) motorizeMultiBody(f);
      } else {
        self->addLink(f);
      }
    }
    //  self->addMultiBody(C(0), verbose);
    //  self->addExample();
  } else {
    FrameL links = C.getLinks();
    for(rai::Frame* a : links) {
      if(a->inertia && a->inertia->type==rai::BT_soft) {
        CHECK(opt.softBody, "");
        self->addSoft(a);
      } else {
        self->addLink(a);
      }
    }
  }

  if(opt.verbose>0) LOG(0) <<"... done creating Configuration within bullet";
}

BulletInterface::~BulletInterface() {
  for(int i = self->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = self->dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = dynamic_cast<btRigidBody*>(obj);
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
  delete self->broadphase;
  delete self->dispatcher;
  delete self->collisionConfiguration;
  self->collisionShapes.clear();
}

void BulletInterface::step(double tau) {
  self->stepCount++;
  self->dynamicsWorld->stepSimulation(tau, 10, 1. / 240.f);
}

void pullPoses(rai::Configuration& C, const rai::Array<btCollisionObject*>& actors, const rai::Array<MultiBodyInfo>& multibodies, arr& frameVelocities, bool alsoStaticAndKinematic) {
  if(!!frameVelocities) frameVelocities.resize(C.frames.N, 2, 3).setZero();

  for(rai::Frame* f : C.frames) {
    if(actors.N <= f->ID) continue;

    btCollisionObject* obj = actors(f->ID);
    if(!obj) continue;
    btRigidBody* body = dynamic_cast<btRigidBody*>(obj);
    btMultiBodyLinkCollider* multibodycoll = dynamic_cast<btMultiBodyLinkCollider*>(obj);
    btMultibodyLink* link=0;
    if(multibodycoll && multibodycoll->m_link>=0) link = &multibodycoll->m_multiBody->getLink(multibodycoll->m_link);
    btSoftBody* softbody = dynamic_cast<btSoftBody*>(obj);

    if(body || link) {
      if(link || alsoStaticAndKinematic || !body->isStaticOrKinematicObject()) {
        rai::Transformation X;
        btTransform pose;
        if(body && body->getMotionState()) {
          body->getMotionState()->getWorldTransform(pose);
          btTrans2raiTrans(X, pose);
          f->set_X() = X;
        } else {
#if 1 //pull poses of multibody links -> indirectly defines joint transformations (redundant for non-floating roots!)
          pose = obj->getWorldTransform();
          btTrans2raiTrans(X, pose);
          if(f->parent && f->parent->joint) {
            f->parent->set_X() = X * (-f->get_Q());
          } else {
            f->set_X() = X;
          }
#endif
        }
        if(!!frameVelocities) {
          frameVelocities(f->ID, 0, {}) = conv_btVec3_arr(body->getLinearVelocity());
          frameVelocities(f->ID, 1, {}) = conv_btVec3_arr(body->getAngularVelocity());
        }
      }
    } else if(softbody) {
      rai::Mesh& m = f->shape->mesh();
      CHECK_EQ((int)m.V.d0, softbody->m_nodes.size(), "");
      for(int i=0; i<softbody->m_nodes.size(); i++) {
        m.V[i] = conv_btVec3_arr(softbody->m_nodes[i].m_x);
      }
    } else {
      //is ok: compound or multi piece
    }
  }

  //-- pull joint state directly
  if(multibodies.N) {
    arr q = C.getJointState();
    for(const MultiBodyInfo& mi:multibodies) {
      uint n = mi.multibody->getNumLinks();
      for(uint i=0; i<n; i++) {
        q(mi.links(i+1)->joint->qIndex) = mi.multibody->getJointPos(i);
      }
    }
    C.setJointState(q);
  }
}

void BulletInterface::pullDynamicStates(rai::Configuration& C, arr& frameVelocities) {
  pullPoses(C, self->actors, self->multibodies, frameVelocities, false);
}

void BulletInterface::changeObjectType(rai::Frame* f, int _type, const arr& withVelocity) {
  rai::Enum<rai::BodyType> type((rai::BodyType)_type);
  if(self->actorTypes(f->ID) == type) {
    //LOG(-1) <<"frame " <<*f <<" is already of type " <<type;
  }

  btRigidBody* a = dynamic_cast<btRigidBody*>(self->actors(f->ID));
  if(!a) HALT("frame " <<*f <<"is not an actor");

  if(type==rai::BT_kinematic) {
    a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    a->setActivationState(DISABLE_DEACTIVATION);
  } else if(type==rai::BT_dynamic) {
    a->setCollisionFlags(a->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    a->setActivationState(DISABLE_DEACTIVATION);
    if(withVelocity.N) {
      a->setLinearVelocity(btVector3(withVelocity(0), withVelocity(1), withVelocity(2)));
    }
  } else NIY;
  self->actorTypes(f->ID) = type;
}

void BulletInterface::motorizeMultiBody(rai::Frame* base) {
  if(opt().verbose>0) LOG(0) <<"motorizing multibody with base '" <<base->name <<"'";
  CHECK(self->opt.multiBody, "");
  uint i=0;
  for(; i<self->multibodies.N; i++) {
    if(self->multibodies(i).links.first()==base) break;
  }
  CHECK(i<self->multibodies.N, "");
  MultiBodyInfo& mi = self->multibodies(i);
  uint n = mi.multibody->getNumLinks();
  CHECK(!mi.motors.N, "");
  mi.motors.resize(n) = 0;
  auto world = dynamic_cast<btMultiBodyDynamicsWorld*>(self->dynamicsWorld);
  CHECK(world, "need a btMultiBodyDynamicsWorld");
  for(uint i=0; i<n; i++) {
    auto mot = new btMultiBodyJointMotor(mi.multibody, i, 0., 100000.);
    if(!mi.links(i+1)->joint->mimic) {
      world->addMultiBodyConstraint(mot);
      arr q = mi.links(i+1)->joint->calcDofsFromConfig();
      mot->setPositionTarget(q.scalar(), opt().motorKp);
      mot->setVelocityTarget(0., opt().motorKd);
    }
    mi.motors(i) = mot;
  }
}

void BulletInterface::setMotorQ(const arr& q_ref, const arr& qDot_ref) {
  CHECK(self->opt.multiBody, "");
  if(qDot_ref.N) { CHECK_EQ(q_ref.N, qDot_ref.N, ""); }
  uint qIdx=0;
  for(MultiBodyInfo& mi:self->multibodies) {
    for(uint i=0; i<mi.motors.N; i++) {
      mi.motors(i)->setPositionTarget(q_ref.elem(qIdx), opt().motorKp);
      if(qDot_ref.N) {
        mi.motors(i)->setVelocityTarget(qDot_ref.elem(i), opt().motorKd);
      }
      qIdx++;
    }
  }
  CHECK_EQ(qIdx, q_ref.N, ""); //make this only a warning?
}

void BulletInterface::pushKinematicStates(const rai::Configuration& C) {
  for(rai::Frame* f: C.frames) {
    if(self->actors.N <= f->ID) continue;
    if(self->actorTypes(f->ID)==rai::BT_kinematic) {
      btRigidBody* b = dynamic_cast<btRigidBody*>(self->actors(f->ID));
      if(!b) continue; //f is not an actor

      CHECK(b->getMotionState(), "");
      b->getMotionState()->setWorldTransform(conv_trans_btTrans(f->ensure_X()));
    }
  }
}

void BulletInterface::pushFullState(const rai::Configuration& C, const arr& frameVelocities) {
  for(rai::Frame* f : C.frames) {
    if(self->actors.N <= f->ID) continue;
    btRigidBody* b = dynamic_cast<btRigidBody*>(self->actors(f->ID));
    if(!b) continue; //f is not an actor

    b->setWorldTransform(conv_trans_btTrans(f->ensure_X()));
    b->setActivationState(ACTIVE_TAG);
    if(self->actorTypes(f->ID)==rai::BT_dynamic) {
      b->clearForces();
      if(!!frameVelocities && frameVelocities.N) {
        b->setLinearVelocity(btVector3(frameVelocities(f->ID, 0, 0), frameVelocities(f->ID, 0, 1), frameVelocities(f->ID, 0, 2)));
        b->setAngularVelocity(btVector3(frameVelocities(f->ID, 1, 0), frameVelocities(f->ID, 1, 1), frameVelocities(f->ID, 1, 2)));
      } /*else {
        b->setLinearVelocity(btVector3(0., 0., 0.));
        b->setAngularVelocity(btVector3(0., 0., 0.));
      }*/
    }
  }
  self->dynamicsWorld->stepSimulation(.01); //without this, two consequtive pushFullState won't work! (something active tag?)
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

btDiscreteDynamicsWorld* BulletInterface::getDynamicsWorld() {
  return self->dynamicsWorld;
}

rai::Bullet_Options& BulletInterface::opt() {
  return self->opt;
}

BulletBridge::BulletBridge(btDiscreteDynamicsWorld* _dynamicsWorld) : dynamicsWorld(_dynamicsWorld) {
  btCollisionObjectArray& collisionObjects = dynamicsWorld->getCollisionObjectArray();
  actors.resize(collisionObjects.size()).setZero();
  for(int i=0; i<collisionObjects.size(); i++)  actors(i) = collisionObjects[i];
}

void BulletBridge::getConfiguration(rai::Configuration& C) {
  btCollisionObjectArray& collisionObjects = dynamicsWorld->getCollisionObjectArray();
  for(int i=0; i<collisionObjects.size(); i++) {
    btCollisionObject* obj = collisionObjects[i];
    btCollisionShape* shape = obj->getCollisionShape();
    cout <<"OBJECT " <<i <<" type:" <<obj->getInternalType() <<" shapeType: " <<shape->getShapeType() <<' ' <<shape->getName() <<endl;
    if(obj->getInternalType()==obj->CO_COLLISION_OBJECT) {
      rai::Transformation X;
      btTrans2raiTrans(X, obj->getWorldTransform());
      C.addFrame(STRING("coll"<<i))
      ->setShape(rai::ST_marker, {.1})
      .setPose(X);
    }
    btRigidBody* body = dynamic_cast<btRigidBody*>(obj);
    btMultiBodyLinkCollider* multibodycoll = dynamic_cast<btMultiBodyLinkCollider*>(obj);
    btMultibodyLink* link=0;
    if(multibodycoll && multibodycoll->m_link>=0) link = &multibodycoll->m_multiBody->getLink(multibodycoll->m_link);

    if(body || link) {
      rai::Transformation X;
      btTransform pose;
      if(body && body->getMotionState()) {
        body->getMotionState()->getWorldTransform(pose);
      } else {
        pose = obj->getWorldTransform();
      }
      btTrans2raiTrans(X, pose);
      cout <<"OBJECT " <<i <<" pose: " <<X <<" shapeType: " <<shape->getShapeType() <<' ' <<shape->getName();
      rai::Frame* f=0;
      switch(shape->getShapeType()) {
        case CONVEX_HULL_SHAPE_PROXYTYPE: {
          btConvexHullShape* obj = dynamic_cast<btConvexHullShape*>(shape);
          arr V(obj->getNumPoints(), 3);
          for(uint i=0; i<V.d0; i++) V[i] = conv_btVec3_arr(obj->getUnscaledPoints()[i]);
          f = C.addFrame(STRING("obj"<<i));
          f->setConvexMesh(V)
          .setPose(X);
        } break;
        case BOX_SHAPE_PROXYTYPE: {
          btBoxShape* box = dynamic_cast<btBoxShape*>(shape);
          arr size = 2.*conv_btVec3_arr(box->getHalfExtentsWithMargin());
          cout <<" margin: " <<box->getMargin() <<" size: " <<size;
          f = C.addFrame(STRING("obj"<<i));
          f->setShape(rai::ST_box, size)
          .setPose(X);
        } break;
        case SPHERE_SHAPE_PROXYTYPE: {
          btSphereShape* obj = dynamic_cast<btSphereShape*>(shape);
          arr size = {obj->getRadius()};
          cout <<" margin: " <<obj->getMargin() <<" size: " <<size;
          f = C.addFrame(STRING("obj"<<i));
          f->setShape(rai::ST_sphere, size)
          .setPose(X);
        } break;
        case CYLINDER_SHAPE_PROXYTYPE: {
          btCylinderShape* obj = dynamic_cast<btCylinderShape*>(shape);
          arr size = 2.*conv_btVec3_arr(obj->getHalfExtentsWithMargin());
          cout <<" margin: " <<obj->getMargin() <<" size: " <<size;
          size(1) = size(0);
          size(0) = size(2);
          size.resizeCopy(2);
          f = C.addFrame(STRING("obj"<<i));
          f->setShape(rai::ST_cylinder, size)
          .setPose(X);
        } break;
        case COMPOUND_SHAPE_PROXYTYPE:
        case STATIC_PLANE_PROXYTYPE: {
          f = C.addFrame(STRING("obj"<<i));
          f->setShape(rai::ST_marker, {.1})
          .setPose(X);
        } break;
        default: {
          NIY;
        }
      }
      if(f) {
        double mInv = 0.;
        if(body) mInv = body->getInvMass();
        if(link) mInv = 1./link->m_mass;
        if(mInv>0.) f->setMass(1./mInv);
      }
      cout <<endl;
    }
    btSoftBody* softbody = dynamic_cast<btSoftBody*>(obj);
    if(softbody) {
      rai::Frame& f = C.addFrame(STRING("soft"<<i))
                      ->setShape(rai::ST_mesh, {});
      rai::Mesh& m = f.shape->mesh();
      {
        m.V.resize(softbody->m_nodes.size(), 3);
        for(int i=0; i<softbody->m_nodes.size(); i++) {
          m.V[i] = conv_btVec3_arr(softbody->m_nodes[i].m_x);
        }
        m.makeLines();
      }

      f.setMass(softbody->getTotalMass());
      f.inertia->type = rai::BT_soft;
    }
  }
  CHECK_EQ(C.frames.N, actors.N, "");
}

void BulletBridge::pullPoses(rai::Configuration& C, bool alsoStaticAndKinematic) {
  CHECK_EQ(C.frames.N, actors.N, "");
  ::pullPoses(C, actors, {}, NoArr, alsoStaticAndKinematic);
}

#else

BulletInterface::BulletInterface(rai::Configuration& K, const rai::Bullet_Options& opt) { NICO }
BulletInterface::~BulletInterface() { NICO }
void BulletInterface::step(double tau) { NICO }
void BulletInterface::pushFullState(const rai::Configuration& C, const arr& vel) { NICO }
void BulletInterface::pushKinematicStates(const rai::Configuration& C) { NICO }
void BulletInterface::pullDynamicStates(rai::Configuration& C, arr& vel) { NICO }
void BulletInterface::setMotorQ(const arr& q_ref, const arr& qDot_ref) { NICO }
void BulletInterface::saveBulletFile(const char* filename) { NICO }
void BulletInterface::changeObjectType(rai::Frame* f, int _type, const arr& withVelocity) { NICO }
rai::Bullet_Options& BulletInterface::opt() { NICO; }

#endif

#ifdef RAI_BULLET
RUN_ON_INIT_BEGIN(kin_bullet)
rai::Array<btCollisionObject*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_bullet)
#endif
