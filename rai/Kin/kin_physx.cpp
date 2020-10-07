/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PHYSX

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <physx/PxPhysicsAPI.h>
#include <physx/extensions/PxExtensionsAPI.h>
#include <physx/extensions/PxDefaultErrorCallback.h>
#include <physx/extensions/PxDefaultAllocator.h>
#include <physx/extensions/PxDefaultSimulationFilterShader.h>
#include <physx/extensions/PxDefaultCpuDispatcher.h>
#include <physx/extensions/PxShapeExt.h>
#include <physx/foundation/PxMat33.h>
//#include <physx/pvd/PxVisualDebugger.h>
//#include <physx/physxvisualdebuggersdk/PvdConnectionFlags.h>
//#include <PxMat33Legacy.h>
#include <physx/extensions/PxSimpleFactory.h>
#pragma GCC diagnostic pop

#include "kin_physx.h"
#include "frame.h"
#include "../Gui/opengl.h"

using namespace physx;

struct PhysXSingleton {
  PxFoundation* mFoundation = nullptr;
  PxPhysics* mPhysics = nullptr;
  PxCooking* mCooking = nullptr;
  PxDefaultErrorCallback gDefaultErrorCallback;
  PxDefaultAllocator gDefaultAllocatorCallback;
  PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

  void create() {
    mFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    PxTolerancesScale scale;
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale);
    PxCookingParams cookParams(mPhysics->getTolerancesScale());
//    cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");
    //if(!PxInitExtensions(*mPhysics)) HALT("PxInitExtensions failed!");
  }

  ~PhysXSingleton() {
    if(mPhysics) {
      mCooking->release();
      mPhysics->release();
    }
    //  mFoundation->release();
  }
};

static PhysXSingleton& physxSingleton() {
  static PhysXSingleton singleton;
  return singleton;
}

// ============================================================================

void PxTrans2raiTrans(rai::Transformation& f, const PxTransform& pose) {
  f.pos.set(pose.p.x, pose.p.y, pose.p.z);
  f.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
}

PxTransform conv_Transformation2PxTrans(const rai::Transformation& f) {
  return PxTransform(PxVec3(f.pos.x, f.pos.y, f.pos.z), PxQuat(f.rot.x, f.rot.y, f.rot.z, f.rot.w));
}

PxTransform Id_PxTrans() {
  return PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(0.f, 0.f, 0.f, 1.f));
}

arr conv_PxVec3_arr(const PxVec3& v) {
  return {v.x, v.y, v.z};
}

// ============================================================================
//stuff from Samples/PxToolkit

namespace PxToolkit {
PxConvexMesh* createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = vertCount;
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = verts;
  convexDesc.flags            = flags;

  PxDefaultMemoryOutputStream buf;
  if(!cooking.cookConvexMesh(convexDesc, buf))
    return nullptr;

  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  return physics.createConvexMesh(input);
}

PxTriangleMesh* createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count     = vertCount;
  meshDesc.points.stride      = 3*sizeof(float);
  meshDesc.points.data      = verts;

  meshDesc.triangles.count    = triCount;
  meshDesc.triangles.stride   = 3*sizeof(uint);
  meshDesc.triangles.data     = indices32;

  PxDefaultMemoryOutputStream writeBuffer;
  bool status = cooking.cookTriangleMesh(meshDesc, writeBuffer);
  if(!status)
    return nullptr;

  PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
  return physics.createTriangleMesh(readBuffer);
}
}

// ============================================================================

struct PhysXInterface_self {
  PxScene* gScene = nullptr;
  rai::Array<PxRigidActor*> actors;
  rai::Array<rai::BodyType> actorTypes;
  rai::Array<PxD6Joint*> joints;
  OpenGL* gl=nullptr;
  rai::Configuration* C=nullptr;

  uint stepCount=0;

  PxMaterial* defaultMaterial;

//  debugger::comm::PvdConnection* connection = nullptr;

  void addLink(rai::Frame* b, int verbose);
  void addJoint(rai::Joint* jj);

  void lockJoint(PxD6Joint* joint, rai::Joint* rai_joint);
  void unlockJoint(PxD6Joint* joint, rai::Joint* rai_joint);
};

// ============================================================================

PhysXInterface::PhysXInterface(const rai::Configuration& C, int verbose): self(nullptr) {
  self = new PhysXInterface_self;

  if(verbose>0) LOG(0) <<"starting PhysX engine ...";

  if(!physxSingleton().mFoundation) physxSingleton().create();

  //PxExtensionVisualDebugger::connect(mPhysics->getPvdConnectionManager(),"localhost",5425, 10000, true);

  //-- Create the scene
  PxSceneDesc sceneDesc(physxSingleton().mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, -9.8f);
  sceneDesc.bounceThresholdVelocity = 10.;

  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = physxSingleton().gDefaultFilterShader;
  }

  self->gScene = physxSingleton().mPhysics->createScene(sceneDesc);
  if(!self->gScene) {
    cerr << "createScene failed!" << endl;
  }

  self->gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  self->gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

  //-- Create objects
  self->defaultMaterial = physxSingleton().mPhysics->createMaterial(10.f, 10.f, 0.1f);

  //Create ground plane
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

  PxRigidStatic* plane = physxSingleton().mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");

  PxShape* planeShape = plane->createShape(PxPlaneGeometry(), *self->defaultMaterial);
  CHECK(planeShape, "create shape failed!");
  self->gScene->addActor(*plane);

  if(verbose>0) LOG(0) <<"... done starting PhysX engine";
  if(verbose>0) LOG(0) <<"creating Configuration within PhysX ...";

  //-- create Configuration equivalent in PhysX
  // loop through Configuration
  self->actors.resize(C.frames.N); self->actors.setZero();
  self->actorTypes.resize(C.frames.N); self->actorTypes.setZero();
  for(rai::Frame* a : C.frames) a->ensure_X();
  FrameL links = C.getLinks();
  for(rai::Frame* a : links) self->addLink(a, verbose);
  //  for(rai::Joint *j : C.activeJoints) self->addJoint(j); //DONT ADD JOINTS!!!!

  if(verbose>0) LOG(0) <<"... done creating Configuration within PhysX";

  /// save data for the PVD
  //  if(rai::getParameter<bool>("physx_debugger", false)) {
  //    const char* filename = "pvd_capture.pxd2";
  //    PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

  //    self->connection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(), filename, connectionFlags);
  //    mPhysics->getVisualDebugger()->setVisualDebuggerFlags(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS | PxVisualDebuggerFlag::eTRANSMIT_CONSTRAINTS);
  //  }
}

PhysXInterface::~PhysXInterface() {
  ShutdownPhysX();
  delete self;
}

void PhysXInterface::step(double tau) {
  self->stepCount++;
  self->gScene->simulate(tau);

  //...perform useful work here using previous frame's state data
  while(!self->gScene->fetchResults()) {
  }
}

void PhysXInterface::pullDynamicStates(FrameL& frames, arr& frameVelocities) {
  if(!!frameVelocities) frameVelocities.resize(frames.N, 2, 3).setZero();

  for(rai::Frame* f : frames) {
    if(self->actors.N <= f->ID) continue;
    PxRigidActor* a = self->actors(f->ID);
    if(!a) continue;

    if(self->actorTypes(f->ID) == rai::BT_dynamic) {
      rai::Transformation X;
      PxTrans2raiTrans(X, a->getGlobalPose());
      f->set_X() = X;
      if(!!frameVelocities && a->getType() == PxActorType::eRIGID_DYNAMIC) {
        PxRigidBody* px_body = (PxRigidBody*) a;
        frameVelocities(f->ID, 0, {}) = conv_PxVec3_arr(px_body->getLinearVelocity());
        frameVelocities(f->ID, 1, {}) = conv_PxVec3_arr(px_body->getAngularVelocity());
      }
    }
  }
}

void PhysXInterface::changeObjectType(rai::Frame* f, int _type) {
  rai::Enum<rai::BodyType> type((rai::BodyType)_type);
  if(self->actorTypes(f->ID) == type) {
    LOG(-1) <<"frame " <<*f <<" is already of type " <<type;
  }
  PxRigidActor* a = self->actors(f->ID);
  if(!a) HALT("frame " <<*f <<"is not an actor");
  if(type==rai::BT_kinematic) {
    ((PxRigidDynamic*)a)->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
  } else if(type==rai::BT_dynamic) {
    ((PxRigidDynamic*)a)->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
  } else NIY;
  self->actorTypes(f->ID) = type;
}

void PhysXInterface::postAddObject(rai::Frame* f) {
  while(self->actors.N<=f->ID) self->actors.append(0);
  while(self->actorTypes.N<=f->ID) self->actorTypes.append(rai::BT_none);
  CHECK(!f->joint, "");
  f->ensure_X();
  if(!self->actors(f->ID)) {
    self->addLink(f, true);
  } else {
    HALT("NO!");
  }
}

void PhysXInterface::pushKinematicStates(const FrameL& frames) {
  for(rai::Frame* f: frames) {
    if(self->actors.N <= f->ID) continue;
    if(self->actorTypes(f->ID)==rai::BT_kinematic) {
      PxRigidActor* a = self->actors(f->ID);
      if(!a) continue; //f is not an actor

      ((PxRigidDynamic*)a)->setKinematicTarget(conv_Transformation2PxTrans(f->ensure_X()));
    }
  }
}

void PhysXInterface::pushFullState(const FrameL& frames, const arr& frameVelocities, bool onlyKinematic) {
  for(rai::Frame* f : frames) {
    if(self->actors.N <= f->ID) continue;
    PxRigidActor* a = self->actors(f->ID);
    if(!a) continue; //f is not an actor

    bool isKinematic = self->actorTypes(f->ID)==rai::BT_kinematic;
    if(onlyKinematic && !isKinematic) continue;
    if(self->actorTypes(f->ID)!=rai::BT_kinematic) {
      a->setGlobalPose(conv_Transformation2PxTrans(f->ensure_X()));
    } else {
      ((PxRigidDynamic*)a)->setKinematicTarget(conv_Transformation2PxTrans(f->ensure_X()));
    }
    if(self->actorTypes(f->ID)==rai::BT_dynamic) {
      if(!!frameVelocities && frameVelocities.N) {
        arr v = frameVelocities(f->ID, 0, {}), w = frameVelocities(f->ID, 1, {});
        PxRigidBody* px_body = (PxRigidBody*) a;
        px_body->setLinearVelocity(PxVec3(frameVelocities(f->ID, 0, 0), frameVelocities(f->ID, 0, 1), frameVelocities(f->ID, 0, 2)));
        px_body->setAngularVelocity(PxVec3(frameVelocities(f->ID, 1, 0), frameVelocities(f->ID, 1, 1), frameVelocities(f->ID, 1, 2)));
      } else {
        PxRigidBody* px_body = (PxRigidBody*) a;
        px_body->setLinearVelocity(PxVec3(0., 0., 0.));
        px_body->setAngularVelocity(PxVec3(0., 0., 0.));
      }
    }
  }
}

void PhysXInterface::setArticulatedBodiesKinematic(const rai::Configuration& C) {
  HALT("NOT SURE IF THIS IS DESIRED");
  for(rai::Joint* j:C.activeJoints) if(j->type!=rai::JT_free) {
      if(j->from()->inertia && j->from()->inertia->type==rai::BT_dynamic) j->from()->inertia->type=rai::BT_kinematic;
      if(j->frame->inertia   && j->frame->inertia->type==rai::BT_dynamic) j->frame->inertia->type=rai::BT_kinematic;
    }
  for(rai::Frame* b: C.frames) if(self->actors(b->ID) && b->inertia) {
      if(b->inertia->type==rai::BT_kinematic)
        ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
      if(b->inertia->type==rai::BT_dynamic)
        ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    }
}

/**
 * @brief Create the PhysX interface which then can be used by OpenGL.
 *
 * - setup some physx stuff
 * - create PhysX equivalent to the Configuration
 */

void PhysXInterface_self::addJoint(rai::Joint* jj) {
  HALT("REALLY?");
  while(joints.N <= jj->frame->ID)
    joints.append(nullptr);

  //  cout <<"ADDING JOINT " <<jj->frame->parent->name <<'-' <<jj->frame->name <<endl;

  rai::Transformation rel;
  rai::Frame* from = jj->frame->getUpwardLink(rel);

  if(!jj->frame->inertia || !from || !from->inertia) return;
  CHECK(jj->frame->inertia, "this joint belongs to a frame '" <<jj->frame->name <<"' without inertia");
  CHECK(from, "this joint ('" <<jj->frame->name <<"') links from nullptr");
  CHECK(from->inertia, "this joint ('" <<jj->frame->name <<"') links from a frame '" <<from->name <<"' without inertia");

  PxTransform A = conv_Transformation2PxTrans(rel);
  PxTransform B = Id_PxTrans();
  switch(jj->type) {
    case rai::JT_free: //do nothing
      break;
    case rai::JT_hingeX:
    case rai::JT_hingeY:
    case rai::JT_hingeZ: {

      PxD6Joint* desc = PxD6JointCreate(*physxSingleton().mPhysics, actors(from->ID), A, actors(jj->frame->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->frame->ats.find<arr>("drive")) {
        arr drive_values = jj->frame->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eTWIST, drive);
      }

      if(jj->frame->ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);

        arr limits = jj->frame->ats.get<arr>("limit");
        PxJointAngularLimitPair limit(limits(0), limits(1), 0.1f);
        limit.restitution = limits(2);
        //limit.spring = limits(3);
        //limit.damping= limits(4);
        //}
        desc->setTwistLimit(limit);
      } else {
        desc->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      }

      if(jj->frame->ats.find<arr>("drive")) {
        arr drive_values = jj->frame->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, false);
        desc->setDrive(PxD6Drive::eTWIST, drive);
        //desc->setDriveVelocity(PxVec3(0, 0, 0), PxVec3(5e-1, 0, 0));
      }
      joints(jj->frame->ID) = desc;
    }
    break;
    case rai::JT_rigid: {
      // PxFixedJoint* desc =
      PxFixedJointCreate(*physxSingleton().mPhysics, actors(jj->from()->ID), A, actors(jj->frame->ID), B.getInverse());
      // desc->setProjectionLinearTolerance(1e10);
      // desc->setProjectionAngularTolerance(3.14);
    }
    break;
    case rai::JT_trans3: {
      break;
    }
    case rai::JT_transXYPhi: {
      PxD6Joint* desc = PxD6JointCreate(*physxSingleton().mPhysics, actors(jj->from()->ID), A, actors(jj->frame->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

      joints(jj->frame->ID) = desc;
      break;
    }
    case rai::JT_transX:
    case rai::JT_transY:
    case rai::JT_transZ: {
      PxD6Joint* desc = PxD6JointCreate(*physxSingleton().mPhysics, actors(jj->from()->ID), A, actors(jj->frame->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(jj->frame->ats.find<arr>("drive")) {
        arr drive_values = jj->frame->ats.get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eX, drive);
      }

      if(jj->frame->ats.find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);

        arr limits = jj->frame->ats.get<arr>("limit");
        PxJointLinearLimit limit(physxSingleton().mPhysics->getTolerancesScale(), limits(0), 0.1f);
        limit.restitution = limits(2);
        //if(limits(3)>0) {
        //limit.spring = limits(3);
        //limit.damping= limits(4);
        //}
        desc->setLinearLimit(limit);
      } else {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      }
      joints(jj->frame->ID) = desc;
    }
    break;
    default:
      NIY;
  }
}
void PhysXInterface_self::lockJoint(PxD6Joint* joint, rai::Joint* rai_joint) {
  joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
  joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
  joint->setTwistLimit(PxJointAngularLimitPair(joint->getTwist()-.001, joint->getTwist()+.001));
}
void PhysXInterface_self::unlockJoint(PxD6Joint* joint, rai::Joint* rai_joint) {
  switch(rai_joint->type) {
    case rai::JT_hingeX:
    case rai::JT_hingeY:
    case rai::JT_hingeZ:
      //joint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
      //joint->setLinearLimit(PxJointLimit(rai_joint->Q.rot.getRad(), rai_joint->Q.rot.getRad()));
      joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      break;
    case rai::JT_transX:
    case rai::JT_transY:
    case rai::JT_transZ:
      //joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
      joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      break;
    default:
      break;
  }
}

void PhysXInterface_self::addLink(rai::Frame* f, int verbose) {
  //-- collect all shapes of that link
  FrameL parts = {f};
  f->getRigidSubFrames(parts);
  bool hasShape=false;
  for(rai::Frame* p:parts) if(p->shape && p->getShape().type()!=rai::ST_marker && p->shape->alpha()==1.) { hasShape=true; break; }

  //-- decide on the type
  rai::BodyType type = rai::BT_static;
  if(hasShape) {
    if(f->joint)   type = rai::BT_kinematic;
    if(f->inertia) type = f->inertia->type;
  }
  actorTypes(f->ID) = type;
  if(verbose>0) LOG(0) <<"adding link anchored at '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type);

  //-- create a PhysX actor
  PxRigidDynamic* actor=nullptr;
  switch(type) {
    case rai::BT_static:
      actor = (PxRigidDynamic*) physxSingleton().mPhysics->createRigidStatic(conv_Transformation2PxTrans(f->ensure_X()));
      break;
    case rai::BT_dynamic:
      actor = physxSingleton().mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
      break;
    case rai::BT_kinematic:
      actor = physxSingleton().mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
      actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
      break;
    case rai::BT_none:
      HALT("this shoudn't be none BT!?")
      //      actor = mPhysics->createRigidDynamic(RaiTrans2PxTrans(b->X));
      break;
  }
  CHECK(actor, "create actor failed!");

//  cout <<RAI_HERE <<"adding '" <<b->name <<"' as " <<rai::Enum<rai::BodyType>(type) <<" with parts";
//  for(auto* p:parts) cout <<' ' <<p->name; cout <<endl;

  //-- for each shape create its geometry with the respective material
  for(rai::Frame* p: parts) {
    rai::Shape* s = p->shape;
    if(!s) continue;
    if(s->frame.name.startsWith("coll_")) continue; //these are the 'pink' collision boundary shapes..
    if(s->alpha()<1.) continue; //no transparent objects!
    PxGeometry* geometry;
    switch(s->type()) {
      case rai::ST_box: {
        geometry = new PxBoxGeometry(.5*s->size(0), .5*s->size(1), .5*s->size(2));
      }
      break;
      case rai::ST_sphere: {
        geometry = new PxSphereGeometry(s->size(-1));
      }
      break;
//      case rai::ST_capsule: {
//        geometry = new PxCapsuleGeometry(s->size(-1), .5*s->size(-2));
//      }
//      break;
      case rai::ST_capsule:
      case rai::ST_cylinder:
      case rai::ST_ssBox:
      case rai::ST_ssCvx:
      case rai::ST_mesh: {
        // Note: physx can't decompose meshes itself.
        // Physx doesn't support triangle meshes in dynamic objects! See:
        // file:///home/mtoussai/lib/PhysX/Documentation/PhysXGuide/Manual/Shapes.html
        // We have to decompose the meshes "by hand" and feed them to PhysX.

        // PhysX uses float for the vertices
        floatA Vfloat;

        Vfloat.clear();
        copy(Vfloat, s->mesh().V); //convert vertices from double to float array..
        PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                       *physxSingleton().mPhysics, *physxSingleton().mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                       PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX);
        geometry = new PxConvexMeshGeometry(triangleMesh);
      } break;
      case rai::ST_marker: {
        geometry = nullptr;
      } break;
      default:
        NIY;
    }

    if(geometry) {
      //-- decide/create a specific material
      PxMaterial* mMaterial = defaultMaterial;
      double fric=-1.;
      if(s->frame.ats.get<double>(fric, "friction")) {
        double rest=s->frame.ats.get<double>("restitution", 0.1);
        mMaterial = physxSingleton().mPhysics->createMaterial(fric, fric, rest);
      }

      PxShape* shape = actor->createShape(*geometry, *mMaterial);
      if(&s->frame!=f) {
        if(s->frame.parent==f) {
          shape->setLocalPose(conv_Transformation2PxTrans(s->frame.get_Q()));
        } else {
          rai::Transformation rel = p->ensure_X() / f->ensure_X();
          shape->setLocalPose(conv_Transformation2PxTrans(rel));
        }
      }
      CHECK(shape, "create shape failed!");
    }
  }

  if(type != rai::BT_static) {
    if(f->inertia && f->inertia->mass>0.) {
      PxRigidBodyExt::updateMassAndInertia(*actor, f->inertia->mass);
    } else {
      PxRigidBodyExt::updateMassAndInertia(*actor, 1.f);
    }
    actor->setAngularDamping(0.25);
  }
  gScene->addActor(*actor);

  actor->userData = f;
  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = actor;
}

void PhysXInterface::ShutdownPhysX() {
  //  self->mMaterial
  //  self->plane
  //  self->planeShape

  for(PxRigidActor* a: self->actors) if(a) {
      self->gScene->removeActor(*a);
      a->release();
    }
//  if(self->connection) {
//    self->connection->release();
//    self->connection=nullptr;
//  }
  if(self->gScene) {
    self->gScene->release();
    self->gScene = nullptr;
  }
  if(self->gl) {
    delete self->gl;
    self->gl=nullptr;
  }

//  mCooking->release();
//  mPhysics->release();
  //  mFoundation->release();
}

void DrawActor(PxRigidActor* actor, rai::Frame* frame) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;

  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape* shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    rai::Shape* s = frame->shape;
    if(!s) s = frame->children.elem(0)->shape;
    if(s) glColor(s->mesh().C);

    rai::Transformation f;
    double mat[16];
    PxTrans2raiTrans(f, PxShapeExt::getGlobalPose(*shape, *actor));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    //cout <<"drawing shape " <<body->name <<endl;
    switch(shape->getGeometryType()) {
      case PxGeometryType::eBOX: {
        PxBoxGeometry g;
        shape->getBoxGeometry(g);
        //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
        glDrawBox(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
      } break;
      case PxGeometryType::eSPHERE: {
        PxSphereGeometry g;
        shape->getSphereGeometry(g);
        glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCAPSULE: {
        PxCapsuleGeometry g;
        shape->getCapsuleGeometry(g);
        glDrawCappedCylinder(g.radius, g.halfHeight*2);
      } break;
      case PxGeometryType::eCONVEXMESH: {
#if 1
        PxConvexMeshGeometry g;
        shape->getConvexMeshGeometry(g);
        floatA Vfloat((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices(), true); //reference!
        rai::Mesh mesh;
        copy(mesh.V, Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(), 3);
        mesh.makeConvexHull();
        mesh.glDraw(NoOpenGL);
#else
        self->mesh.glDraw();
#endif
      } break;

      default:
        RAI_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw(OpenGL&) {
  for(PxRigidActor* a: self->actors) {
    if(a) {
      rai::Frame* f = (rai::Frame*)a->userData;
      DrawActor(a, f);
    }
  }
}

void PhysXInterface::watch(bool pause, const char* txt) {
  NIY;
//  if(!s->gl) {
//    self->gl = new OpenGL("PHYSX direct");
//    self->gl->add(glStandardScene, nullptr);
//    self->gl->add(*this);
//    self->gl->camera.setDefault();
//  }
//  if(pause) self->gl->watch(txt);
//  else self->gl->update(txt);
}

void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxRigidBody* actor = (PxRigidBody*)(self->actors(b->ID));  // dynamic_cast fails for missing RTTI in physx
  actor->addForce(px_force);
}

void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos) {
  PxVec3 px_force = PxVec3(force.x, force.y, force.z);
  PxVec3 px_pos = PxVec3(pos.x, pos.y, pos.z);
  PxRigidBody* actor = (PxRigidBody*)(self->actors(b->ID));
  PxRigidBodyExt::addForceAtPos(*actor, px_force, px_pos);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //RAI_PHYSX

#include "kin_physx.h"
PhysXInterface::PhysXInterface(const rai::Configuration& C, int verbose) : self(nullptr) { NICO }
PhysXInterface::~PhysXInterface() { NICO }

void PhysXInterface::step(double tau) { NICO }
void PhysXInterface::pushKinematicStates(const FrameL& frames) { NICO }
void PhysXInterface::pushFullState(const FrameL& frames, const arr& vels, bool onlyKinematic) { NICO }
void PhysXInterface::pullDynamicStates(FrameL& frames, arr& vels) { NICO }
void PhysXInterface::postAddObject(rai::Frame* f) { NICO }

void PhysXInterface::changeObjectType(rai::Frame* f, int _type) { NICO }
void PhysXInterface::setArticulatedBodiesKinematic(const rai::Configuration& C) { NICO }
void PhysXInterface::ShutdownPhysX() { NICO }
void PhysXInterface::watch(bool pause, const char* txt) { NICO }
void PhysXInterface::glDraw(OpenGL&) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos) { NICO }

void glPhysXInterface(void* classP) { NICO }

#endif

#ifdef RAI_PHYSX
RUN_ON_INIT_BEGIN(kin_physx)
rai::Array<PxRigidActor*>::memMove=true;
rai::Array<PxD6Joint*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_physx)
#endif
