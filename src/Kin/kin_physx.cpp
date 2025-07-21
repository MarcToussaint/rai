/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PHYSX

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <PxPhysicsAPI.h>
#include <extensions/PxExtensionsAPI.h>
#include <extensions/PxDefaultErrorCallback.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxShapeExt.h>
#include <foundation/PxMat33.h>
//#include <pvd/PxVisualDebugger.h>
//#include <physxvisualdebuggersdk/PvdConnectionFlags.h>
//#include <PxMat33Legacy.h>
#include <extensions/PxSimpleFactory.h>
#pragma GCC diagnostic pop

#include "kin_physx.h"
#include "frame.h"
#include "../Gui/opengl.h"

//===========================================================================

constexpr float px_gravity = -9.81f;
using namespace physx;

//===========================================================================

rai::Transformation conv_PxTrans2Transformation(const PxTransform& pose) {
  rai::Transformation X;
  X.pos.set(pose.p.x, pose.p.y, pose.p.z);
  X.rot.set(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  return X;
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

//===========================================================================
//stuff from Samples/PxToolkit

namespace PxToolkit {
PxConvexMesh* createConvexMesh(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, PxConvexFlags flags) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = vertCount;
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = verts;
  convexDesc.flags            = flags;

  return cooking.createConvexMesh(convexDesc); //, physics.getPhysicsInsertionCallback());
//  PxDefaultMemoryOutputStream buf;
//  if(!cooking.cookConvexMesh(convexDesc, buf)) return nullptr;
//  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
//  return physics.createConvexMesh(input);
}

PxTriangleMesh* createTriangleMesh32(PxPhysics& physics, PxCooking& cooking, const PxVec3* verts, PxU32 vertCount, const PxU32* indices32, PxU32 triCount) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count     = vertCount;
  meshDesc.points.stride    = 3*sizeof(float);
  meshDesc.points.data      = verts;

  meshDesc.triangles.count    = triCount;
  meshDesc.triangles.stride   = 3*sizeof(uint);
  meshDesc.triangles.data     = indices32;

  PxTolerancesScale scale;

  PxCookingParams params(scale);
  params.meshWeldTolerance = 0.001f;
  params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
  // params.buildTriangleAdjacencies = false;
  // params.buildGPUData = true;

  // params.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_INERTIA;
  // params.meshWeldTolerance = 1e-7f;

  PxSDFDesc sdfDesc;
  float sdfSpacing=-.01;
  if(sdfSpacing > 0.f) {
    sdfDesc.spacing = sdfSpacing;
    sdfDesc.subgridSize = 6; //sdfSubgridSize;
    sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL; //bitsPerSdfSubgridPixel;
    sdfDesc.numThreadsForSdfConstruction = 16;
    meshDesc.sdfDesc = &sdfDesc;
  }

  return PxCreateTriangleMesh(params, meshDesc, physics.getPhysicsInsertionCallback());
  // return cooking.createTriangleMesh(meshDesc); //, physics.getPhysicsInsertionCallback());
}
}

//===========================================================================


struct PhysXInterface_Engine {
  PxFoundation* mFoundation = nullptr;
  PxPhysics* mPhysics = nullptr;
  PxCooking* mCooking = nullptr;
  PxDefaultErrorCallback gDefaultErrorCallback;
  PxDefaultAllocator gDefaultAllocatorCallback;
  PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

  PhysXInterface_Engine(){
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    PxTolerancesScale scale;
    // scale.length = .1;
    // scale.speed = .1;
    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale);
    PxCookingParams cookParams(mPhysics->getTolerancesScale());
    //    cookParams.skinWidth = .001f;
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, cookParams);
    if(!mCooking) HALT("PxCreateCooking failed!");
    if(!mPhysics) HALT("Error creating PhysX3 device.");
    //if(!PxInitExtensions(*mPhysics)) HALT("PxInitExtensions failed!");
  }

  ~PhysXInterface_Engine() {
    mPhysics->release();
    mCooking->release();
    mFoundation->release();
  }
};

PhysXInterface_Engine* core() {
  static PhysXInterface_Engine singleton;
  return &singleton;
}

struct PhysXInterface_self {
  ~PhysXInterface_self();

  FrameL freeFrames;
  FrameL articulationJoints;

  PxScene* gScene = nullptr;
  rai::Array<PxConvexMesh*> meshes;
  rai::Array<PxRigidActor*> actors;
  rai::Array<rai::BodyType> actorTypes;
  rai::Array<PxArticulationAxis::Enum> jointAxis;
  rai::Array<PxJoint*> joints;

  rai::PhysX_Options opt;

  uint stepCount=0;

  PxRigidStatic* plane = nullptr;
  PxMaterial* defaultMaterial = nullptr;
  PxDefaultCpuDispatcher* defaultCpuDispatcher = nullptr;

  rai::Configuration debugConfig;

  void initPhysics();
  void addGround();
  void addLink(rai::Frame* b);
  void addJoint(const rai::Joint* jj);
  void addMultiBody(rai::Frame* base);

  void lockJoint(PxD6Joint* joint, rai::Joint* rai_joint);
  void unlockJoint(PxD6Joint* joint, rai::Joint* rai_joint);

  void prepareLinkShapes(ShapeL& shapes, rai::BodyType& type, rai::Frame* link);
  void addSingleShape(PxRigidActor* actor, rai::Frame* f, rai::Shape* s);
  void addShapesAndInertia(PxRigidBody* actor, ShapeL& shapes, rai::BodyType type, rai::Frame* f);

  void syncDebugConfig();
};

//===========================================================================

PhysXInterface_self::~PhysXInterface_self() {
  //destroy actors in reverse order, including plane
  actors.reverse();
  actors.append(plane);
  for(PxRigidActor* a:  actors) if(a) {
      rai::Array<PxShape*> shapes(a->getNbShapes());
      shapes.setZero();
      a->getShapes(shapes.p, shapes.N);
      for(PxShape* s:shapes){ a->detachShape(*s); s->release(); }

      PxArticulationLink* actor = a->is<PxArticulationLink>();
      if(actor){
        if(actor->getLinkIndex()==0){ //root of an articulation
          PxArticulationReducedCoordinate* art = &actor->getArticulation();
          gScene->removeArticulation(*art);
          art->release();
        }
      }else{
        gScene->removeActor(*a);
        a->release();
      }
    }
  //destropy remaining materials
  {
    rai::Array<PxMaterial*> materials(core()->mPhysics->getNbMaterials());
    materials.setZero();
    core()->mPhysics->getMaterials(materials.p, materials.N);
    for(PxMaterial* m:materials) m->release();
  }
  for(PxConvexMesh* m: meshes){ m->release(); }
  if(gScene) gScene->release();
  if(defaultCpuDispatcher) defaultCpuDispatcher->release();
  //check everything is cleaned
  // LOG(0)
  //     <<core()->mPhysics->getNbConvexMeshes() <<' '
  //     <<core()->mPhysics->getNbTriangleMeshes() <<' '
  //     <<core()->mPhysics->getNbConvexMeshes() <<' '
  //     <<core()->mPhysics->getNbMaterials() <<' '
  //     <<core()->mPhysics->getNbScenes() <<' '
  //     <<core()->mPhysics->getNbShapes() <<' ';
}

void PhysXInterface_self::initPhysics() {
  //-- Create the scene
  PxSceneDesc sceneDesc(core()->mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, px_gravity);
  // sceneDesc.frictionType = PxFrictionType::eTWO_DIRECTIONAL;
  sceneDesc.solverType = PxSolverType::eTGS;
  sceneDesc.bounceThresholdVelocity = .1;
  //sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
  sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
  // sceneDesc.flags |= PxSceneFlag::eENABLE_CCD; //continuous collision detection (requires more enables for each body.. https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/AdvancedCollisionDetection.html )
  //sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
  sceneDesc.gpuMaxNumPartitions = 8;

  if(!sceneDesc.cpuDispatcher) {
    defaultCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!defaultCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = defaultCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = core()->gDefaultFilterShader;
  }

  gScene = core()->mPhysics->createScene(sceneDesc);
  if(!gScene) {
    cerr << "createScene failed!" << endl;
  }

  gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

  //-- Create objects
  defaultMaterial = core()->mPhysics->createMaterial(opt.defaultFriction, opt.defaultFriction, opt.defaultRestitution);
}

void PhysXInterface_self::addGround() {
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

  plane = core()->mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");

  PxShape* planeShape = core()->mPhysics->createShape(PxPlaneGeometry(), *defaultMaterial);
  plane->attachShape(*planeShape);
  CHECK(planeShape, "create shape failed!");
  gScene->addActor(*plane);

  if(opt.verbose>0) LOG(0) <<"... done starting PhysX engine";
  if(opt.verbose>0) LOG(0) <<"creating Configuration within PhysX ...";
}

void PhysXInterface_self::addLink(rai::Frame* f) {
  ShapeL shapes;
  rai::BodyType type;
  prepareLinkShapes(shapes, type, f);

  if(!shapes.N) return;

  if(f->joint && !f->joint->isPartBreak) type=rai::BT_dynamic;

  if(opt.verbose>0) {
    rai::String str;
    str <<"adding link '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type) <<" with " <<shapes.N <<" shapes (";
    for(rai::Shape* s:shapes) str <<' ' <<s->frame.name;
    str <<")";
    if(f->inertia) str <<" and mass " <<f->inertia->mass;
    LOG(0) <<str;
  }

  //-- create a PhysX actor
  PxRigidDynamic* actor=nullptr;
  if(type==rai::BT_static) {
    actor = (PxRigidDynamic*) core()->mPhysics->createRigidStatic(conv_Transformation2PxTrans(f->ensure_X()));
  } else if(type==rai::BT_dynamic) {
    actor = core()->mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
  } else if(type==rai::BT_kinematic) {
    actor = core()->mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
  } else NIY;
  CHECK(actor, "create actor failed!");

  addShapesAndInertia(actor, shapes, type, f);

  double angularDamping = opt.angularDamping;
  if(f->ats && f->ats->find<double>("angularDamping")) {
    angularDamping = f->ats->get<double>("angularDamping");
  }
  actor->setAngularDamping(angularDamping);

  gScene->addActor(*actor);

  actor->userData = f;
  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = actor;
  actorTypes(f->ID) = type;
}

#if 1
void PhysXInterface_self::addJoint(const rai::Joint* jj) {

  rai::Transformation rel=0;
  rai::Frame* to = jj->frame;
  rai::Frame* from = jj->frame->parent->getUpwardLink(rel);

  LOG(0) <<"ADDING JOINT " <<from->name <<'-' <<to->name <<" of type " <<jj->type <<" with rel " <<rel;

//  if(!to->inertia || !from || !from->inertia) return;
//  CHECK(to->inertia, "this joint belongs to a frame '" <<to->name <<"' without inertia");
  CHECK(from, "this joint ('" <<to->name <<"') links from nullptr");
//  CHECK(from->inertia, "this joint ('" <<to->name <<"') links from a frame '" <<from->name <<"' without inertia");

  PxTransform A = conv_Transformation2PxTrans(rel);
  PxTransform B = Id_PxTrans();
  switch(jj->type) {
    case rai::JT_free: //do nothing
      break;
    case rai::JT_hingeX:
    case rai::JT_hingeY:
    case rai::JT_hingeZ: {

      //PxD6Joint* desc = PxD6JointCreate(*core()->mPhysics, actors(from->ID), A, actors(to->ID), B.getInverse());
      PxRevoluteJoint* joint = PxRevoluteJointCreate(*core()->mPhysics, actors(from->ID), A, actors(to->ID), B.getInverse());
      joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      joint->setDriveForceLimit(1e1);
//      cout <<joint->getDriveVelocity() <<endl;
//      joint->setDriveVelocity(0.f);
      if(jj->limits.N) {
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
        joint->setLimit({(float)jj->limits(0), (float)jj->limits(1)});
      }
      CHECK(joint, "PhysX joint creation failed.");

#if 0
      if(to->ats && to->ats->find<arr>("drive")) {
        arr drive_values = to->ats->get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        joint->setDrive(PxD6Drive::eTWIST, drive);
      }

      if(to->ats && to->ats->find<arr>("limit")) {
        joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);

        arr limits = to->ats->get<arr>("limit");
        PxJointAngularLimitPair limit(limits(0), limits(1), 0.1f);
        limit.restitution = limits(2);
        //limit.spring = limits(3);
        //limit.damping= limits(4);
        //}
        joint->setTwistLimit(limit);
      } else {
        joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
      }

      if(to->ats && to->ats->find<arr>("drive")) {
        arr drive_values = to->ats->get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, false);
        joint->setDrive(PxD6Drive::eTWIST, drive);
        //desc->setDriveVelocity(PxVec3(0, 0, 0), PxVec3(5e-1, 0, 0));
      }
#endif
      joints.append(joint);
    }
    break;
    case rai::JT_rigid: {
      PxTransform A = conv_Transformation2PxTrans(rel * to->get_Q()); //add the current relative transform!
      PxFixedJoint* joint = PxFixedJointCreate(*core()->mPhysics, actors(from->ID), A, actors(to->ID), B.getInverse());
      // desc->setProjectionLinearTolerance(1e10);
      // desc->setProjectionAngularTolerance(3.14);
      joints.append(joint);
    }
    break;
    case rai::JT_trans3: {
      break;
    }
    case rai::JT_transXYPhi: {
#if 0
      PxD6Joint* desc = PxD6JointCreate(*core()->mPhysics, actors(jj->from()->ID), A, actors(to->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

      joints.append(desc);
#endif
      break;
    }
    case rai::JT_transX:
    case rai::JT_transY:
    case rai::JT_transZ: {
#if 0
      PxD6Joint* desc = PxD6JointCreate(*core()->mPhysics, actors(jj->from()->ID), A, actors(to->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(to->ats && to->ats->find<arr>("drive")) {
        arr drive_values = to->ats->get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eX, drive);
      }

      if(to->ats && to->ats->find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);

        arr limits = to->ats->get<arr>("limit");
        PxJointLinearLimit limit(core()->mPhysics->getTolerancesScale(), limits(0), 0.1f);
        limit.restitution = limits(2);
        //if(limits(3)>0) {
        //limit.spring = limits(3);
        //limit.damping= limits(4);
        //}
        desc->setLinearLimit(limit);
      } else {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      }
      joints.append(desc);
#endif
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
#endif

void PhysXInterface_self::addMultiBody(rai::Frame* base) {
  //CHECK(!base->parent || (base->joint && base->joint->type==rai::JT_rigid) || (base->joint && base->inertia), "base needs to be either rigid or with inertia");

  //multibody options
  bool multibody_fixedBase = base->getAts().get<bool>("multibody_fixedBase", true);
  bool multibody_gravity = base->getAts().get<bool>("multibody_gravity", false);

  if(opt.verbose>0) {
    LOG(0) <<"adding multibody with base '" <<base->name <<"' fixedBase: " <<multibody_fixedBase <<" gravity: " <<multibody_gravity;
  }

  //-- collect all links/joints for that root
  FrameL F = {base};
  base->getSubtree(F);
  FrameL links = {base};
  for(auto* f:F) { if(f->joint && !f->joint->isPartBreak) links.append(f); }
  intA parents(links.N);
  parents = -1;
  for(uint i=1; i<links.N; i++) {
    articulationJoints.append(links(i));
    rai::Frame* p = links(i)->parent->getUpwardLink();
    parents(i) = links.findValue(p);
    if(parents(i)==-1) {
      //special case: the multibody contains a partbreak link - skip over it (same as assuming fixed)
      CHECK(p->joint && p->joint->isPartBreak, "");
      p = p->parent->getUpwardLink();
      parents(i) = links.findValue(p);
    }
    CHECK(parents(i)>=0, "");
  }
  rai::Array<PxArticulationLink*> linksPx(links.N);
  linksPx = NULL;

  PxArticulationReducedCoordinate* articulation = core()->mPhysics->createArticulationReducedCoordinate();
  articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, multibody_fixedBase);
  articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
  //articulation->setSolverIterationCounts(minPositionIterations, minVelocityIterations);
  //articulation->setMaxCOMLinearVelocity(maxCOMLinearVelocity);

  for(uint i=0; i<links.N; i++) {
    //prepare link shapes, inertia, and type
    rai::Frame* f = links(i);
    if(i!=0) CHECK(f->joint, "");
    if(opt.verbose>0) cout <<"-- kin_physx.cpp:  adding multibody link '" <<f->name <<"'" <<endl;

    ShapeL shapes;
    rai::BodyType type;
    prepareLinkShapes(shapes, type, f);
    if(i>0) type = rai::BT_dynamic;

    //create link
    PxArticulationLink* actor = 0;
    actor = articulation->createLink(i==0?NULL:linksPx(parents(i)), conv_Transformation2PxTrans(f->ensure_X()));
    linksPx(i) = actor;
    actor->userData = f;
    CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
    actors(f->ID) = actor;
    actorTypes(f->ID) = type;

    addShapesAndInertia(actor, shapes, type, f);

    //get options for link and motor
    rai::Graph *ats = &f->getAts();
    if(f->joint && f->joint->mimic) ats = &f->joint->mimic->frame->getAts();
    bool noMotor = ats->get<bool>("noMotor", false);
    double motorKp = ats->get<double>("motorKp", opt.motorKp);
    double motorKd = ats->get<double>("motorKd", opt.motorKd);
    double motorLambda =  ats->get<double>("motorLambda", -1.);
    if(motorLambda>0.){
      double motorMass =  ats->get<double>("motorMass", f->inertia->mass);
      double dampingRatio = 1.;
      double freq = 1./motorLambda;
      motorKp = motorMass*freq*freq;
      motorKd = 2.*motorMass*dampingRatio*freq;
    }
    double angularDamping = ats->get<double>("angularDamping", opt.angularDamping);
    double jointFriction = ats->get<double>("jointFriction", opt.jointFriction);

    actor->setLinearDamping(0.);
    actor->setAngularDamping(angularDamping);
    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, !multibody_gravity);

    if(opt.verbose>0) {
      rai::String str;
      str <<"    multibody link '" <<f->name <<"' is " <<rai::Enum<rai::BodyType>(type) <<" with " <<shapes.N <<" shapes (";
      for(rai::Shape* s:shapes) str <<' ' <<s->frame.name;
      str <<")";
      if(f->joint){
        str <<" and joint " <<f->joint->type;
        if(noMotor) str <<" (no motor!)";
        else str <<" (Kp=" <<motorKp <<" Kd=" <<motorKd <<")";
        if(f->joint->limits.N) str <<" limits: " <<f->joint->limits;
        if(!f->joint->active) str <<"(inactive)";
      }
      if(f->inertia) str <<" and mass " <<f->inertia->mass;
      cout <<"-- kin_physx.cpp:" <<str <<endl;
    }

    if(i>0) {
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
      rai::Frame* prevLink = links(parents(i));
      rai::Transformation relA = f->parent->ensure_X() / prevLink->ensure_X();
      rai::Transformation relB = 0; //-f->get_Q();
      joint->setParentPose(conv_Transformation2PxTrans(relA));
      joint->setChildPose(conv_Transformation2PxTrans(relB));
      joint->setFrictionCoefficient(jointFriction);

      PxArticulationAxis::Enum axis = PxArticulationAxis::eCOUNT;
      PxArticulationJointType::Enum type = PxArticulationJointType::eUNDEFINED;
      switch(f->joint->type) {
        case rai::JT_hingeX: {
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eTWIST;
          break;
        }
        case rai::JT_hingeY: {
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eSWING1;
          break;
        }
        case rai::JT_hingeZ: {
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eSWING2;
          break;
        }
        case rai::JT_transX: {
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eX;
          break;
        }
        case rai::JT_transY: {
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eY;
          break;
        }
        case rai::JT_transZ: {
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eZ;
          break;
        }
        case rai::JT_transXY: {
          NIY; //ePRISMATIC is only for SINGLE dof!
          break;
        }
        case rai::JT_quatBall:{
          type = PxArticulationJointType::eSPHERICAL;
          joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE); //eLIMITED
          joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE); //eLIMITED
          joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE); //eLIMITED
          auto vec = f->get_Q().rot.getVector();
          joint->setJointPosition(PxArticulationAxis::eTWIST, vec.x);
          joint->setJointPosition(PxArticulationAxis::eSWING1, vec.y);
          joint->setJointPosition(PxArticulationAxis::eSWING2, vec.z);
          axis = PxArticulationAxis::eCOUNT;
          break;
        }
        default: NIY;
      }
      joint->setJointType(type);
      if(axis!=PxArticulationAxis::eCOUNT){
        if(f->joint->limits.N){
          joint->setMotion(axis, PxArticulationMotion::eLIMITED);
          if(f->joint->scale>0.){
            joint->setLimitParams(axis, {float(f->joint->scale*f->joint->limits.elem(0)), float(f->joint->scale*f->joint->limits.elem(1))});
          }else{
            joint->setLimitParams(axis, {float(f->joint->scale*f->joint->limits.elem(1)), float(f->joint->scale*f->joint->limits.elem(0))});
          }
        }else{
          joint->setMotion(axis, PxArticulationMotion::eFREE);
        }
        joint->setJointPosition(axis, f->joint->scale*f->joint->get_q());
      }
      jointAxis(f->ID) = axis;

      if(f->joint->mimic) {
//        NIY;
        //        btVector3 pivot(0,1,0);
        //        btMatrix3x3 frame(1,0,0,0,1,0,0,0,1);
        //        //HARD CODED: mimicer is the previous one
        //        btMultiBodyConstraint* gearCons = new btMultiBodyGearConstraint(multibody, i-1, multibody, i-2, pivot, pivot, frame, frame);
        //        gearCons->setGearRatio(-1); //why needed? already flipped in config..
        //        gearCons->setErp(0.1);
        //        gearCons->setMaxAppliedImpulse(50);
        //        world->addMultiBodyConstraint(gearCons);
      }

      if(!noMotor && axis!=PxArticulationAxis::eCOUNT) { //only 1D joints have drives!
        PxArticulationDrive posDrive;
        posDrive.stiffness = motorKp;                      // the spring constant driving the joint to a target position
        posDrive.damping = motorKd;                        // the damping coefficient driving the joint to a target velocity
        posDrive.maxForce = PX_MAX_F32; //1e10f;                              // force limit for the drive
        posDrive.driveType = PxArticulationDriveType::eFORCE;  // make the drive output be a force/torque (default)
        joint->setDriveParams(axis, posDrive);
        joint->setDriveVelocity(axis, 0.);
        joint->setDriveTarget(axis, f->joint->scale*f->joint->get_q());
      }
    }
  }

  gScene->addArticulation(*articulation);

  //articulation->updateKinematic(PxArticulationKinematicFlag::ePOSITION);

  if(opt.verbose>0) {
    LOG(0) <<"... done with multibody with base '" <<base->name <<"'";
  }
}

void PhysXInterface_self::prepareLinkShapes(ShapeL& shapes, rai::BodyType& type, rai::Frame* link) {
  CHECK(!link->parent || link->joint, "this is not a link");

  FrameL sub = {link};
  link->getRigidSubFrames(sub, false);

  //-- collect all shapes of that link
  shapes.clear();
  {
    for(rai::Frame* ch: sub) {
      if(ch->shape && ch->getShape().type()!=rai::ST_marker
          && ch->getShape().type()!=rai::ST_camera){ //is a candidate
        if(ch->ats && ch->ats->find<bool>("simulate")){
          if(ch->ats->get<bool>("simulate")) shapes.append(ch->shape);
        }else{
          if(ch->getShape().alpha()==1.) shapes.append(ch->shape);
          else{}//transparent and no simulate flag
        }
      }
    }
  }

  //-- prepare inertia
  bool subHaveInertia=false;
  for(rai::Frame* ch: sub) if(ch->inertia && ch!=link) { subHaveInertia=true; break; }
  if(subHaveInertia) {
    if(opt.verbose>0) cout <<"-- kin_physx.cpp:    computing compound inertia for link frame '" <<link->name <<endl;
    link->computeCompoundInertia();
  }
  if(!link->inertia) {
    bool hasMass = link->standardizeInertias();
    if(hasMass){
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    link '" <<link->name <<"' does not have inertia -> computing standard inertias (alternatively, define inertias for links before starting physx)" <<endl;
    }else{
      if(opt.verbose>0) LOG(0) <<"link '" <<link->name <<"' has no mass -> becomes static";
    }
    // PxRigidBodyExt::updateMassAndInertia(*actor, 1000.f);
    // if(!f->inertia) new rai::Inertia(*f);
    // f->inertia->mass = actor->getMass();
    // f->inertia->matrix.setDiag(conv_PxVec3_arr(actor->getMassSpaceInertiaTensor()));
    // f->inertia->com = conv_PxVec3_arr(actor->getCMassLocalPose().p);
    // //cout <<*f->inertia <<" m:" <<actor->getMass() <<" I:" <<conv_PxVec3_arr(actor->getMassSpaceInertiaTensor()) <<endl;
  }
  if(link->inertia && link->inertia->mass<1e-12) {
    LOG(-1) <<"link '" <<link->name <<"' has zero mass -> making it minimally .001";
    link->inertia->mass = .001;
    link->inertia->com.setZero();
    link->inertia->matrix.setDiag({.001,.001,.001});
  }

  //-- decide on the type
  type = rai::BT_static;
  if(link->joint)   type = rai::BT_kinematic;
  if(link->inertia) type = rai::BT_dynamic;
}

void PhysXInterface_self::addSingleShape(PxRigidActor* actor, rai::Frame* f, rai::Shape* s) {
  std::shared_ptr<PxGeometry> geometry;
  double paddingRadius=0.;

  switch(s->type()) {
    case rai::ST_box: {
      geometry = make_shared<PxBoxGeometry>(.5*s->size(0), .5*s->size(1), .5*s->size(2));
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape box '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
    } break;
    case rai::ST_ssBox: {
      double r = s->size(3);
      geometry = make_shared<PxBoxGeometry>(.5*s->size(0)-r, .5*s->size(1)-r, .5*s->size(2)-r);
      paddingRadius = r;
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape ssBox '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
    } break;
    case rai::ST_sphere: {
      geometry = make_shared<PxSphereGeometry>(s->size(0));
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape sphere '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
    } break;
    case rai::ST_capsule: { //GRRRR... capsule are extended along x-axis in physx.. all inconsistent. use explicit mesh
      // geometry = make_shared<PxCapsuleGeometry>(s->size(1), .5*s->size(0));
      floatA Vfloat = rai::convert<float>(s->mesh().V);
      PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
          *core()->mPhysics, *core()->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
          PxConvexFlag::eCOMPUTE_CONVEX);
      meshes.append(triangleMesh);
      geometry = make_shared<PxConvexMeshGeometry>(triangleMesh);
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape capsule '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
    } break;
    // case rai::ST_cylinder:{
    //   NIY;
    // } break;
    case rai::ST_ssCylinder:
    case rai::ST_ssCvx:
    default: {
      // CHECK(s->sscCore().N, "physx needs a convex collision shape, frame: " <<s->frame.name <<" (cvxParts are disabled->convert them to child frames)");
      if(s->sscCore().N){ //has a convex collision core
        floatA Vfloat = rai::convert<float>(s->sscCore());
        PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
            *core()->mPhysics, *core()->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
            PxConvexFlag::eCOMPUTE_CONVEX);
        geometry = make_shared<PxConvexMeshGeometry>(triangleMesh);
        paddingRadius = s->coll_cvxRadius;
        if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape cvx mesh '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
      }else if(s->mesh().V.N){ //a non-convex mesh!
        floatA Vfloat = rai::convert<float>(s->mesh().V);
        uintA& Tri = s->mesh().T;
        PxTriangleMesh* triangleMesh = PxToolkit::createTriangleMesh32(
            *core()->mPhysics, *core()->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0, (PxU32*) Tri.p, Tri.d0);
        geometry = make_shared<PxTriangleMeshGeometry>(triangleMesh);
        if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape non-cvx mesh '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
      }else NIY;
      } break;
    case rai::ST_sdf:
    // default: {
    //   rai::Mesh& M = s->mesh();
    //   if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding shape mesh '" <<s->frame.name <<"' (" <<s->type() <<") as mesh" <<endl;
    //   if(M.cvxParts.N) {
    //     floatA Vfloat;
    //     if(opt.verbose>0) cout <<"-- kin_physx.cpp:    creating " <<M.cvxParts.N <<" convex parts for shape " <<s->frame.name <<endl;
    //     for(uint i=0; i<M.cvxParts.N; i++) {
    //       Vfloat.clear();
    //       int start = M.cvxParts(i);
    //       int end = i+1<M.cvxParts.N ? M.cvxParts(i+1)-1 : -1;
    //       copy(Vfloat, M.V({start, end+1}));
    //       PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
    //                                      *core()->mPhysics, *core()->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
    //                                      PxConvexFlag::eCOMPUTE_CONVEX);
    //       meshes.append(triangleMesh);
    //       geometry = make_shared<PxConvexMeshGeometry>(triangleMesh);
    //       PxShape* shape = core()->mPhysics->createShape(*geometry, *defaultMaterial);
    //       actor->attachShape(*shape);
    //       if(&s->frame!=f) {
    //         if(s->frame.parent==f) {
    //           shape->setLocalPose(conv_Transformation2PxTrans(s->frame.get_Q()));
    //         } else {
    //           rai::Transformation rel = s->frame.ensure_X() / f->ensure_X();
    //           shape->setLocalPose(conv_Transformation2PxTrans(rel));
    //         }
    //       }
    //     }
    //     geometry.reset();
    //   } else {
    //     if(opt.verbose>0) cout <<"-- kin_physx.cpp:    using cvx hull of mesh as no decomposition (M.cvxParts) is available" <<endl;
    //     floatA Vfloat = rai::convert<float>(s->mesh().V);
    //     PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
    //                                    *core()->mPhysics, *core()->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
    //                                    PxConvexFlag::eCOMPUTE_CONVEX);
    //     meshes.append(triangleMesh);
    //     geometry = make_shared<PxConvexMeshGeometry>(triangleMesh);
    //   }
    // } break;
    case rai::ST_camera:
    case rai::ST_pointCloud:
    case rai::ST_marker: {
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    skipping shape '" <<s->frame.name <<"' (" <<s->type() <<")" <<endl;
      geometry = nullptr;
    } break;
    // default:
    //   LOG(0) <<"can't create shape of type:" <<s->type();
    //   NIY;
  }

  if(geometry) {
    //decide/create a specific material
    PxMaterial* mMaterial = defaultMaterial;
    if(s->frame.ats &&
        (s->frame.ats->find<double>("friction") || s->frame.ats->find<double>("restitution"))) {
      double fric=s->frame.ats->get<double>("friction", opt.defaultFriction);
      double rest=s->frame.ats->get<double>("restitution", opt.defaultRestitution);
      //LOG(0) <<" shape " <<s->frame.name <<" friction: " <<fric <<" restitution: " <<rest;
      mMaterial = core()->mPhysics->createMaterial(fric, fric, rest);
    }

    //create the shape
    PxShape* shape = core()->mPhysics->createShape(*geometry, *mMaterial);
    CHECK(shape, "create shape failed!");

    //attach to actor
    actor->attachShape(*shape);
    if(&s->frame!=f) {
      if(s->frame.parent==f) {
        shape->setLocalPose(conv_Transformation2PxTrans(s->frame.get_Q()));
      } else {
        rai::Transformation rel = s->frame.ensure_X() / f->ensure_X();
        shape->setLocalPose(conv_Transformation2PxTrans(rel));
      }
    }

    //set contact parameters
    //double contactOffset = shape->getContactOffset();
    //double restOffset = shape->getRestOffset();
    //LOG(0) <<"contact params: " <<contactOffset <<' ' <<restOffset;
    shape->setContactOffset(paddingRadius+.02);
    shape->setRestOffset(paddingRadius);
  }
}

void PhysXInterface_self::addShapesAndInertia(PxRigidBody* actor, ShapeL& shapes, rai::BodyType type, rai::Frame* f) {
  //-- add each shape to the actor
  for(rai::Shape* s: shapes) addSingleShape(actor, f, s);

  //-- set inertia
  if(type != rai::BT_static) {
    CHECK(f->inertia, "dynamic links need inertia! (frame: " <<f->name <<")");
    CHECK(f->inertia->mass>0., "dynamic links need inertia! (frame: " <<f->name <<")");
    actor->setMass(f->inertia->mass);
    if(f->inertia->com.isZero && f->inertia->matrix.isDiagonal()){
      actor->setMassSpaceInertiaTensor({float(f->inertia->matrix.m00), float(f->inertia->matrix.m11), float(f->inertia->matrix.m22)});
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding mass " <<f->inertia->mass <<" inertia " <<f->inertia->matrix.getDiag() <<" with zero trans " <<endl;
    }else{
      arr Idiag;
      rai::Transformation t = f->inertia->getDiagTransform(Idiag);
      if(!t.pos.isZero || !t.rot.isZero){
        actor->setCMassLocalPose(conv_Transformation2PxTrans(t));
      }
      actor->setMassSpaceInertiaTensor({float(Idiag(0)), float(Idiag(1)), float(Idiag(2))});
      if(opt.verbose>0) cout <<"-- kin_physx.cpp:    adding mass " <<f->inertia->mass <<" inertia " <<Idiag <<" trans " <<t <<endl;
    }
    //      //cout <<*f->inertia <<" m:" <<actor->getMass() <<" I:" <<conv_PxVec3_arr(actor->getMassSpaceInertiaTensor()) <<endl;
  }
}

void PhysXInterface_self::syncDebugConfig() {
  if(!debugConfig.frames.N){
    for(PxRigidActor* actor: actors) {
      if(actor) {
        rai::Frame* frame = (rai::Frame*)actor->userData;
        rai::Frame* f = debugConfig.addFrame(frame->name);

	PxU32 nShapes = actor->getNbShapes();
	PxShape** shapes=new PxShape*[nShapes];
	//cout <<"#shapes=" <<nShapes;

	actor->getShapes(shapes, nShapes);
	rai::Mesh mesh;

	while(nShapes--) {
	  PxShape* shape = shapes[nShapes];

	  //    // use the color of the first shape of the body for the entire body
	  //    rai::Shape* s = frame->shape;
	  //    if(!s) for(rai::Frame* ch:frame->children) {
	  // if(ch->shape && ch->shape->alpha()==1.) {
	  //   s = ch->shape;
	  //   break;
	  // }
	  //      }
	  //    if(s) glColor(s->mesh().C);

	  //cout <<"drawing shape " <<body->name <<endl;
	  rai::Transformation Q = conv_PxTrans2Transformation(shape->getLocalPose());
	  switch(shape->getGeometryType()) {
	    case PxGeometryType::eBOX: {
	      PxBoxGeometry g;
	      shape->getBoxGeometry(g);
	      //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
	      rai::Mesh m;
	      m.setBox();
	      m.scale(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
	      if(!Q.isZero()) m.transform(Q);
	      mesh.addMesh(m);
	    } break;
	    case PxGeometryType::eSPHERE: {
	      PxSphereGeometry g;
	      shape->getSphereGeometry(g);
	      // glutSolidSphere(g.radius, 10, 10);
	      rai::Mesh m;
	      m.setSphere();
	      m.scale(g.radius);
	      if(!Q.isZero()) m.transform(Q);
	      mesh.addMesh(m);
	    } break;
	    case PxGeometryType::eCAPSULE: {
	      PxCapsuleGeometry g;
	      shape->getCapsuleGeometry(g);
	      // glDrawCappedCylinder(g.radius, g.halfHeight*2);
	      rai::Mesh m;
	      m.setCapsule(g.radius, g.halfHeight*2.);
	      if(!Q.isZero()) m.transform(Q);
	      mesh.addMesh(m);
	    } break;
	    case PxGeometryType::eCONVEXMESH: {
#if 1
	      PxConvexMeshGeometry g;
	      shape->getConvexMeshGeometry(g);
	      floatA Vfloat;
	      Vfloat.referTo((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference!
	      rai::Mesh m;
	      copy(m.V, Vfloat);
	      m.V.reshape(g.convexMesh->getNbVertices(), 3);
	      m.makeConvexHull();
	      if(!Q.isZero()) m.transform(Q);
	      mesh.addMesh(m);
#else
	      self->mesh.glDraw();
#endif
	    } break;
	    case PxGeometryType::eTRIANGLEMESH: {
	      PxTriangleMeshGeometry g;
	      shape->getTriangleMeshGeometry(g);
	      floatA Vfloat;
	      Vfloat.referTo((float*)g.triangleMesh->getVertices(), 3*g.triangleMesh->getNbVertices()).reshape(-1, 3);
	      rai::Mesh m;
	      m.V = rai::convert<double>(Vfloat);
	      if(g.triangleMesh->getTriangleMeshFlags()&PxTriangleMeshFlag::e16_BIT_INDICES) {
		rai::Array<uint16_t> T16;
		T16.referTo((uint16_t*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
		m.T = rai::convert<uint>(T16);
	      } else {
		m.T.setCarray((uint*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
	      }
	      if(!Q.isZero()) m.transform(Q);
	      mesh.addMesh(m);
	    } break;

	    default:
	      RAI_MSG("can't draw this type");
	  }
	}
	delete [] shapes;
	f->setMesh2(mesh);
      }
    }
  }

  uint id=0;
  for(PxRigidActor* actor: actors) {
    if(actor) {
      rai::Frame* frame = (rai::Frame*)actor->userData;
      rai::Frame* f = debugConfig.frames(id++);
      CHECK_EQ(frame->name, f->name, "");

      f->set_X() = conv_PxTrans2Transformation(actor->getGlobalPose());
    }
  }
}

//===========================================================================

PhysXInterface::PhysXInterface(rai::Configuration& C, int verbose, const rai::PhysX_Options* _opt) {
  CHECK(C._state_q_isGood, "PhysX needs joint angles for initialization");

  self = new PhysXInterface_self;

  if(_opt) self->opt = *_opt;
  self->opt.verbose = verbose;

  if(self->opt.verbose>0) LOG(0) <<"starting PhysX engine ...";

  self->initPhysics();

  self->addGround();

  //-- create Configuration equivalent in PhysX
  self->actors.resize(C.frames.N).setZero();
  self->actorTypes.resize(C.frames.N).setZero();
  self->jointAxis.resize(C.frames.N) = PxArticulationAxis::eCOUNT;

  for(rai::Frame* a : C.frames) a->ensure_X();

  FrameL parts = C.getParts();
  for(rai::Frame* f : parts) {
    if(f->ats && f->ats->get<bool>("multibody", false)) {
      self->addMultiBody(f);
    } else {
      self->addLink(f);
    }
    if(self->actorTypes(f->ID)==rai::BT_dynamic){
      self->freeFrames.append(f);
    }
  }

  if(self->opt.verbose>0){
    LOG(0) <<"list of free frames: " <<rai::framesToNames(self->freeFrames);
    LOG(0) <<"list of articulation joints: " <<rai::framesToNames(self->articulationJoints);
    LOG(0) <<"... done creating Configuration within PhysX";
  }
}

PhysXInterface::~PhysXInterface() {
  delete self;
}

const FrameL& PhysXInterface::getFreeFrames(){
  return self->freeFrames;
}

const FrameL& PhysXInterface::getJointFrames(){
  return self->articulationJoints;
}


void PhysXInterface::step(double tau) {
  self->stepCount++;
  self->gScene->simulate(tau);

  //...perform useful work here using previous frame's state data
  while(!self->gScene->fetchResults()) {
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

void PhysXInterface::addRigidJoint(rai::Frame *from, rai::Frame *to) {
  // self->addJoint(from, to, rai::JT_rigid, to->ensure_X()/from->ensure_X());
  PxRigidActor *From = self->actors(from->ID);
  PxRigidActor *To = self->actors(to->ID);
  CHECK(From, from->name <<" it not an actor");
  CHECK(To, to->name <<" it not an actor");

  PxTransform A = conv_Transformation2PxTrans(to->ensure_X()/from->ensure_X());
  PxTransform B = Id_PxTrans();
  PxFixedJoint* joint = PxFixedJointCreate(*core()->mPhysics, From, A, To, B);
  self->joints.append(joint);
}

void PhysXInterface::removeJoint(const rai::Frame* from, const rai::Frame* to) {
  LOG(0) <<"REMOVING JOINT " <<from->name <<'-' <<to->name;
  PxRigidActor *From = self->actors(from->ID);
  PxRigidActor *To = self->actors(to->ID);
  CHECK(From, from->name <<" it not an actor");
  CHECK(To, to->name <<" it not an actor");

  bool found = false;
  for(uint i=self->joints.N;i--;){
    PxJoint *joint = self->joints.elem(i);
    CHECK(joint, "");
    PxRigidActor *a, *b;
    joint->getActors(a, b);
    if(a==From && b==To){
      joint->release();
      self->joints.remove(i);
      found = true;
    }
  }
  if(!found){
    LOG(-1) <<"that joint didn't exist!";
  }
}

void PhysXInterface::postAddObject(rai::Frame* f) {
  while(self->actors.N<=f->ID) self->actors.append(0);
  while(self->actorTypes.N<=f->ID) self->actorTypes.append(rai::BT_none);
  CHECK(!f->joint, "");
  f->ensure_X();
  if(!self->actors(f->ID)) {
    self->addLink(f);
  } else {
    HALT("NO!");
  }
}

void PhysXInterface::pushJointTargets(const rai::Configuration& C, const arr& qDot_ref, bool setStatesInstantly) {
  // for(rai::Frame* f:C.frames) if(f->joint && self->actors(f->ID)) {
  for(rai::Frame *f:self->articulationJoints){
      PxArticulationLink* actor = self->actors(f->ID)->is<PxArticulationLink>();
      CHECK(actor,""); if(!actor) continue;
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
      CHECK(joint,""); if(!joint) continue;

      auto axis = self->jointAxis(f->ID);
      if(axis!=PxArticulationAxis::eCOUNT){ //only joints with drive
        if(setStatesInstantly) joint->setJointPosition(axis, f->joint->scale*f->joint->get_q());
        joint->setDriveTarget(axis, f->joint->scale*f->joint->get_q());

	if(!!qDot_ref && qDot_ref.N) { //also setting vel reference!
	  if(setStatesInstantly) joint->setJointVelocity(axis, f->joint->scale*qDot_ref(f->joint->qIndex));
	  joint->setDriveVelocity(axis, f->joint->scale*qDot_ref(f->joint->qIndex));
	} else {
	  if(setStatesInstantly) joint->setJointVelocity(axis, 0.);
	  joint->setDriveVelocity(axis, 0.);
	}
      }
    }
}

void PhysXInterface::pullJointStates(rai::Configuration& C, arr& qDot) {
  C.ensure_q();
  arr q = C.getJointState();
  arr qInactive = C.qInactive;
  if(!!qDot) qDot.resize(q.N).setZero();

  // for(rai::Frame* f:C.frames) if(f->joint && self->actors(f->ID)) { //f->joint->active &&
  for(rai::Frame *f:self->articulationJoints){
      PxArticulationLink* actor = self->actors(f->ID)->is<PxArticulationLink>();
    CHECK(actor,""); if(!actor) continue;
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
    CHECK(joint,""); if(!joint) continue;

      auto axis = self->jointAxis(f->ID);
      if(axis!=PxArticulationAxis::eCOUNT){ //only joints with drive
        if(f->joint->active){
          q(f->joint->qIndex) = joint->getJointPosition(axis) / f->joint->scale;
          if(!!qDot) qDot(f->joint->qIndex) = joint->getJointVelocity(axis) / f->joint->scale;
        }else{
          qInactive(f->joint->qIndex) = joint->getJointPosition(axis) / f->joint->scale;
          if(!!qDot){ NIY }
        }
      }
    }
  // C.qInactive = qInactive;
  C.setJointState(q);

#if 0
  //articulation joint torques
  for(rai::Frame* f:C.frames) if(self->actors(f->ID)) {
      PxArticulationLink* actor = self->actors(f->ID)->is<PxArticulationLink>();
      if(!actor) continue;
      if(actor->getLinkIndex()==0){ //root of an articulation
        LOG(0) <<"articulation: " <<f->name;
        PxArticulationReducedCoordinate* articulation = &actor->getArticulation();
        PxArticulationCache* cache = articulation->createCache();
        articulation->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eALL);
        // articulation->computeJointForce(cache);
        uint n = articulation->getDofs();
        for(uint i=0;i<n;i++){
          cout <<"joint" <<i;
          // cout <<" pos:" <<cache->jointPosition[i];
          // cout <<" vel:" <<cache->jointVelocity[i];
          cout <<" acc:" <<cache->jointAcceleration[i];
          cout <<" frc:" <<cache->jointForce[i];
          cout <<" sof:" <<cache->jointSolverForces[i];
          cout <<endl;
        }
        n = articulation->getNbLinks();
        for(uint i=0;i<n;i++){
          cout <<"link" <<i;
          cout <<" exF:" <<conv_PxVec3_arr(cache->externalForces[i].force);
          // cout <<" vel:" <<conv_PxVec3_arr(cache->linkVelocity[i].linear);
          // cout <<" acc:" <<conv_PxVec3_arr(cache->linkAcceleration[i].linear);
          cout <<endl;
        }
        cache->release();
      }
    }
#endif
}

void PhysXInterface::pushFreeStates(const rai::Configuration& C, const arr& frameVelocities, bool onlyKinematic) {
  if(!!frameVelocities && frameVelocities.N) CHECK_EQ(frameVelocities.d0, self->freeFrames.N, "");

  // frame states (including of dynamic, e.g. falling, objects)
  uint i=0;
  for(rai::Frame* f : self->freeFrames) {
    PxRigidActor* a = self->actors(f->ID);
    if(!a) continue; //f is not an actor

    if(self->actorTypes(f->ID)==rai::BT_kinematic) {
      ((PxRigidDynamic*)a)->setKinematicTarget(conv_Transformation2PxTrans(f->ensure_X()));
    } else if(!onlyKinematic) {
      a->setGlobalPose(conv_Transformation2PxTrans(f->ensure_X()));

      if(self->actorTypes(f->ID)==rai::BT_dynamic && (a->getType() == PxActorType::eRIGID_DYNAMIC)) {
        PxRigidDynamic* px_body = (PxRigidDynamic*)(a);
        if(!!frameVelocities && frameVelocities.N) {
          px_body->setLinearVelocity(PxVec3(frameVelocities(i, 0, 0), frameVelocities(i, 0, 1), frameVelocities(i, 0, 2)));
          px_body->setAngularVelocity(PxVec3(frameVelocities(i, 1, 0), frameVelocities(i, 1, 1), frameVelocities(i, 1, 2)));
        } else {
          px_body->setLinearVelocity(PxVec3(0., 0., 0.));
          px_body->setAngularVelocity(PxVec3(0., 0., 0.));
        }
      }
    }
    i++;
  }
}

void PhysXInterface::pullFreeStates(rai::Configuration& C, arr& frameVelocities) {
  if(!!frameVelocities) frameVelocities.resize(self->freeFrames.N, 2, 3).setZero();

  uint i=0;
  for(rai::Frame* f : self->freeFrames) {
    PxRigidActor* a = self->actors(f->ID);
    if(!a) continue;

    // if(f->joint && !f->joint->active && f->joint->dim==1) continue; //don't pull gripper joint states

    if(self->actorTypes(f->ID) == rai::BT_dynamic) {
      f->set_X() = conv_PxTrans2Transformation(a->getGlobalPose());
      if(!!frameVelocities && (a->getType() == PxActorType::eRIGID_DYNAMIC || a->getType() == PxActorType::eARTICULATION_LINK)) {
        PxRigidBody* px_body = (PxRigidBody*)(a);
        frameVelocities(i, 0, {}) = conv_PxVec3_arr(px_body->getLinearVelocity());
        frameVelocities(i, 1, {}) = conv_PxVec3_arr(px_body->getAngularVelocity());
      }
      //      if(f->parent) LOG(0) <<f->parent->name <<f->ensure_X().pos <<f->parent->ensure_X().pos <<f->get_Q().pos;
    }
    i++;
  }
}

// void PhysXInterface::setArticulatedBodiesKinematic(const rai::Configuration& C) {
//   HALT("NOT SURE IF THIS IS DESIRED");
//   for(rai::Dof* d:C.activeDofs) {
//     rai::Joint* j = dynamic_cast<rai::Joint*>(d);
//     if(!j) continue;
//     if(j->type!=rai::JT_free) {
//       if(j->from()->inertia && j->from()->inertia->type==rai::BT_dynamic) j->from()->inertia->type=rai::BT_kinematic;
//       if(j->frame->inertia   && j->frame->inertia->type==rai::BT_dynamic) j->frame->inertia->type=rai::BT_kinematic;
//     }
//   }
//   for(rai::Frame* b: C.frames) if(self->actors(b->ID) && b->inertia) {
//       if(b->inertia->type==rai::BT_kinematic)
//         ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
//       if(b->inertia->type==rai::BT_dynamic)
//         ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
//     }
// }

rai::Configuration& PhysXInterface::getDebugConfig(){
  self->syncDebugConfig();
  return self->debugConfig;
}

void PhysXInterface::setGravity(float grav) {
  self->gScene->setGravity(PxVec3(0.f, 0.f, grav));
}

void PhysXInterface::disableGravity(rai::Frame* f, bool disable) {
  self->actors(f->ID)->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, disable);
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

rai::PhysX_Options& PhysXInterface::opt() {
  return self->opt;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //RAI_PHYSX

#include "kin_physx.h"
PhysXInterface::PhysXInterface(rai::Configuration& C, int verbose, const rai::PhysX_Options* _opt) { NICO }
PhysXInterface::~PhysXInterface() { NICO }

void PhysXInterface::step(double tau) { NICO }
void PhysXInterface::pushFrameStates(const rai::Configuration& C, const arr& frameVelocities, bool onlyKinematic) { NICO }
void PhysXInterface::pullDynamicStates(rai::Configuration& C, arr& vels) { NICO }
void PhysXInterface::pushMotorTargets(const rai::Configuration& C, const arr& qDot_ref, bool setStatesInstantly) { NICO }
void PhysXInterface::pullMotorStates(rai::Configuration& C, arr& qDot) { NICO }
void PhysXInterface::postAddObject(rai::Frame* f) { NICO }

void PhysXInterface::changeObjectType(rai::Frame* f, int _type) { NICO }
void PhysXInterface::addJoint(rai::Joint* j) { NICO }
void PhysXInterface::removeJoint(rai::Joint* j) { NICO }
void PhysXInterface::setGravity(float grav) { NICO }
void PhysXInterface::disableGravity(rai::Frame* f, bool disable) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos) { NICO }

rai::Configuration& PhysXInterface::getDebugConfig() { NICO }

void glPhysXInterface(void* classP) { NICO }

rai::PhysX_Options& PhysXInterface::opt() { NICO }

#endif

#ifdef RAI_PHYSX
RUN_ON_INIT_BEGIN(kin_physx)
rai::Array<PxGeometry*>::memMove=true;
rai::Array<PxShape*>::memMove=true;
rai::Array<PxMaterial*>::memMove=true;
rai::Array<PxRigidActor*>::memMove=true;
rai::Array<PxD6Joint*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_physx)
#endif
