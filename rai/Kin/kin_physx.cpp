/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
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
  params.buildTriangleAdjacencies = false;
  //params.buildGPUData = true;

  params.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_INERTIA;
  params.meshWeldTolerance = 1e-7f;

  PxSDFDesc sdfDesc;

  float sdfSpacing=-.01;
  if (sdfSpacing > 0.f){
    sdfDesc.spacing = sdfSpacing;
    sdfDesc.subgridSize = 6; //sdfSubgridSize;
    sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL; //bitsPerSdfSubgridPixel;
    sdfDesc.numThreadsForSdfConstruction = 16;
    meshDesc.sdfDesc = &sdfDesc;
  }

  return PxCreateTriangleMesh(params, meshDesc, physics.getPhysicsInsertionCallback());
  //return cooking.createTriangleMesh(meshDesc); //, physics.getPhysicsInsertionCallback());
//  PxDefaultMemoryOutputStream writeBuffer;
//  if(!cooking.cookTriangleMesh(meshDesc, writeBuffer)) return nullptr;
//  PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
//  return physics.createTriangleMesh(readBuffer);
}
}

//===========================================================================

struct PhysXInterface_self {
  struct Engine{
    PxFoundation* mFoundation = nullptr;
    PxPhysics* mPhysics = nullptr;
    PxCooking* mCooking = nullptr;
    PxDefaultErrorCallback gDefaultErrorCallback;
    PxDefaultAllocator gDefaultAllocatorCallback;
    PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;
  };

  static Engine* core;

  ~PhysXInterface_self() {
    //  self->mMaterial
    //  self->plane
    //  self->planeShape
    for(PxRigidActor* a: actors) if(a) {
      PxArticulationLink* actor = a->is<PxArticulationLink>();
      if(actor) continue; //don't remove articulation links
      gScene->removeActor(*a);
      a->release();
    }
    if(gScene) {
      gScene->release();
      gScene = nullptr;
    }
//    if(mPhysics) {
//      mCooking->release();
//      mPhysics->release();
//    }
    //  mFoundation->release();
  }


  PxScene* gScene = nullptr;
  rai::Array<PxRigidActor*> actors;
  rai::Array<rai::BodyType> actorTypes;
  rai::Array<PxArticulationAxis::Enum> jointAxis;
  rai::Array<PxRevoluteJoint*> joints;

  rai::PhysX_Options opt;

  uint stepCount=0;

  PxMaterial* defaultMaterial;

  void initPhysics();
  void addGround();
  void addLink(rai::Frame* b);
  void addJoint(const rai::Joint* jj);
  void addMultiBody(rai::Frame* base);

  void lockJoint(PxD6Joint* joint, rai::Joint* rai_joint);
  void unlockJoint(PxD6Joint* joint, rai::Joint* rai_joint);

  void prepareLinkShapes(ShapeL& shapes, rai::BodyType& type, rai::Frame* f);
  void addSingleShape(PxRigidActor* actor, rai::Frame *f, rai::Shape* s);
  void addShapesAndInertia(PxRigidBody* actor, ShapeL& shapes, rai::BodyType type, rai::Frame* f);
};

PhysXInterface_self::Engine *PhysXInterface_self::core = 0;

//===========================================================================

void PhysXInterface_self::initPhysics(){
  if(!core){
    core = new Engine;

    core->mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, core->gDefaultAllocatorCallback, core->gDefaultErrorCallback);
    PxTolerancesScale scale;
    core->mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *core->mFoundation, scale);
    PxCookingParams cookParams(core->mPhysics->getTolerancesScale());
//    cookParams.skinWidth = .001f;
    core->mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *core->mFoundation, cookParams);
    if(!core->mCooking) HALT("PxCreateCooking failed!");
    if(!core->mPhysics) HALT("Error creating PhysX3 device.");
    //if(!PxInitExtensions(*mPhysics)) HALT("PxInitExtensions failed!");
  }

  //-- Create the scene
  PxSceneDesc sceneDesc(core->mPhysics->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.f, 0.f, px_gravity);
  sceneDesc.bounceThresholdVelocity = 10.;
  //sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
  sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
  //sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
  sceneDesc.gpuMaxNumPartitions = 8;
  sceneDesc.solverType = PxSolverType::eTGS;

  if(!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if(!mCpuDispatcher) {
      cerr << "PxDefaultCpuDispatcherCreate failed!" << endl;
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if(!sceneDesc.filterShader) {
    sceneDesc.filterShader  = core->gDefaultFilterShader;
  }

  gScene = core->mPhysics->createScene(sceneDesc);
  if(!gScene) {
    cerr << "createScene failed!" << endl;
  }

  gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
  gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

  //-- Create objects
  defaultMaterial = core->mPhysics->createMaterial(opt.defaultFriction, opt.defaultFriction, opt.defaultRestitution);
}

void PhysXInterface_self::addGround(){
  PxTransform pose = PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f)));

  PxRigidStatic* plane = core->mPhysics->createRigidStatic(pose);
  CHECK(plane, "create plane failed!");

  PxShape* planeShape = core->mPhysics->createShape(PxPlaneGeometry(), *defaultMaterial);
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

  if(opt.multiBody){
    if(f->joint && !f->joint->isPartBreak()) type=rai::BT_dynamic;
  }

  if(opt.verbose>0){
    LOG(0) <<"adding link '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type) <<" with " <<shapes.N <<" shapes";
    for(rai::Shape* s:shapes) cout <<' ' <<s->frame.name;
    cout <<endl;
  }

  //-- create a PhysX actor
  PxRigidDynamic* actor=nullptr;
  if(type==rai::BT_static){
    actor = (PxRigidDynamic*) core->mPhysics->createRigidStatic(conv_Transformation2PxTrans(f->ensure_X()));
  } else if(type==rai::BT_dynamic){
    actor = core->mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
  } else if(type==rai::BT_kinematic){
    actor = core->mPhysics->createRigidDynamic(conv_Transformation2PxTrans(f->ensure_X()));
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
  } else NIY;
  CHECK(actor, "create actor failed!");

  addShapesAndInertia(actor, shapes, type, f);

  double angularDamping = opt.angularDamping;
  if(f->ats && f->ats->find<double>("angularDamping")){
    angularDamping = f->ats->get<double>("angularDamping");
  }
  actor->setAngularDamping(angularDamping);

  gScene->addActor(*actor);

  actor->userData = f;
  CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
  actors(f->ID) = actor;
  actorTypes(f->ID) = type;
}

void PhysXInterface_self::addJoint(const rai::Joint* jj) {
  //HALT("REALLY?");
  while(joints.N <= jj->frame->ID)
    joints.append(nullptr);

  rai::Transformation rel;
  rai::Frame* to = jj->frame;
  rai::Frame* from = jj->frame->parent->getUpwardLink(rel);

  LOG(0) <<"ADDING JOINT " <<from->name <<'-' <<to->name <<" of type " <<jj->type;

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

      //PxD6Joint* desc = PxD6JointCreate(*core->mPhysics, actors(from->ID), A, actors(to->ID), B.getInverse());
      PxRevoluteJoint* joint = PxRevoluteJointCreate(*core->mPhysics, actors(from->ID), A, actors(to->ID), B.getInverse());
      joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      joint->setDriveForceLimit(1e1);
//      cout <<joint->getDriveVelocity() <<endl;
//      joint->setDriveVelocity(0.f);
      if(jj->limits.N){
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
      joints(to->ID) = joint;
    }
    break;
    case rai::JT_rigid: {
      // PxFixedJoint* desc =
      PxFixedJointCreate(*core->mPhysics, actors(jj->from()->ID), A, actors(to->ID), B.getInverse());
      // desc->setProjectionLinearTolerance(1e10);
      // desc->setProjectionAngularTolerance(3.14);
    }
    break;
    case rai::JT_trans3: {
      break;
    }
    case rai::JT_transXYPhi: {
#if 0
      PxD6Joint* desc = PxD6JointCreate(*core->mPhysics, actors(jj->from()->ID), A, actors(to->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
      desc->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

      joints(to->ID) = desc;
#endif
      break;
    }
    case rai::JT_transX:
    case rai::JT_transY:
    case rai::JT_transZ: {
#if 0
      PxD6Joint* desc = PxD6JointCreate(*core->mPhysics, actors(jj->from()->ID), A, actors(to->ID), B.getInverse());
      CHECK(desc, "PhysX joint creation failed.");

      if(to->ats && to->ats->find<arr>("drive")) {
        arr drive_values = to->ats->get<arr>("drive");
        PxD6JointDrive drive(drive_values(0), drive_values(1), PX_MAX_F32, true);
        desc->setDrive(PxD6Drive::eX, drive);
      }

      if(to->ats && to->ats->find<arr>("limit")) {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);

        arr limits = to->ats->get<arr>("limit");
        PxJointLinearLimit limit(core->mPhysics->getTolerancesScale(), limits(0), 0.1f);
        limit.restitution = limits(2);
        //if(limits(3)>0) {
        //limit.spring = limits(3);
        //limit.damping= limits(4);
        //}
        desc->setLinearLimit(limit);
      } else {
        desc->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
      }
      joints(to->ID) = desc;
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

void PhysXInterface_self::addMultiBody(rai::Frame* base) {
  //CHECK(!base->parent || (base->joint && base->joint->type==rai::JT_rigid) || (base->joint && base->inertia), "base needs to be either rigid or with inertia");

  //-- collect all links for that root
  FrameL F = {base};
  //base->getPartSubFrames(F);
  base->getSubtree(F);
  FrameL links = {base};
  for(auto* f:F){ if(f->joint && /*f->joint->active &&*/ !f->joint->isPartBreak()) links.append(f); }
  intA parents(links.N);
  parents = -1;
  for(uint i=1;i<links.N;i++){
    rai::Frame *p = links(i)->parent->getUpwardLink();
    parents(i) = links.findValue(p);
    if(parents(i)==-1){
      //special case: the multibody contains a partbreak link - skip over it (same as assuming fixed)
      CHECK(p->joint && p->joint->isPartBreak(), "");
      p = p->parent->getUpwardLink();
      parents(i) = links.findValue(p);
    }
    CHECK(parents(i)>=0, "");
  }
  rai::Array<PxArticulationLink*> linksPx(links.N);
  linksPx = NULL;

  if(opt.verbose>0){
    LOG(0) <<"adding multibody with base '" <<base->name <<"' with the following links ...";
  }

  PxArticulationReducedCoordinate* articulation = core->mPhysics->createArticulationReducedCoordinate();
  articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
  articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
  //articulation->setSolverIterationCounts(minPositionIterations, minVelocityIterations);
  //articulation->setMaxCOMLinearVelocity(maxCOMLinearVelocity);

  for(uint i=0; i<links.N; i++) {
    //prepare link shapes, inertia, and type
    rai::Frame* f = links(i);
    if(i!=0) CHECK(f->joint, "");
    ShapeL shapes;
    rai::BodyType type;
    prepareLinkShapes(shapes, type, f);
    if(i>0) type = rai::BT_dynamic;

    //create link
    if(opt.verbose>0){
      LOG(0) <<"adding multibody link '" <<f->name <<"' as " <<rai::Enum<rai::BodyType>(type) <<" with " <<shapes.N <<" shapes";
      for(rai::Shape* s:shapes) cout <<' ' <<s->frame.name;
      cout <<endl;
    }

    PxArticulationLink* actor = 0;
    actor = articulation->createLink(i==0?NULL:linksPx(parents(i)), conv_Transformation2PxTrans(f->ensure_X()));
    linksPx(i) = actor;
    actor->userData = f;
    CHECK(!actors(f->ID), "you already added a frame with ID" <<f->ID);
    actors(f->ID) = actor;
    actorTypes(f->ID) = type;

    addShapesAndInertia(actor, shapes, type, f);

    if(i>0){
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
      rai::Frame* prevLink = links(parents(i));
      rai::Transformation relA = f->parent->ensure_X() / prevLink->ensure_X();
      rai::Transformation relB = 0; //-f->get_Q();
      joint->setParentPose(conv_Transformation2PxTrans(relA));
      joint->setChildPose(conv_Transformation2PxTrans(relB));

      PxArticulationAxis::Enum axis = PxArticulationAxis::eCOUNT;
      PxArticulationJointType::Enum type = PxArticulationJointType::eUNDEFINED;
      switch(f->joint->type){
        case rai::JT_hingeX:{
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eTWIST;
          break;
        }
        case rai::JT_hingeY:{
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eSWING1;
          break;
        }
        case rai::JT_hingeZ:{
          type = PxArticulationJointType::eREVOLUTE;
          axis = PxArticulationAxis::eSWING2;
          break;
        }
        case rai::JT_transX:{
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eX;
          break;
        }
        case rai::JT_transY:{
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eY;
          break;
        }
        case rai::JT_transZ:{
          type = PxArticulationJointType::ePRISMATIC;
          axis = PxArticulationAxis::eZ;
          break;
        }
//        case rai::JT_quatBall:{
//          type = PxArticulationJointType::eSPHERICAL;
//          axis = PxArticulationAxis::eZ;
//          break;
//        }
        default: NIY;
      }
      joint->setJointType(type);
      joint->setMotion(axis, PxArticulationMotion::eFREE); //eLIMITED
      joint->setJointPosition(axis, f->joint->scale*f->joint->get_q());
      jointAxis(f->ID) = axis;

//      if(f->joint->limits.N){
//        joint->setLimitParams(axis, {(float)f->joint->limits(0), (float)f->joint->limits(1)});
//      }

      if(f->joint->mimic){
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


      if(true){
        PxArticulationDrive posDrive;
        if(f->joint->active){
          posDrive.stiffness = opt.motorKp;                      // the spring constant driving the joint to a target position
          posDrive.damping = opt.motorKd;                        // the damping coefficient driving the joint to a target velocity
        }else{ //hack for grippers
          posDrive.stiffness = opt.motorKp/10.;
          posDrive.damping = opt.motorKd/10.;
        }
        posDrive.maxForce = PX_MAX_F32; //1e10f;                              // force limit for the drive
        posDrive.driveType = PxArticulationDriveType::eFORCE;  // make the drive output be a force/torque (default)
        joint->setDriveParams(axis, posDrive);
        joint->setDriveVelocity(axis, 0.);
        joint->setDriveTarget(axis, f->joint->scale*f->joint->get_q());
      }
    }
  }

  gScene->addArticulation(*articulation);

  if(opt.verbose>0){
    LOG(0) <<"... done with multibody with base '" <<base->name <<"'";
  }
}

void PhysXInterface_self::prepareLinkShapes(ShapeL& shapes, rai::BodyType& type, rai::Frame* f) {
  //-- collect all shapes of that link
  shapes.clear();
  {
    rai::Frame* link=f->getUpwardLink();
    FrameL tmp = {link};
    link->getRigidSubFrames(tmp, false);
    for(rai::Frame* p: tmp){
      if(p->shape
         && p->getShape().type()!=rai::ST_marker
         && p->getShape().type()!=rai::ST_camera
         && p->getShape().alpha()==1.) shapes.append(p->shape);
    }
  }

  //-- prepare inertia
  bool shapesHaveInertia=false;
  for(rai::Shape *s:shapes) if(s->frame.inertia){ shapesHaveInertia=true; break; }
  if(shapesHaveInertia && !f->inertia){
    LOG(-1) <<"computing compound inertia for object frame '" <<f->name <<"' -- this should have been done earlier?";
    f->computeCompoundInertia();
    f->transformToDiagInertia();
  }
  if(f->inertia && !f->inertia->com.isZero){
    LOG(-2) <<"DON'T DO THAT! Bullet can only properly handle (compound) inertias if transformed to zero com and diagonal tensor";
  }

  //-- decide on the type
  type = rai::BT_static;
  if(f->joint)   type = rai::BT_kinematic;
  if(f->inertia) type = f->inertia->type;
}

void PhysXInterface_self::addSingleShape(PxRigidActor* actor, rai::Frame* f, rai::Shape* s){
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
    case rai::ST_ssCylinder:
    case rai::ST_ssBox:
    case rai::ST_ssCvx: {
      floatA Vfloat = rai::convert<float>(s->mesh().V);
      PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                     *core->mPhysics, *core->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                     PxConvexFlag::eCOMPUTE_CONVEX);
      geometry = new PxConvexMeshGeometry(triangleMesh);
    } break;
    case rai::ST_sdf:
    case rai::ST_mesh: {
#if 0
      floatA Vfloat = rai::convert<float>(s->mesh().V);
      uintA& T = s->mesh().T;
      PxTriangleMesh* triangleMesh =  PxToolkit::createTriangleMesh32(*core->mPhysics, *core->mCooking,
                                                                      (PxVec3*)Vfloat.p, Vfloat.d0,
                                                                      T.p, T.d0);
      geometry = new PxTriangleMeshGeometry(triangleMesh);
#else
      rai::Mesh &M = s->mesh();
      if(!M.cvxParts.N){
        M.getComponents();
      }
      if(M.cvxParts.N){
        floatA Vfloat;
        LOG(0) <<"creating " <<M.cvxParts.N <<" convex parts for shape " <<s->frame.name;
        for(uint i=0;i<M.cvxParts.N;i++){
          Vfloat.clear();
          int start = M.cvxParts(i);
          int end = i+1<M.cvxParts.N ? M.cvxParts(i+1)-1 : -1;
          copy(Vfloat, M.V({start, end}));
          PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                         *core->mPhysics, *core->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                         PxConvexFlag::eCOMPUTE_CONVEX);
          geometry = new PxConvexMeshGeometry(triangleMesh);
          PxShape* shape = core->mPhysics->createShape(*geometry, *defaultMaterial);
          actor->attachShape(*shape);
          if(&s->frame!=f) {
            if(s->frame.parent==f) {
              shape->setLocalPose(conv_Transformation2PxTrans(s->frame.get_Q()));
            } else {
              rai::Transformation rel = s->frame.ensure_X() / f->ensure_X();
              shape->setLocalPose(conv_Transformation2PxTrans(rel));
            }
          }
        }
        geometry=0;
      }else{
        if(opt.verbose>1) LOG(0) <<"using cvx hull of mesh as no decomposition (M.cvsParts) is available";
        floatA Vfloat = rai::convert<float>(s->mesh().V);
        PxConvexMesh* triangleMesh = PxToolkit::createConvexMesh(
                                       *core->mPhysics, *core->mCooking, (PxVec3*)Vfloat.p, Vfloat.d0,
                                       PxConvexFlag::eCOMPUTE_CONVEX);
        geometry = new PxConvexMeshGeometry(triangleMesh);
      }
#endif
    } break;
    case rai::ST_camera:
    case rai::ST_marker: {
      geometry = nullptr;
    } break;
    default:
      LOG(0) <<"can't create shape of type:" <<s->type();
      NIY;
  }

  if(geometry) {
    //-- decide/create a specific material
    PxMaterial* mMaterial = defaultMaterial;
    if(s->frame.ats &&
       (s->frame.ats->find<double>("friction") || s->frame.ats->find<double>("restitution"))) {
      double fric=s->frame.ats->get<double>("friction", opt.defaultFriction);
      double rest=s->frame.ats->get<double>("restitution", opt.defaultRestitution);
      //LOG(0) <<" shape " <<s->frame.name <<" friction: " <<fric <<" restitution: " <<rest;
      mMaterial = core->mPhysics->createMaterial(fric, fric, rest);
    }

    PxShape* shape = core->mPhysics->createShape(*geometry, *mMaterial);
    actor->attachShape(*shape);
    if(&s->frame!=f) {
      if(s->frame.parent==f) {
        shape->setLocalPose(conv_Transformation2PxTrans(s->frame.get_Q()));
      } else {
        rai::Transformation rel = s->frame.ensure_X() / f->ensure_X();
        shape->setLocalPose(conv_Transformation2PxTrans(rel));
      }
    }
    CHECK(shape, "create shape failed!");
  }
}

void PhysXInterface_self::addShapesAndInertia(PxRigidBody* actor, ShapeL& shapes, rai::BodyType type, rai::Frame* f){
  //-- add each shape to the actor
  for(rai::Shape* s: shapes) addSingleShape(actor, f, s);

  //-- set inertia
  if(type != rai::BT_static) {
    if(/*false && */f->inertia && f->inertia->mass>0.) {
      //PxRigidBodyExt::updateMassAndInertia(*actor, f->inertia->mass);
      actor->setMass(f->inertia->mass);
      actor->setMassSpaceInertiaTensor({float(f->inertia->matrix.m00), float(f->inertia->matrix.m11), float(f->inertia->matrix.m22)});
      //cout <<*f->inertia <<" m:" <<actor->getMass() <<" I:" <<conv_PxVec3_arr(actor->getMassSpaceInertiaTensor()) <<endl;
    } else {
      PxRigidBodyExt::updateMassAndInertia(*actor, 1000.f);
      if(!f->inertia) new rai::Inertia(*f);
      f->inertia->mass = actor->getMass();
      f->inertia->matrix.setDiag(conv_PxVec3_arr( actor->getMassSpaceInertiaTensor() ));
      f->inertia->com = conv_PxVec3_arr( actor->getCMassLocalPose().p );
      //cout <<*f->inertia <<" m:" <<actor->getMass() <<" I:" <<conv_PxVec3_arr(actor->getMassSpaceInertiaTensor()) <<endl;
    }
  }
}

//===========================================================================

PhysXInterface::PhysXInterface(const rai::Configuration& C, int verbose): self(nullptr) {
  CHECK(C._state_q_isGood, "PhysX needs joint angles for initialization");

  self = new PhysXInterface_self;

  self->opt.verbose = verbose;

  if(self->opt.verbose>0) LOG(0) <<"starting PhysX engine ...";

  self->initPhysics();

  self->addGround();

  //-- create Configuration equivalent in PhysX
  self->actors.resize(C.frames.N).setZero();
  self->actorTypes.resize(C.frames.N).setZero();
  self->jointAxis.resize(C.frames.N) = PxArticulationAxis::eCOUNT;

  for(rai::Frame* a : C.frames) a->ensure_X();

  if(self->opt.multiBody){
    FrameL parts = C.getParts();
    for(rai::Frame *f : parts){
      bool asMultiBody=false;
#if 0
      FrameL sub = f->getSubtree();
      for(rai::Frame *a:sub) if(a->joint){ asMultiBody=true; break; }
#else
      if(f->ats && f->ats->findNode("multibody")) asMultiBody=true;
#endif
      if(asMultiBody){
        self->addMultiBody(f);
      }else{
        self->addLink(f);
      }
    }
  } else {
    FrameL links = C.getLinks();
    for(rai::Frame* a : links) self->addLink(a);
    if(self->opt.jointedBodies){
      for(rai::Dof *j : C.activeDofs) self->addJoint(j->joint());
    }
  }

  if(self->opt.verbose>0) LOG(0) <<"... done creating Configuration within PhysX";
}

PhysXInterface::~PhysXInterface() {
  delete self;
}

void PhysXInterface::step(double tau) {
  self->stepCount++;
  self->gScene->simulate(tau);

  //...perform useful work here using previous frame's state data
  while(!self->gScene->fetchResults()) {
  }
}

void PhysXInterface::pullDynamicStates(rai::Configuration& C, arr& frameVelocities) {
  if(!!frameVelocities) frameVelocities.resize(C.frames.N, 2, 3).setZero();

  for(rai::Frame* f : C.frames) {
    if(self->actors.N <= f->ID) continue;
    PxRigidActor* a = self->actors(f->ID);
    if(!a) continue;

    if(self->opt.multiBody && f->joint && !f->joint->active && f->joint->dim==1) continue; //don't pull gripper joint states

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

  //-- pull joint state directly
  if(self->opt.jointedBodies){
    arr q = C.getJointState();
    for(rai::Dof *d:C.activeDofs) if(self->joints(d->frame->ID)){
      q(d->qIndex) = self->joints(d->frame->ID)->getAngle();
    }
    C.setJointState(q);
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
    self->addLink(f);
  } else {
    HALT("NO!");
  }
}

void PhysXInterface::setMotorQ(const arr& q_ref, const arr& qDot_ref){
  if(qDot_ref.N){ CHECK_EQ(q_ref.N, qDot_ref.N, ""); }
  uint qIdx = 0;
  if(self->opt.multiBody){
    for(PxRigidActor* a : self->actors) {
      if(!a) continue;
      rai::Frame *f = ((rai::Frame*)a->userData);
      PxArticulationLink* actor = a->is<PxArticulationLink>();
      if(!actor) continue;
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
      if(!joint) continue;
      if(!f->joint->active) continue;
      CHECK_EQ(f->joint->qIndex, qIdx, "inconsistent q indexing");

      auto axis = self->jointAxis(f->ID);
      CHECK_LE(axis, self->jointAxis(0)-1, "");
      if(q_ref.N) joint->setDriveTarget(axis, q_ref(qIdx));
      if(qDot_ref.N) joint->setDriveVelocity(axis, qDot_ref(qIdx));
      qIdx++;
    }
  }else if(self->opt.jointedBodies){
    for(PxRevoluteJoint* j:self->joints) if(j){
      double qi = j->getAngle();
      double v_ref = self->opt.motorKp * (q_ref(qIdx) - qi);
      //cout <<' ' <<v_ref <<' ' <<qi;
      j->setDriveVelocity(v_ref);
      qIdx++;
    }
    //cout <<endl;
  }
  if(q_ref.N) CHECK_EQ(qIdx, q_ref.N, ""); //make this only a warning?
}

void PhysXInterface::setMotorQ(const rai::Configuration& C, bool setHardInstantly){
  if(self->opt.multiBody){
    for(rai::Frame* f:C.frames) if(f->joint && self->actors(f->ID)){
      PxArticulationLink* actor = self->actors(f->ID)->is<PxArticulationLink>();
      if(!actor) continue;
      PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
      if(!joint) continue;

      auto axis = self->jointAxis(f->ID);
      CHECK_LE(axis, self->jointAxis(0)-1, "");
      if(setHardInstantly){
        //LOG(0) <<"setting joint pos hard: " <<f->name <<' ' <<f->joint->get_q();
        joint->setJointPosition(axis, f->joint->scale*f->joint->get_q());
        joint->setDriveVelocity(axis, 0.);
      }
      joint->setDriveTarget(axis, f->joint->scale*f->joint->get_q());
    }
  }else if(self->opt.jointedBodies){
    NIY;
  }
}

void PhysXInterface::pushKinematicStates(const rai::Configuration& C) {
  for(rai::Frame* f: C.frames) {
    if(self->actors.N <= f->ID) continue;
    if(self->actorTypes(f->ID)==rai::BT_kinematic) {
      PxRigidActor* a = self->actors(f->ID);
      if(!a) continue; //f is not an actor

      //LOG(0) <<"pushing kinematic state (i.e. frame pose) of " <<f->name;
      ((PxRigidDynamic*)a)->setKinematicTarget(conv_Transformation2PxTrans(f->ensure_X()));
    }
  }
}

void PhysXInterface::pushFullState(const rai::Configuration& C, const arr& frameVelocities, bool onlyKinematic) {
  // frame states (including of dynamic, e.g. falling, objects)
  for(rai::Frame* f : C.frames) {
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
        PxRigidDynamic* px_body = (PxRigidDynamic*) a;
        px_body->setLinearVelocity(PxVec3(frameVelocities(f->ID, 0, 0), frameVelocities(f->ID, 0, 1), frameVelocities(f->ID, 0, 2)));
        px_body->setAngularVelocity(PxVec3(frameVelocities(f->ID, 1, 0), frameVelocities(f->ID, 1, 1), frameVelocities(f->ID, 1, 2)));
      } else {
//        PxRigidDynamic* px_body = dynamic_cast<PxRigidDynamic*>(a);
//        if(px_body){
//          px_body->setLinearVelocity(PxVec3(0., 0., 0.));
//          px_body->setAngularVelocity(PxVec3(0., 0., 0.));
//        }
      }
    }
  }

  //-- joint states for multibody (similar to setMotorQ)
  setMotorQ(C, true);
}

void PhysXInterface::setArticulatedBodiesKinematic(const rai::Configuration& C) {
  HALT("NOT SURE IF THIS IS DESIRED");
  for(rai::Dof* d:C.activeDofs){
    rai::Joint *j = dynamic_cast<rai::Joint*>(d);
    if(!j) continue;
    if(j->type!=rai::JT_free) {
      if(j->from()->inertia && j->from()->inertia->type==rai::BT_dynamic) j->from()->inertia->type=rai::BT_kinematic;
      if(j->frame->inertia   && j->frame->inertia->type==rai::BT_dynamic) j->frame->inertia->type=rai::BT_kinematic;
    }
  }
  for(rai::Frame* b: C.frames) if(self->actors(b->ID) && b->inertia) {
    if(b->inertia->type==rai::BT_kinematic)
      ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    if(b->inertia->type==rai::BT_dynamic)
      ((PxRigidDynamic*)self->actors(b->ID))->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
  }
}

void DrawActor(PxRigidActor* actor, rai::Frame* frame, OpenGL& gl) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;

  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape* shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    rai::Shape* s = frame->shape;
    if(!s) for(rai::Frame* ch:frame->children){
      if(ch->shape && ch->shape->alpha()==1.){
        s = ch->shape;
        break;
      }
    }
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
        floatA Vfloat;
        Vfloat.referTo((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference!
        rai::Mesh mesh;
        copy(mesh.V, Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(), 3);
        mesh.makeConvexHull();
        mesh.glDraw(gl);
#else
        self->mesh.glDraw();
#endif
      } break;
      case PxGeometryType::eTRIANGLEMESH: {
        PxTriangleMeshGeometry g;
        shape->getTriangleMeshGeometry(g);
        floatA Vfloat;
        Vfloat.referTo((float*)g.triangleMesh->getVertices(), 3*g.triangleMesh->getNbVertices()).reshape(-1,3);
        rai::Mesh mesh;
        mesh.V = rai::convert<double>(Vfloat);
        if(g.triangleMesh->getTriangleMeshFlags()&PxTriangleMeshFlag::e16_BIT_INDICES){
          rai::Array<uint16_t> T16;
          T16.referTo((uint16_t*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
          mesh.T = rai::convert<uint>(T16);
        }else{
          mesh.T.referTo((uint*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
        }
        mesh.glDraw(gl);
      } break;

      default:
        RAI_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void PhysXInterface::glDraw(OpenGL& gl) {
  gl.text.clear() <<self->stepCount;
  for(PxRigidActor* a: self->actors) {
    if(a) {
      rai::Frame* f = (rai::Frame*)a->userData;
      DrawActor(a, f, gl);
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

void PhysXInterface::setGravity(float grav){
  self->gScene->setGravity(PxVec3(0.f, 0.f, grav));
}

void PhysXInterface::disableGravity(rai::Frame* f, bool disable){
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

rai::PhysX_Options& PhysXInterface::opt(){
  return self->opt;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#else //RAI_PHYSX

#include "kin_physx.h"
PhysXInterface::PhysXInterface(const rai::Configuration& C, int verbose) : self(nullptr) { NICO }
PhysXInterface::~PhysXInterface() { NICO }

void PhysXInterface::step(double tau) { NICO }
void PhysXInterface::pushKinematicStates(const rai::Configuration& C) { NICO }
void PhysXInterface::pushFullState(const rai::Configuration& C, const arr& vels, bool onlyKinematic) { NICO }
void PhysXInterface::pullDynamicStates(rai::Configuration& C, arr& vels) { NICO }
void PhysXInterface::setMotorQ(const arr& q_ref, const arr& qDot_ref){ NICO }
void PhysXInterface::setMotorQ(const rai::Configuration& C, bool setHardInstantly){ NICO }
void PhysXInterface::postAddObject(rai::Frame* f) { NICO }

void PhysXInterface::disableGravity(rai::Frame* f, bool disable){ NICO }
void PhysXInterface::changeObjectType(rai::Frame* f, int _type) { NICO }
void PhysXInterface::setArticulatedBodiesKinematic(const rai::Configuration& C) { NICO }
void PhysXInterface::watch(bool pause, const char* txt) { NICO }
void PhysXInterface::glDraw(OpenGL&) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b) { NICO }
void PhysXInterface::addForce(rai::Vector& force, rai::Frame* b, rai::Vector& pos) { NICO }

void glPhysXInterface(void* classP) { NICO }

rai::PhysX_Options& PhysXInterface::opt(){ NICO }

#endif

#ifdef RAI_PHYSX
RUN_ON_INIT_BEGIN(kin_physx)
rai::Array<PxRigidActor*>::memMove=true;
rai::Array<PxD6Joint*>::memMove=true;
rai::Array<rai::BodyType>::memMove=true;
RUN_ON_INIT_END(kin_physx)
#endif
