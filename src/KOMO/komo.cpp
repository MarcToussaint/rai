/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "komo.h"
#include <Algo/spline.h>
#include <iomanip>
#include <Kin/frame.h>
#include <Kin/switch.h>
#include <Kin/kin_swift.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Kin/taskMap_FixSwitchedObjects.h>
#include <Kin/taskMap_QuaternionNorms.h>
#include <Kin/taskMap_AboveBox.h>
#include <Kin/taskMap_InsideBox.h>
#include <Kin/taskMap_AlignStacking.h>
#include <Kin/taskMap_GJK.h>
#include <Kin/taskMap_linTrans.h>
#include <Optim/optimization.h>
#include <Optim/convert.h>

//===========================================================================

double height(const mlr::KinematicWorld& K, const char* name){
    mlr::Frame *f = K.getFrameByName(name);
    mlr::Shape *s = f->shape;
    if(!s){
        for(mlr::Frame *b:f->outLinks) if(b->name==name && b->shape){ s=b->shape; break; }
    }
    CHECK(s, "");
    return s->size(2);// + s->size(3);
}

KOMO::KOMO() : T(0), tau(0.), k_order(2), useSwift(true), opt(NULL), gl(NULL), verbose(1), komo_problem(*this){
  verbose = mlr::getParameter<int>("KOMO/verbose",1);
}

KOMO::~KOMO(){
  if(gl) delete gl;
  if(opt) delete opt;
}

KOMO::KOMO(const Graph& specs) : KOMO(){
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& specs){
//  specs = _specs;

  Graph &glob = specs.get<Graph>("KOMO");
  stepsPerPhase=glob.get<double>("T");
  double duration=glob.get<double>("duration");
  maxPhase=glob.get<double>("phases", 1.);
  k_order=glob.get<double>("k_order", 2);

  if(glob["model"]){
    mlr::FileToken model = glob.get<mlr::FileToken>("model");
    world.read(model);
  }else{
    world.init(specs);
  }

  if(glob["meldFixedJoints"]){
    world.optimizeTree();
  }

  if(glob["makeConvexHulls"])
    makeConvexHulls(world.frames);

  if(glob["computeOptimalSSBoxes"]){
    NIY;
    //for(mlr::Shape *s: world.shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(glob["activateAllContacts"])
    for(mlr::Frame *a : world.frames) if(a->shape) a->shape->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

//  if(MP) delete MP;
//  MP = new KOMO(world);
  if(stepsPerPhase>=0) setTiming(maxPhase, stepsPerPhase, duration);
//  MP->k_order=k_order;

  for(Node *n:specs) parseTask(n, stepsPerPhase);
}

void KOMO::setFact(const char* fact){
  Graph specs;
  specs.readNode(STRING(fact));
  parseTask(specs.last());
}

void KOMO::setModel(const mlr::KinematicWorld& K,
                    bool _useSwift,
                    bool meldFixedJoints, bool makeConvexHulls, bool computeOptimalSSBoxes, bool activateAllContacts){

  world.copy(K);

  useSwift = _useSwift;

  if(meldFixedJoints){
    world.optimizeTree();
  }

  if(makeConvexHulls){
    ::makeConvexHulls(world.frames);
  }
  computeMeshNormals(world.frames);

  if(computeOptimalSSBoxes){
    NIY;
    //for(mlr::Shape *s: world.shapes) s->mesh.computeOptimalSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(activateAllContacts){
    for(mlr::Frame *a : world.frames) if(a->shape) a->shape->cont=true;
    world.swift().initActivations(world);
  }

  FILE("z.komo.model") <<world;
}

void KOMO::useJointGroups(const StringA& groupNames, bool OnlyTheseOrNotThese){
  mlr::Joint *j;
  for(mlr::Frame *f:world.frames) if((j=f->joint())){
    bool lock;
    if(OnlyTheseOrNotThese){ //only these
      lock=true;
      for(const mlr::String& s:groupNames) if(f->ats.getNode(s)){ lock=false; break; }
    }else{
      lock=false;
      for(const mlr::String& s:groupNames) if(f->ats.getNode(s)){ lock=true; break; }
    }
    if(lock) j->makeRigid();
  }

  world.reset_q();
  world.optimizeTree();
  world.getJointState();

//  world.meldFixedJoints();
//  world.removeUselessBodies();

  FILE("z.komo.model") <<world;
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order){
  maxPhase = _phases;
  stepsPerPhase = _stepsPerPhase;
  if(stepsPerPhase>=0){
    T = stepsPerPhase*maxPhase;
    CHECK(T, "using T=0 to indicate inverse kinematics is deprecated.");
    tau = durationPerPhase*maxPhase/T;
  }
//    setTiming(stepsPerPhase*maxPhase, durationPerPhase*maxPhase);
  k_order = _k_order;
}

void KOMO::activateCollisions(const char* s1, const char* s2){
  mlr::Frame *sh1 = world.getFrameByName(s1);
  mlr::Frame *sh2 = world.getFrameByName(s2);
  if(sh1 && sh2) world.swift().activate(sh1, sh2);
}

void KOMO::deactivateCollisions(const char* s1, const char* s2){
  mlr::Frame *sh1 = world.getFrameByName(s1);
  mlr::Frame *sh2 = world.getFrameByName(s2);
  if(sh1 && sh2) world.swift().deactivate(sh1, sh2);
}

//===========================================================================
//
// task specs
//

//#define STEP(t) (floor(t*double(stepsPerPhase) + .500001))-1

Task* KOMO::addTask(const char* name, TaskMap *m, const ObjectiveType& termType){
  Task *t = new Task(m, termType);
  t->name=name;
  tasks.append(t);
  return t;
}

bool KOMO::parseTask(const Node *n, int stepsPerPhase){
  if(stepsPerPhase==-1) stepsPerPhase=T;
  //-- task?
  Task *task = Task::newTask(n, world, stepsPerPhase, T);
  if(task){
    tasks.append(task);
    return true;
  }
  //-- switch?
  mlr::KinematicSwitch *sw = mlr::KinematicSwitch::newSwitch(n, world, stepsPerPhase, T);
  if(sw){
    switches.append(sw);
    return true;
  }
//  LOG(-1) <<"task spec '" <<*n <<"' could not be parsed";
  return false;
}

Task *KOMO::setTask(double startTime, double endTime, TaskMap *map, ObjectiveType type, const arr& target, double prec, uint order){
  CHECK(k_order>=order,"");
  map->order = order;
  Task *task = addTask(map->shortTag(world), map, type);
  task->setCostSpecs(startTime, endTime, stepsPerPhase, T, target, prec);
  return task;
}

void KOMO::setKinematicSwitch(double time, bool before, mlr::KinematicSwitch *sw){
  sw->setTimeOfApplication(time, before, stepsPerPhase, T);
  switches.append(sw);
}

void KOMO::setKinematicSwitch(double time, bool before, const char* type, const char* ref1, const char* ref2, const mlr::Transformation& jFrom, const mlr::Transformation& jTo){
  mlr::KinematicSwitch *sw = mlr::KinematicSwitch::newSwitch(type, ref1, ref2, world, 0/*STEP(time)+(before?0:1)*/, jFrom, jTo );
  setKinematicSwitch(time, before, sw);
}

void KOMO::setKS_placeOn(double time, bool before, const char* obj, const char* table, bool actuated){
  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, obj);

  //connect object to table
  mlr::Transformation rel = 0;
  rel.addRelativeTranslation( 0., 0., .5*(height(world, obj) + height(world, table)));
  if(!actuated)
    setKinematicSwitch(time, before, "transXYPhiZero", table, obj, rel );
  else
    setKinematicSwitch(time, before, "transXYPhiActuated", table, obj, rel );
}

void KOMO::setKS_slider(double time, bool before, const char* obj, const char* slider, const char* table){
  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, obj);

  //the two slider objects
  mlr::String slidera = STRING(slider <<'a');
  mlr::String sliderb = STRING(slider <<'b');

  //disconnect object from grasp ref
  setKinematicSwitch(time, before, "delete", NULL, slidera);

  mlr::Transformation rel = 0;
  rel.addRelativeTranslation( 0., 0., .5*(height(world, obj) + height(world, table)));

  setKinematicSwitch(time, true, "transXYPhiZero", table, slidera, rel);
  setKinematicSwitch(time, true, "hingeZZero", sliderb, obj);

//  setKinematicSwitch(time, before, "sliderMechanism", table, obj, rel );

//  if(!actuated)
//    setKinematicSwitch(time, before, "hingeZZero", slider, obj, rel );
//  else
//    setKinematicSwitch(time, before, "transXActuated", slider, obj, rel );
}

void KOMO::setHoming(double startTime, double endTime, double prec){
  uintA bodies;
  mlr::Joint *j;
  for(mlr::Frame *f:world.frames) if((j=f->joint()) && !j->constrainToZeroVel && j->qDim()>0) bodies.append(f->ID);
  setTask(startTime, endTime, new TaskMap_qItself(bodies, true), OT_sumOfSqr, NoArr, prec); //world.q, prec);
}

void KOMO::setSquaredQAccelerations(double startTime, double endTime, double prec){
  CHECK(k_order>=2,"");
  setTask(startTime, endTime, new TaskMap_Transition(world), OT_sumOfSqr, NoArr, prec, 2);
}

void KOMO::setSquaredQVelocities(double startTime, double endTime, double prec){
  auto *map = new TaskMap_Transition(world);
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  setTask(startTime, endTime, map, OT_sumOfSqr, NoArr, prec, 1);
}

void KOMO::setFixEffectiveJoints(double startTime, double endTime, double prec){
  auto *map = new TaskMap_Transition(world, true);
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  setTask(startTime, endTime, map, OT_eq, NoArr, prec, 1);
}

void KOMO::setFixSwitchedObjects(double startTime, double endTime, double prec){
    setTask(startTime, endTime, new TaskMap_FixSwichedObjects(), OT_eq, NoArr, prec, 1);
}

void KOMO::setSquaredQuaternionNorms(double startTime, double endTime, double prec){
    setTask(startTime, endTime, new TaskMap_QuaternionNorms(), OT_sumOfSqr, NoArr, prec);
}

void KOMO::setHoldStill(double startTime, double endTime, const char* shape, double prec){
  mlr::Frame *s = world.getFrameByName(shape);
  setTask(startTime, endTime, new TaskMap_qItself(TUP(s->ID)), OT_sumOfSqr, NoArr, prec, 1);
}

void KOMO::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);
}

void KOMO::setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec, 1);
}

void KOMO::setLastTaskToBeVelocity(){
  tasks.last()->map->order = 1; //set to be velocity!
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO::setGrasp(double time, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift){
  if(verbose>0) cout <<"KOMO_setGrasp t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;
//  mlr::String& endeffRef = world.getFrameByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;

  //-- position the hand & graspRef
  //hand upright
//  setTask(time, time, new TaskMap_Default(vecTMT, world, endeffRef, Vector_z), OT_sumOfSqr, {0.,0.,1.}, weightFromTop);

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TaskMap_Default(posDiffTMT, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 1e3);

  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, endeffRef, Vector_x, object, Vector_x), OT_sumOfSqr, NoArr, 1e1);
  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, endeffRef, Vector_y, object, Vector_x), OT_sumOfSqr, {-1.}, 1e1);

  //hand touches object
//  mlr::Shape *endeffShape = world.getFrameByName(endeffRef)->body->shapes.first();
//  setTask(time, time, new TaskMap_GJK(endeffShape, world.getFrameByName(object), false), OT_eq, NoArr, 1e3);


  //disconnect object from table
  setKinematicSwitch(time, true, "delete", NULL, object);
  //connect graspRef with object
#if 1
  setKinematicSwitch(time, true, "ballZero", endeffRef, object);
  setKinematicSwitch(time, true, "insert_transX", NULL, object);
//  setKinematicSwitch(time, true, "insert_trans3", NULL, object);
  setTask(time, time, new TaskMap_InsideBox(world, endeffRef, NoVector, object), OT_ineq, NoArr, 1e2);
#else
  setKinematicSwitch(time, true, "freeZero", endeffRef, object);
  setTask(time, time, new TaskMap_InsideBox(world, endeffRef, object), OT_ineq, NoArr, 1e2);
//  setTask(time, time, new TaskMap_Default(posDiffTMT, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 1e3);
#endif

  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-timeToLift, time-2.*timeToLift/3, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time-timeToLift/3,  time+timeToLift/3, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,0.}, 1e1, 1); //move down
    setTask(time+2.*timeToLift/3, time+timeToLift, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO::setGraspStick(double time, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift){
  if(verbose>0) cout <<"KOMO_setGraspStick t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;

  //disconnect object from table
  setKinematicSwitch(time, true, "delete", NULL, object);

  //connect graspRef with object
  setKinematicSwitch(time, true, "ballZero", endeffRef, object);
  setKinematicSwitch(time, true, "insert_transX", NULL, object);
//  setTask(time, time,
//          new TaskMap_LinTrans(
//              new TaskMap_Default(posDiffTMT, world, endeffRef, NoVector, object, NoVector),
//              arr(2,3,{0,1,0,0,0,1}), {}),
//          OT_eq, NoArr, 1e3);
  setTask(time, time, new TaskMap_InsideBox(world, endeffRef, NoVector, object), OT_ineq, NoArr, 1e2);


  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-timeToLift, time, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+timeToLift, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// slide on table while grasping rigidly (kinematic loop)
void KOMO::setGraspSlide(double startTime, double endTime, const char* endeffRef, const char* object, const char* placeRef, int verbose, double weightFromTop){
  if(verbose>0) cout <<"KOMO_setGraspSlide t=" <<startTime <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;

  //-- grasp part
  //hand upright
  setTask(startTime, startTime, new TaskMap_Default(vecTMT, world, endeffRef, Vector_z), OT_sumOfSqr, {0.,0.,1.}, weightFromTop);

  //disconnect object from table
  setKinematicSwitch(startTime, true, "delete", placeRef, object);
  //connect graspRef with object
  setKinematicSwitch(startTime, true, "ballZero", endeffRef, object);

  //-- place part
  //place inside box support
  setTask(endTime, endTime, new TaskMap_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e2);

  //disconnect object from grasp ref
  setKinematicSwitch(endTime, true, "delete", endeffRef, object);

  //connect object to table
  mlr::Transformation rel = 0;
  rel.pos.set(0,0, .5*(height(world, object) + height(world, placeRef)));
  setKinematicSwitch(endTime, true, "transXYPhiZero", placeRef, object, rel );

  //-- slide constraints!
  setTask(startTime, endTime,
          new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, world, object, NoVector, placeRef), ~ARR(0,0,1), ARR(0)),
                               OT_sumOfSqr, ARR(rel.pos.z), 1e2);

  if(stepsPerPhase>2){ //velocities down and up
    setTask(startTime-.15, startTime, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(endTime, endTime+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// standard place on a table
void KOMO::setPlace(double time, const char* endeff, const char* object, const char* placeRef, int verbose){
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" obj=" <<object <<" place=" <<placeRef <<endl;

  if(stepsPerPhase>2){ //velocities down and up
    if(endeff){
      setTask(time-.15, time-.10, new TaskMap_Default(posTMT, world, endeff), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
      setTask(time-.05, time+.05, new TaskMap_Default(posTMT, world, endeff), OT_sumOfSqr, {0.,0.,0. }, 1e2, 1); //hold still
      setTask(time+.10, time+.15, new TaskMap_Default(posTMT, world, endeff), OT_sumOfSqr, {0.,0.,+.1}, 1e1, 1); //move up
    }else{
      setTask(time-.15, time, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    }
  }

  //place upright
  setTask(time-.02, time, new TaskMap_Default(vecTMT, world, object, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);

  //place inside box support
  setTask(time, time, new TaskMap_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e2);

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", NULL, object);

  //connect object to placeRef
  mlr::Transformation rel = 0;
  rel.pos.set(0,0, .5*(height(world, object) + height(world, placeRef)));
  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );
}

/// place with a specific relative pose -> no effective DOFs!
void KOMO::setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const mlr::Transformation& relPose, int verbose){
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<" place=" <<placeRef <<endl;

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", endeffRef, object);

  //connect object to table
  setKinematicSwitch(time, true, "rigidZero", placeRef, object, relPose );

  if(stepsPerPhase>2){ //velocities down and up
    setTask(time-.15, time, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); //move down
    setTask(time, time+.15, new TaskMap_Default(posTMT, world, endeffRef), OT_sumOfSqr, {0.,0.,.1}, 1e1, 1); // move up
  }
}

/// switch attachemend (-> ball eDOF)
void KOMO::setHandover(double time, const char* oldHolder, const char* object, const char* newHolder, int verbose){
  if(verbose>0) cout <<"KOMO_setHandover t=" <<time <<" oldHolder=" <<oldHolder <<" obj=" <<object <<" newHolder=" <<newHolder <<endl;

  //hand center at object center (could be replaced by touch)

  //disconnect object from table
  setKinematicSwitch(time, true, "delete", oldHolder, object);
  //connect graspRef with object
#if 0
  setKinematicSwitch(time, true, "ballZero", newHolder, object); //why does this sometimes lead to worse motions?
#else
  setKinematicSwitch(time, true, "freeZero", newHolder, object);
  setTask(time, time, new TaskMap_Default(posDiffTMT, world, newHolder, NoVector, object, NoVector), OT_eq, NoArr, 1e3);
#endif

  if(stepsPerPhase>2){ //velocities: no motion
    setTask(time-.15, time+.15, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,0.}, 1e1, 1); // no motion
  }

}

void KOMO::setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose){
  if(verbose>0) cout <<"KOMO_setPush t=" <<startTime <<" stick=" <<stick <<" object=" <<object <<" table=" <<table <<endl;

  setTask(startTime, startTime+1., new TaskMap_Default(vecAlignTMT, world, stick, -Vector_y, "slider1b", Vector_x), OT_sumOfSqr, {1.}, 1e2);
//  setTask(startTime, startTime+1., new TaskMap_Default(vecAlignTMT, world, stick, Vector_x, "slider1b", Vector_x), OT_sumOfSqr, {0.}, 1e1);
  setTask(startTime, startTime+1., new TaskMap_Default(vecAlignTMT, world, stick, Vector_x, NULL, Vector_z), OT_sumOfSqr, {0.}, 1e2);
//  setTask(startTime, startTime+1., new TaskMap_Default(posDiffTMT, world, stick, NoVector, "slider1b", {.12, .0, .0}), OT_sumOfSqr, {}, 1e2);
  setTask(startTime, startTime+1., new TaskMap_InsideBox(world, "slider1b", {.12, .0, .0}, stick), OT_ineq);

  setKS_slider(startTime, true, object, "slider1", table);

  if(stepsPerPhase>2){ //velocities down and up
    setTask(startTime-.3, startTime, new TaskMap_Default(posTMT, world, stick), OT_sumOfSqr, {0.,0., -.2}, 1e2, 1); //move down
    setTask(startTime+1., startTime+1.3, new TaskMap_Default(posTMT, world, stick), OT_sumOfSqr, {0.,0., .2}, 1e2, 1); // move up
  }
}

void KOMO::setDrop(double time, const char* object, const char* from, const char* to, int verbose){

  //disconnect object from anything
  setKinematicSwitch(time, true, "delete", NULL, object);

  //connect to world with lift
//  setKinematicSwitch(time, true, "JT_trans3", "world", object);
  setKinematicSwitch(time, true, new mlr::KinematicSwitch(mlr::KinematicSwitch::addActuated, mlr::JT_transZ, "world", object, world, 0));
  setKinematicSwitch(time, true, new mlr::KinematicSwitch(mlr::KinematicSwitch::insertJoint, mlr::JT_transXY, NULL, object, world, 0));


  if(stepsPerPhase>2){ //velocities down and up
    setTask(time, time+.2, new TaskMap_Default(posTMT, world, object), OT_sumOfSqr, {0.,0.,-.1}, 1e1, 1); // move down
  }


}

void KOMO::setAttach(double time, const char* endeff, const char* object1, const char* object2, mlr::Transformation& rel, int verbose){
  if(verbose>0) cout <<"KOMO_setAttach t=" <<time <<" endeff=" <<endeff <<" obj1=" <<object1 <<" obj2=" <<object2 <<endl;

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TaskMap_Default(posTMT, world, object2, NoVector, object1, NoVector), OT_sumOfSqr, rel.pos.getArr(), 1e3);
//  setTask(time, time, new TaskMap_Default(quatDiffTMT, world, object2, NoVector, object1, NoVector), OT_sumOfSqr, conv_quat2arr(rel.rot), 1e3);

//  setTask(time, time, new TaskMap_Default(vecAlignTMT, world, newHolder, Vector_y, object, Vector_x), OT_sumOfSqr, {-1.}, 1e1);

  //disconnect object from grasp ref
  setKinematicSwitch(time, true, "delete", endeff, object2);

//  mlr::Transformation rel = 0;
//  rel.addRelativeTranslation( 0., 0., .5*(height(world.getFrameByName(object)) + height(world.getFrameByName(placeRef))));
  setKinematicSwitch(time, true, "rigidZero", object1, object2, rel );

}

void KOMO::setSlowAround(double time, double delta, double prec){
  if(stepsPerPhase>2) //otherwise: no velocities
    setTask(time-delta, time+delta, new TaskMap_qItself(), OT_sumOfSqr, NoArr, prec, 1);
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=1e1 } #slow down
}

void KOMO::setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize, const char* gripper, const char* gripper2){
  double t1=-.25; //time when gripper is positined above
  double t2=-.1;  //time when gripper is lowered
  double t3=-.05; //time when gripper is closed

  //position above
  setTask(time+t1, 1., new TaskMap_Default(vecTMT, world, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0);
  setTask(time+t1, t1, new TaskMap_Default(posDiffTMT, world, endeff, NoVector, object, NoVector), OT_sumOfSqr, {0.,0.,above+.1}, 1e3);
  setTask(time+t1, 1., new TaskMap_Default(vecAlignTMT, world, endeff, Vector_x, object, Vector_y), OT_sumOfSqr, NoArr, 1e1);
  setTask(time+t1, 1., new TaskMap_Default(vecAlignTMT, world, endeff, Vector_x, object, Vector_z), OT_sumOfSqr, NoArr, 1e1);
  //open gripper
  if(gripper)  setTask(time+t1, .85, new TaskMap_qItself(QIP_byJointNames, {gripper}, world), OT_sumOfSqr, {gripSize + .05});
  if(gripper2) setTask(time+t1, .85, new TaskMap_qItself(QIP_byJointNames, {gripper2}, world), OT_sumOfSqr, {::asin((gripSize + .05)/(2.*.10))});
  //lower
  setTask(time+t2, 1., new TaskMap_Default(posDiffTMT, world, endeff, NoVector, object, NoVector), OT_sumOfSqr, {0.,0.,above}, 1e3);
  //close gripper
  if(gripper)  setTask(time+t3, 1., new TaskMap_qItself(QIP_byJointNames, {gripper}, world), OT_sumOfSqr, {gripSize});
  if(gripper2) setTask(time+t3, 1., new TaskMap_qItself(QIP_byJointNames, {gripper2}, world), OT_sumOfSqr, {::asin((gripSize)/(2.*.10))});
  setSlowAround(time, .05, 1e3);
}

/// translate a list of facts (typically facts in a FOL state) to LGP tasks
void KOMO::setAbstractTask(double phase, const Graph& facts, int verbose){
//  CHECK(phase<=maxPhase,"");
//  listWrite(facts, cout,"\n");  cout <<endl;
  for(Node *n:facts){
    if(!n->parents.N) continue;
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());
    if(n->keys.N && n->keys.last().startsWith("komo")){
      double time=n->get<double>(); //komo tag needs to be double valued!
      if(n->keys.last()=="komoGrasp")         setGrasp(phase+time, *symbols(0), *symbols(1), verbose);
      else if(n->keys.last()=="komoPlace"){
        if(symbols.N==2) setPlace(phase+time, NULL, *symbols(0), *symbols(1), verbose);
        else  setPlace(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      }else if(n->keys.last()=="komoHandover") setHandover(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoPush")     setPush(phase+time, phase+time+1., *symbols(0), *symbols(1), *symbols(2), verbose); //TODO: the +1. assumes pushes always have duration 1
      else if(n->keys.last()=="komoDrop")     setDrop(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoAttach"){
        Node *attachableSymbol = facts.getNode("attachable");
        CHECK(attachableSymbol!=NULL,"");
        Node *attachableFact = facts.getEdge({attachableSymbol, n->parents(1), n->parents(2)});
        mlr::Transformation rel = attachableFact->get<mlr::Transformation>();
        setAttach(phase+time, *symbols(0), *symbols(1), *symbols(2), rel, verbose);
      }else HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}

void KOMO::setAlign(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, ObjectiveType type, const arr& target, double prec){
#if 0
  mlr::String map;
  map <<"map=vecAlign ref1="<<shape;
  if(whichAxis) map <<" vec1=[" <<whichAxis <<']';
  if(shapeRel) map <<" ref2=" <<shapeRel <<" vec2=" <<;
  if(whichAxisRel) map <<" vec2=[" <<whichAxisRel <<']';
  setTask(startTime, endTime, map, type, target, prec);
#else
  setTask(startTime, endTime, new TaskMap_Default(vecAlignTMT, world, shape, mlr::Vector(whichAxis), shapeRel, mlr::Vector(whichAxisRel)), type, target, prec);
#endif

}

void KOMO::setTouch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type, const arr& target, double prec){
  setTask(startTime, endTime, new TaskMap_GJK(world, shape1, shape2, true), type, target, prec);
}

void KOMO::setAlignedStacking(double time, const char* object, ObjectiveType type, double prec){
  setTask(time, time, new TaskMap_AlignStacking(world, object), type, NoArr, prec);
}

void KOMO::setCollisions(bool hardConstraint, double margin, double prec){
  if(hardConstraint){ //interpreted as hard constraint (default)
    setTask(0., -1., new CollisionConstraint(margin), OT_ineq, NoArr, prec);
  }else{ //cost term
    setTask(0., -1., new TaskMap_Proxy(allPTMT, {0u}, margin, true  ), OT_sumOfSqr, NoArr, prec);
  }
}

void KOMO::setLimits(bool hardConstraint, double margin, double prec){
  if(hardConstraint){ //interpreted as hard constraint (default)
    setTask(0., -1., new LimitsConstraint(margin), OT_ineq, NoArr, prec);
  }else{ //cost term
    NIY;
//    setTask(0., -1., new TaskMap_Proxy(allPTMT, {0u}, margin), OT_sumOfSqr, NoArr, prec);
  }
}

//===========================================================================
//
// config
//


void KOMO::setConfigFromFile(){
  mlr::KinematicWorld K(mlr::getParameter<mlr::String>("KOMO/modelfile"));
//  K.optimizeTree();
  setModel(
        K,
        mlr::getParameter<bool>("KOMO/useSwift", true),
        mlr::getParameter<bool>("KOMO/meldFixedJoints", false),
        mlr::getParameter<bool>("KOMO/makeConvexHulls", true),
        mlr::getParameter<bool>("KOMO/computeOptimalSSBoxes", false),
        mlr::getParameter<bool>("KOMO/activateAllContact", false)
        );
  setTiming(
        mlr::getParameter<uint>("KOMO/phases"),
        mlr::getParameter<uint>("KOMO/stepsPerPhase"),
        mlr::getParameter<double>("KOMO/durationPerPhase", 5.),
        mlr::getParameter<uint>("KOMO/k_order", 2)
        );
}

void KOMO::setPoseOpt(){
  setTiming(1., 2, 5., 1);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQVelocities();
  setSquaredQuaternionNorms();
}

void KOMO::setSequenceOpt(double _phases){
  setTiming(_phases, 2, 5., 1);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQVelocities();
  setSquaredQuaternionNorms();
}

void KOMO::setPathOpt(double _phases, uint stepsPerPhase, double timePerPhase){
  setTiming(_phases, stepsPerPhase, timePerPhase, 2);
  setFixEffectiveJoints();
  setFixSwitchedObjects();
  setSquaredQAccelerations();
  setSquaredQuaternionNorms();
}

void setTasks(KOMO& MP,
              mlr::Frame& endeff,
              mlr::Frame& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration){

  //-- parameters
  double posPrec = mlr::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = mlr::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = mlr::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = mlr::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

  //-- set up the KOMO
  target.shape->cont=false; //turn off contact penalization with the target

//  MP.world.swift().initActivations(MP.world);
  //MP.world.watch(false);

  MP.setTiming(1., mlr::getParameter<uint>("timeSteps", 50), mlr::getParameter<double>("duration", 5.));
  if(timeSteps>=0) MP.setTiming(1., timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task *t;

  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), OT_sumOfSqr);
  if(timeSteps!=0){
    t->map->order=2; //make this an acceleration task!
  }else{
    t->map->order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T-1, {0.}, 1e0);

  if(timeSteps!=0){
    t = MP.addTask("final_vel", new TaskMap_qItself(), OT_sumOfSqr);
    t->map->order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T-1, {0.}, zeroVelPrec);
  }

  if(colPrec<0){ //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), OT_ineq);
    t->setCostSpecs(0, MP.T-1, {0.}, 1.);
  }else{ //cost term
    t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0u}, margin), OT_sumOfSqr);
    t->setCostSpecs(0, MP.T-1, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new TaskMap_Default(posTMT, endeff.ID, NoVector, target.ID, NoVector), OT_sumOfSqr);
  t->setCostSpecs(MP.T-1, MP.T-1, {0.}, posPrec);


  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    mlr::Vector axis;
    axis.setZero();
    axis(i)=1.;
    t = MP.addTask(STRING("endeff_align_"<<i),
                   new TaskMap_Default(vecAlignTMT, endeff.ID, axis, target.ID, axis),
                   OT_sumOfSqr);
    t->setCostSpecs(MP.T-1, MP.T-1, {1.}, alignPrec);
  }
}

void KOMO::setMoveTo(mlr::KinematicWorld& world, mlr::Frame& endeff, mlr::Frame& target, byte whichAxesToAlign){
//  if(MP) delete MP;
//  MP = new KOMO(world);
  setModel(world);
  this->world.checkConsistency();


  setTasks(*this, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

void KOMO::setSpline(uint splineT){
  mlr::Spline S;
  S.setUniformNonperiodicBasis(T-1, splineT, 2);
  uint n=dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0;i<S.basis.d0;i++) for(uint j=0;j<S.basis.d1;j++)
    splineB.setMatrixBlock(S.basis(i,j)*eye(n,n), i*n, j*n);
  z = pseudoInverse(splineB) * x;
}

void KOMO::reset(){
  x = getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config
  if(splineB.N){
    z = pseudoInverse(splineB) * x;
  }
}

void KOMO::run(){
  mlr::KinematicWorld::setJointStateCount=0;
  mlr::timerStart();
  CHECK(T,"");
  if(opt) delete opt;
  if(!splineB.N){
    Convert C(komo_problem);
    opt = new OptConstrained(x, dual, C);
    opt->run();
  }else{
    arr a,b,c,d,e;
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P(P0, splineB);
    opt = new OptConstrained(z, dual, P);
    opt->run();
  }
  if(verbose>0){
    cout <<"** optimization time=" <<mlr::timerRead()
      <<" setJointStateCount=" <<mlr::KinematicWorld::setJointStateCount <<endl;
  }
  if(verbose>1) cout <<getReport(false);
}

void KOMO::reportProblem(std::ostream& os){
    os <<"KOMO Problem:" <<endl;
    os <<"  x-dim:" <<x.N <<"  dual-dim:" <<dual.N <<endl;
    os <<"  T:" <<T <<" k:" <<k_order <<" phases:" <<maxPhase <<" stepsPerPhase:" <<stepsPerPhase <<" tau:" <<tau <<endl;
    os <<"  #configurations:" <<configurations.N <<" q-dims: ";
    uintA dims(configurations.N);
    for(uint i=0;i<configurations.N;i++) dims(i)=configurations(i)->q.N;
    writeConsecutiveConstant(os, dims);
    os <<endl;

    os <<"  usingSwift:" <<useSwift <<endl;
    for(Task* t:tasks) os <<"    " <<*t <<endl;
    for(mlr::KinematicSwitch* sw:switches){
        os <<"    ";
        CHECK_LE(sw->timeOfApplication+k_order, configurations.N, "switch time is beyond time horizon");
        sw->write(os, configurations(sw->timeOfApplication+k_order));
        os <<endl;
    }
}


void KOMO::checkGradients(){
  CHECK(T,"");
  if(!splineB.N)
    checkJacobianCP(Convert(komo_problem), x, 1e-4);
  else{
    Conv_KOMO_ConstrainedProblem P0(komo_problem);
    Conv_linearlyReparameterize_ConstrainedProblem P1(P0, splineB);
    checkJacobianCP(P1, z, 1e-4);
  }
}

void KOMO::plotTrajectory(){
  ofstream fil("z.trajectories");
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  x.reshape(T, world.q.N);
  x.write(fil, NULL, NULL, "  ");
  fil.close();

  ofstream fil2("z.trajectories.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'trajectories'" <<endl;
  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot 'z.trajectories' \\" <<endl;
  for(uint i=1;i<=jointNames.N;i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
//    if(dualSolution.N) for(uint i=0;i<tasks.N;i++) fil <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  gnuplot("load 'z.trajectories.plt'");
}

bool KOMO::displayTrajectory(double delay, bool watch){
//  return displayTrajectory(watch?-1:1, "KOMO planned trajectory", delay);
  const char* tag = "KOMO planned trajectory";
  if(!gl){
    gl = new OpenGL ("KOMO display");
    gl->camera.setDefault();
  }

  for(uint t=0; t<T; t++) {
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->addDrawer(configurations(t+k_order));
    if(delay<0.){
      if(delay<-10.) FILE("z.graph") <<*configurations(t+k_order);
      gl->watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    }else{
      gl->update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) mlr::wait(delay);
    }
  }
  if(watch){
    int key = gl->watch(STRING(tag <<" (time " <<std::setw(3) <<T-1 <<'/' <<T <<')').p);
    return !(key==27 || key=='q');
  }else
    return false;
}


mlr::Camera& KOMO::displayCamera(){
  if(!gl){
    gl = new OpenGL ("KOMO display");
    gl->camera.setDefault();
  }
  return gl->camera;
}

//===========================================================================

#define KOMO KOMO

void KOMO::setupConfigurations(){

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(!configurations.N,"why setup again?");
//    listDelete(configurations);

  if(useSwift) {
    makeConvexHulls(world.frames);
    world.swift().setCutoff(2.*mlr::getParameter<double>("swiftCutoff", 0.11));
  }
  computeMeshNormals(world.frames, true);

  configurations.append(new mlr::KinematicWorld())->copy(world, true);
  configurations.last()->calc_q();
  configurations.last()->calc_q_from_Q();
  configurations.last()->checkConsistency();
  for(uint s=1;s<k_order+T;s++){
    configurations.append(new mlr::KinematicWorld())->copy(*configurations(s-1), true);
    CHECK(configurations(s)==configurations.last(), "");
    //apply potential graph switches
    for(mlr::KinematicSwitch *sw:switches){
      if(sw->timeOfApplication+k_order==s){
        sw->apply(*configurations(s));
      }
    }
    configurations(s)->calc_q();
    configurations(s)->calc_q_from_Q();
    configurations(s)->checkConsistency();
  }
}

void KOMO::set_x(const arr& x){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0;t<T;t++){
    uint s = t+k_order;
    uint x_dim = dim_x(t); //configurations(s)->getJointStateDimension();
    if(x_dim){
      if(x.nd==1)  configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else         configurations(s)->setJointState(x[t]);
      if(useSwift) configurations(s)->stepSwift();
      x_count += x_dim;
    }
  }
  CHECK_EQ(x_count, x.N, "");
}

void KOMO::reportProxies(std::ostream& os){
    int t=0;
    for(auto &K:configurations){
        os <<" **** KOMO PROXY REPORT t=" <<t-k_order <<endl;
        K->reportProxies(os);
        t++;
    }
}


Graph KOMO::getReport(bool gnuplt, int reportFeatures, std::ostream& featuresOs) {
  if(featureValues.N>1){ //old optimizer -> remove some time..
    arr tmp;
    for(auto& p:featureValues) tmp.append(p);
    featureValues = ARRAY<arr>(tmp);

    ObjectiveTypeA ttmp;
    for(auto& p:featureTypes) ttmp.append(p);
    featureTypes = ARRAY<ObjectiveTypeA>(ttmp);
  }

  bool wasRun = featureValues.N!=0;

  arr phi;
  ObjectiveTypeA tt;
  if(wasRun){
      phi.referTo( featureValues.scalar() );
      tt.referTo( featureTypes.scalar() );
  }

  //-- collect all task costs and constraints
  StringA name; name.resize(tasks.N);
  arr err=zeros(T,tasks.N);
  arr taskC=zeros(tasks.N);
  arr taskG=zeros(tasks.N);
  uint M=0;
  for(uint t=0; t<T; t++){
    for(uint i=0; i<tasks.N; i++) {
      Task *task = tasks(i);
      if(task->prec.N>t && task->prec(t)){
          uint d=0;
          if(wasRun){
              d=task->map->dim_phi(configurations({t,t+k_order}), t);
              for(uint j=0;j<d;j++) CHECK(tt(M+j)==task->type,"");
              if(d){
                  if(task->type==OT_sumOfSqr){
                      for(uint j=0;j<d;j++) err(t,i) += mlr::sqr(phi(M+j)); //sumOfSqr(phi.sub(M,M+d-1));
                      taskC(i) += err(t,i);
                  }
                  if(task->type==OT_ineq){
                      for(uint j=0;j<d;j++) err(t,i) += mlr::MAX(0., phi(M+j));
                      taskG(i) += err(t,i);
                  }
                  if(task->type==OT_eq){
                      for(uint j=0;j<d;j++) err(t,i) += fabs(phi(M+j));
                      taskG(i) += err(t,i);
                  }
                  M += d;
              }
          }
          if(reportFeatures==1){
            featuresOs <<std::setw(4) <<t <<' ' <<std::setw(2) <<i <<' ' <<std::setw(2) <<d
                <<' ' <<std::setw(40) <<task->name
               <<" k=" <<task->map->order <<" ot=" <<task->type <<" prec=" <<std::setw(4) <<task->prec(t);
            if(task->target.N<5) featuresOs <<" y*=[" <<task->target <<']'; else featuresOs<<"y*=[..]";
            featuresOs <<" y^2=" <<err(t,i) <<endl;
        }
      }
    }
  }
  CHECK_EQ(M , phi.N, "");


  //-- generate a report graph
  Graph report;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task *c = tasks(i);
    Graph *g = &report.newSubgraph({c->name}, {})->value;
    g->newNode<double>({"order"}, {}, c->map->order);
    g->newNode<mlr::String>({"type"}, {}, STRING(c->type.name()));
    g->newNode<double>({"sqrCosts"}, {}, taskC(i));
    g->newNode<double>({"constraints"}, {}, taskG(i));
    totalC += taskC(i);
    totalG += taskG(i);
  }
  report.newNode<double>({"total","sqrCosts"}, {}, totalC);
  report.newNode<double>({"total","constraints"}, {}, totalG);

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks) fil <<c->name <<' ';
  for(auto c:tasks) if(c->type==OT_ineq && dualSolution.N) fil <<c->name <<"_dual ";
  fil <<endl;

  //rest: just the matrix
  if(!dualSolution.N){
    err.write(fil,NULL,NULL,"  ");
  }else{
    dualSolution.reshape(T, dualSolution.N/(T));
    catCol(err, dualSolution).write(fil,NULL,NULL,"  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' \\" <<endl;
  for(uint i=1;i<=tasks.N;i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  if(dualSolution.N) for(uint i=0;i<tasks.N;i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt){
    cout <<"KOMO Report\n" <<report <<endl;
    gnuplot("load 'z.costReport.plt'");
  }

  return report;
}

arr KOMO::getInitialization(){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0;t<T;t++) x.append(configurations(t+k_order)->getJointState());
  return x;
}

void KOMO::Conv_MotionProblem_KOMO_Problem::getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes){
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  variableDimensions.resize(komo.T);
  for(uint t=0;t<komo.T;t++) variableDimensions(t) = komo.configurations(t+komo.k_order)->getJointStateDimension();

  featureTimes.clear();
  featureTypes.clear();
  for(uint t=0;t<komo.T;t++){
    for(Task *task: komo.tasks) if(task->prec.N>t && task->prec(t)){
//      CHECK(task->prec.N<=MP.T,"");
      uint m = task->map->dim_phi(komo.configurations({t,t+komo.k_order}), t); //dimensionality of this task
      featureTimes.append(consts<uint>(t, m));
      featureTypes.append(consts<ObjectiveType>(task->type, m));
    }
  }
  dimPhi = featureTimes.N;
}

void KOMO::Conv_MotionProblem_KOMO_Problem::phi(arr& phi, arrA& J, arrA& H, ObjectiveTypeA& tt, const arr& x){
  //-- set the trajectory
  komo.set_x(x);


  CHECK(dimPhi,"getStructure must be called first");
  phi.resize(dimPhi);
  if(&tt) tt.resize(dimPhi);
  if(&J) J.resize(dimPhi);

  arr y, Jy;
  uint M=0;
  for(uint t=0;t<komo.T;t++){
    for(Task *task: komo.tasks) if(task->prec.N>t && task->prec(t)){
        //TODO: sightly more efficient: pass only the configurations that correspond to the map->order
      task->map->phi(y, (&J?Jy:NoArr), komo.configurations({t,t+komo.k_order}), komo.tau, t);
      if(!y.N) continue;
      if(absMax(y)>1e10) MLR_MSG("WARNING y=" <<y);

      //linear transform (target shift)
      if(task->target.N==1) y -= task->target.elem(0);
      else if(task->target.nd==1) y -= task->target;
      else if(task->target.nd==2) y -= task->target[t];
      y *= sqrt(task->prec(t));

      //write into phi and J
      phi.setVectorBlock(y, M);
      if(&J){
        Jy *= sqrt(task->prec(t));
        if(t<komo.k_order) Jy.delColumns(0,(komo.k_order-t)*komo.configurations(0)->q.N); //delete the columns that correspond to the prefix!!
        for(uint i=0;i<y.N;i++) J(M+i) = Jy[i]; //copy it to J(M+i); which is the Jacobian of the M+i'th feature w.r.t. its variables
      }
      if(&tt) for(uint i=0;i<y.N;i++) tt(M+i) = task->type;

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = ARRAY<arr>(phi);
  if(&tt) komo.featureTypes = ARRAY<ObjectiveTypeA>(tt);
}

