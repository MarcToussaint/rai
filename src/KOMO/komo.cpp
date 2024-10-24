/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "komo.h"
#include "komo_NLP.h"
#include "skeletonSymbol.h"

#include "../Gui/opengl.h"
#include "../Geo/fclInterface.h"

#include "../Kin/frame.h"
//#include "../KOMO/switch.h"
#include "../Kin/proxy.h"
#include "../Kin/forceExchange.h"
//#include "../Kin/kin_swift.h"
//#include "../Kin/kin_physx.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/F_pose.h"
#include "../Kin/F_collisions.h"
//#include "../Kin/F_geometrics.h"
//#include "../Kin/F_operators.h"
#include "../Kin/F_forces.h"
#include "../Kin/viewer.h"
#include "../KOMO/switch.h"

//#include "../Optim/constrained.h"
//#include "../Optim/primalDual.h"
//#include "../Optim/opt-nlopt.h"
//#include "../Optim/opt-ipopt.h"
//#include "../Optim/opt-ceres.h"
#include "../Optim/NLP_Solver.h"

#include "../Core/util.ipp"

#include "pathTools.h"

#include <iomanip>

#ifdef RAI_GL
#  include <GL/gl.h>
#endif

#define RAI_USE_FUNCTIONALS

using namespace rai;

//===========================================================================

struct getQFramesAndScale_Return { uintA frames; arr scale; };
getQFramesAndScale_Return getCtrlFramesAndScale(const rai::Configuration& C);

Shape* getShape(const Configuration& K, const char* name) {
  Frame* f = K.getFrame(name);
  Shape* s = f->shape;
  if(!s) {
    for(Frame* b:f->children) if(b->name==name && b->shape) { s=b->shape; break; }
  }
  return s;
}

rai::Transformation relTransformOn(const rai::Configuration& K, const char* name1, const char* name2) {
  rai::Frame* f1 = K.getFrame(name1);
  rai::Frame* f2 = K.getFrame(name2);
  rai::Shape* s1 = f1->shape;
  rai::Shape* s2 = f2->shape;
  if(!s1) { for(rai::Frame* b:f1->children) if(b->name==name1 && b->shape) { s1=b->shape; break; } }
  if(!s2) { for(rai::Frame* b:f2->children) if(b->name==name2 && b->shape) { s2=b->shape; break; } }
  rai::Transformation rel=0;
  CHECK(s1->type()==rai::ST_ssBox, "");
  rel.pos.z += .5*s1->size(2);
  if(s1->type()==rai::ST_marker) {
  } else if(s2->type()==rai::ST_sphere) {
    rel.pos.z += s2->radius();
  } else if(s2->type()==rai::ST_capsule) {
    rel.pos.z += s2->radius();
    rel.rot.setRadX(.5*RAI_PI);
  } else if(s2->type()==rai::ST_ssCylinder) {
    rel.pos.z += .5*s2->size(0);
  }
  return rel;
}

//===========================================================================

KOMO::KOMO() {
}

KOMO::KOMO(const Configuration& C, double _phases, uint _stepsPerPhase, uint _k_order, bool _enableCollisions)
  : KOMO() {
  setTiming(_phases, _stepsPerPhase, 1., _k_order);
  setConfig(C, _enableCollisions);
}

KOMO::~KOMO() {
  objs.clear();
  objectives.clear();
  switches.clear();
}

void KOMO::setConfig(const Configuration& C, bool _computeCollisions) {
  orgJointIndices = C.getDofIDs();
  if(&C!=&world) world.copy(C, _computeCollisions);
  computeCollisions = _computeCollisions;
//  if(computeCollisions) world.fcl();
  world.ensure_q();

  if(stepsPerPhase) setupPathConfig();
}

void KOMO::setTiming(double _phases, uint _stepsPerPhase, double durationPerPhase, uint _k_order) {
  CHECK(_stepsPerPhase, "needs to be nonzero!");
  stepsPerPhase = _stepsPerPhase;
  T = ceil(stepsPerPhase*_phases);
  tau = durationPerPhase/double(stepsPerPhase);
  k_order = _k_order;

  if(world.frames.N) setupPathConfig();
}

void KOMO::clone(const KOMO& komo, bool deepCopyFeatures) {
  clearObjectives();
  opt = komo.opt;
  setConfig(komo.world, komo.computeCollisions);
  //setTiming:
  stepsPerPhase = komo.stepsPerPhase;
  T = komo.T;
  tau = komo.tau;
  k_order = komo.k_order;

  if(komo.fcl) fcl=komo.fcl;
  //if(komo.swift) swift=komo.swift;

  //directly copy pathConfig instead of recreating it (including switches)
  pathConfig.copy(komo.pathConfig, false);
  timeSlices = pathConfig.getFrames(framesToIndices(komo.timeSlices));

  //copy running objectives
  for(const shared_ptr<Objective>& o:komo.objectives) {
    std::shared_ptr<Feature> f = o->feat;
    if(deepCopyFeatures) f = f->deepCopy();
    objectives.append(make_shared<Objective>(f, o->type, o->name, o->times));
  }

  //copy grounded objectives
  for(const shared_ptr<GroundedObjective>& o:komo.objs) {
    std::shared_ptr<Feature> f = o->feat;
    if(deepCopyFeatures) f = f->deepCopy();
    objs.append(make_shared<GroundedObjective>(f, o->type, o->timeSlices));
    objs(-1)->frames = pathConfig.getFrames(framesToIndices(o->frames));
    objs(-1)->objId = o->objId;
  }
}

void KOMO::addTimeOptimization() {
  world.addTauJoint();
  rai::Frame* timeF = world.frames.first();
#if 0 //break the constraint at phase switches: EMPIRICALLY EQUIVALENT TO BELOW (passive_ballBounce TEST)
  shared_ptr<Objective> o = addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1); //smooth time evolution
  if(o->configs.nd==1) { //for KOMO optimization
    CHECK(o->configs.nd==1 && o->configs.N==T, "");
    CHECK_GE(stepsPerPhase, 10, "NIY");
    for(uint t=2; t<o->configs.N; t+=stepsPerPhase) o->configs(t)=0;
  } else { //for sparse optimization
    CHECK(o->configs.nd==2 && o->configs.N==2*T, "");
    CHECK_GE(stepsPerPhase, 10, "NIY");
    for(uint t=o->configs.d0; t--;) if(o->configs(t, 0)%stepsPerPhase==0) o->configs.delRows(t);
  }
#else
  addObjective({0.}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1, 0, +1); //smooth time evolution
  for(uint t=0; t<T/stepsPerPhase; t++) {
    addObjective({double(t), double(t+1)}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e2}, {}, 1, +3, +1); //smooth time evolution
  }
#endif

  addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_sos, {1e-1}, {tau}); //prior on timing
  addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_ineq, {-1e1}, {.9*tau}); //lower bound on timing
}

//===========================================================================
//
// task specs
//

void KOMO::clearObjectives() {
  objectives.clear(); //listDelete(objectives);
  objs.clear();
  switches.clear();
  reset();
}

void KOMO::removeObjective(const Objective* ob) {
  for(const GroundedObjective* o:ob->groundings) {
    for(uint i=objs.N; i--;) if(objs(i).get()==o) objs.remove(i);
  }
  for(uint i=objectives.N; i--;) if(objectives(i).get()==ob) objectives.remove(i);
}

void KOMO::copyObjectives(KOMO& komoB, bool deepCopyFeatures) {
  for(const std::shared_ptr<Objective>& o: komoB.objectives) {
    std::shared_ptr<Feature> f = o->feat;
    if(f->order==2 && k_order<=1) continue;
    if(f->order==1 && (o->times.N==1 || o->times(1)-o->times(0)<1.)) continue;
    if(deepCopyFeatures) f = f->deepCopy();
    //std::shared_ptr<Objective> ocopy = make_shared<Objective>(f, o->type, o->name, o->times);
    addObjective(o->times, f, {}, o->type);
  }
}

void KOMO::_addObjective(const std::shared_ptr<Objective>& ob, const intA& timeSlices) {
  objectives.append(ob);

  CHECK_EQ(timeSlices.nd, 2, "");
  CHECK_EQ(timeSlices.d1, ob->feat->order+1, "");
  for(uint c=0; c<timeSlices.d0; c++) {
    shared_ptr<GroundedObjective> o = make_shared<GroundedObjective>(ob->feat, ob->type, timeSlices[c]);
    objs.append(o);
    ob->groundings.append(o.get());
    o->objId = objectives.N-1;
    o->frames.resize(timeSlices.d1, o->feat->frameIDs.N);
    for(uint i=0; i<timeSlices.d1; i++) {
      int s = timeSlices(c, i) + k_order;
      for(uint j=0; j<o->feat->frameIDs.N; j++) {
        uint fID = o->feat->frameIDs.elem(j);
        o->frames(i, j) = this->timeSlices(s, fID);
      }
    }
    if(o->feat->frameIDs.nd==2) {
      o->frames.reshape(timeSlices.d1, o->feat->frameIDs.d0, o->feat->frameIDs.d1);
    }
  }
}

shared_ptr<Objective> KOMO::addObjective(const arr& times,
    const shared_ptr<Feature>& f, const StringA& frames,
    ObjectiveType type, const arr& scale, const arr& target, int order,
    int deltaFromStep, int deltaToStep) {
  //-- we need the path configuration to ground the objectives
  CHECK(timeSlices.N, "path config is not setup");
//  if(!timeSlices.N) setupPathConfig();

  //-- if arguments are given, modify the feature's frames, scaling and order
  f->setup(world, frames, scale, target, order);

  //-- determine when exactly it is active (list of tuples of given order
  intA timeSlices = conv_times2tuples(times, f->order, stepsPerPhase, T, deltaFromStep, deltaToStep);

  //-- create a (non-grounded) objective
  CHECK_GE(k_order, f->order, "task requires larger k-order: " <<f->shortTag(world));
  std::shared_ptr<Objective> o = make_shared<Objective>(f, type, f->shortTag(world), times);

  //-- create the grounded objectives
  _addObjective(o, timeSlices);
  return o;
}

//void KOMO::addFlag(double time, Flag *fl, int deltaStep) {
//  if(time<0.) time=0.;
//  fl->stepOfApplication = conv_time2step(time, stepsPerPhase) + deltaStep;
//  flags.append(fl);
//}

rai::Frame* KOMO::addSwitch(double time, bool before, const std::shared_ptr<KinematicSwitch>& sw) {
  sw->setTimeOfApplication(time, before, stepsPerPhase, T);
  rai::Frame* f = applySwitch(*sw); //apply immediately
  switches.append(sw); //only to report, not apply in retrospect
  return f;
}

rai::Frame* KOMO::addSwitch(double time, bool before, bool stable,
                            rai::JointType type, SwitchInitializationType init,
                            const char* ref1, const char* ref2,
                            const rai::Transformation& jFrom) {
  auto sw = make_shared<KinematicSwitch>(SW_joint, type, ref1, ref2, world, init, 0, jFrom);
  sw->isStable = stable;
  return addSwitch(time, before, sw);
}

//void KOMO::addStableFrame(SkeletonSymbol newMode, const char* parent, const char* name, const char* toShape) {
//  //---------------- create the switch and limits for the effective dof ---
//  if(newMode==SY_stable) {
//    rai::Frame* f = addStableFrame(JT_free, parent, name);
//    if(f) { //limits?
//      double maxsize = 0.;
//      rai::Shape* from = world.getFrame(parent)->shape;
//      if(from && from->type()!=rai::ST_marker) {
//        if(from->type()==rai::ST_sphere || from->type()==rai::ST_cylinder || from->type()==rai::ST_ssCylinder) {
//          maxsize += 2.*from->size(0);
//        } else {
//          maxsize += absMax(from->size);
//        }
//      } else if(from) {
//        CHECK_EQ(from->type(), ST_marker, "");
//      }
//      rai::Shape* to = world.getFrame(toShape)->shape;
//      if(to && to->type()!=rai::ST_marker) {
//        if(to->type()==rai::ST_sphere || to->type()==rai::ST_cylinder || to->type()==rai::ST_ssCylinder) {
//          maxsize += 2.*to->size(0);
//        } else {
//          maxsize += absMax(to->size);
//        }
//      }
//      if(maxsize>1e-4) {
//        f->joint->limits = {
//          -.9*maxsize, .9*maxsize,
//            -.9*maxsize, .9*maxsize,
//            -.9*maxsize, .9*maxsize,
//            -1.1, 1.1, -1.1, 1.1, -1.1, 1.1, -1.1, 1.1
//          }; //no limits on rotation
//      }
//      //sample heuristic
//      f->joint->sampleUniform=opt.sampleRate_stable;
//      f->joint->q0.clear(); // = zeros(7); f->joint->q0(3)=1.; //.clear();
//    }
//  } else if(newMode==SY_stableOn) {
//    double height = .5*(shapeSize(world.getFrame(parent)) + shapeSize(world.getFrame(toShape)));
//    Transformation rel = 0;
//    rel.pos.set(0, 0, height);
////    rai::Frame* f = addStableFrame(JT_free, parent, name);
//    rai::Frame* f = addStableFrame(JT_transXYPhi, parent, name, 0, rel);
//    //f->joint->setGeneric("xyc");
//    if(false) {
//      //limits?
//      double zero=1e-4;
//      rai::Shape* from = world.getFrame(parent)->shape;
//      CHECK_EQ(from->type(), rai::ST_ssBox, "");
//      f->joint->limits = {-.5*from->size(0), .5*from->size(0),
//                          -.5*from->size(1), .5*from->size(1),
//                          -zero, +zero,
////                          -1.1,1.1, -1.1,1.1, -1.1,1.1, -1.1,1.1 }; //no limits on rotation
//                          -1.1, 1.1, -zero, zero, -zero, zero, -1.1, 1.1
//                         }; //no limits on rotation
//      //init heuristic
//      f->joint->sampleUniform=opt.sampleRate_stable;
//      f->joint->q0.clear(); // = zeros(3);
//    }
//  } else if(newMode==SY_stableOnX) {
//    Transformation rel = 0;
//    rel.pos.set(.5*(shapeSize(world.getFrame(parent), 0) + shapeSize(world.getFrame(name), 2)), 0., 0.);
//    rel.rot.addY(.5*RAI_PI);
//    rai::Frame* f = addStableFrame(JT_transXYPhi, parent, name, 0, rel);
//    if(f) {
//      //limits?
//      rai::Shape* on = world.getFrame(parent)->shape;
//      CHECK_EQ(on->type(), rai::ST_ssBox, "")
//      f->joint->limits = {
//        -.5*on->size(2), .5*on->size(2),
//          -.5*on->size(1), .5*on->size(1),
//          -RAI_2PI, RAI_2PI
//        };
//      //init heuristic
//      f->joint->sampleUniform=1.;
//      f->joint->q0 = zeros(3);
//    }
//  } else if(newMode==SY_stableOnY) {
//    Transformation rel = 0;
//    rel.pos.set(0., 0, -.5*(shapeSize(world.getFrame(parent), 2) + shapeSize(world.getFrame(name), 1)));
//    rel.rot.addX(.5*RAI_PI);
//    rai::Frame* f = addStableFrame(JT_generic, parent, name, 0, rel);
//    f->joint->setGeneric("xzb");
//    if(f) {
//      //limits?
//      rai::Shape* on = world.getFrame(name)->shape;
//      CHECK_EQ(on->type(), rai::ST_ssBox, "")
//      f->joint->limits = {
//        -.5*on->size(0), .5*on->size(0),
//          -.5*on->size(2), .5*on->size(2),
//          -RAI_2PI, RAI_2PI
//        };
//      //init heuristic
//      f->joint->sampleUniform=1.;
//      f->joint->q0 = zeros(f->joint->dim);
//    }
//  } else NIY;
//}

void KOMO::addRigidSwitch(double time, const StringA& frames, bool noJumpStart) {
  if(noJumpStart) {
    //NOTE: when frames(0) is picking up a kinematic chain (e.g., where frames(1) is a handB of a walker),
    //  then we actually need to impose the no-jump constrained on the root of the kinematic chain!
    //  To prevent special case for the skeleton specifer, we use this ugly code to determine the root of
    //  the kinematic chain for frames(1) -- when frames(1) is a normal object, this should be just frames(1) itself
    rai::Frame* toBePicked = world[frames(1)];
    int s = conv_time2step(time, stepsPerPhase);
    toBePicked = timeSlices(s+k_order, toBePicked->ID);
    rai::Frame* rootOfPicked = toBePicked->getUpwardLink(NoTransformation, true);
    rai::Frame* prev = rootOfPicked->prev;
    if(prev && prev->joint && prev->joint->isStable){
      addObjective({time}, FS_poseRel, {rootOfPicked->name, prev->parent->name}, OT_eq, {1e0}, NoArr, 1);
    }else{
      addObjective({time}, FS_pose, {rootOfPicked->name}, OT_eq, {1e0}, NoArr, 1);
    }
    if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {frames(1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
  }

  addSwitch(time, true, true, JT_rigid, SWInit_zero, frames(0), frames(1));

  //Note: Why impose no-jump at end - why not let the next mode impose continuity?
  //  -> Only the current mode knows frame(0), and we want to impose no relative motion of object (frame(0)) to parent (frame(1))

  //-- no jump at end
//  if(times(1)>=0) {
//    addObjective({times(1)}, FS_poseRel, {frames(1), frames(0)}, OT_eq, {1e1}, NoArr, 1, 0, 0);
//    if(k_order>1) addObjective({times(1)}, make_shared<F_LinAngVel>(), {frames(1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
//  }
}

void KOMO::addModeSwitch(const arr& times, SkeletonSymbol newMode, const StringA& frames, bool firstSwitch) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn
      || newMode==SY_stableYPhi
      || newMode==SY_stableOnX || newMode==SY_stableOnY
      || newMode==SY_stableZero) {

    //---------------- no jump constraints ---

    //-- no jump at start
    if(firstSwitch) {
      //NOTE: when frames(0) is picking up a kinematic chain (e.g., where frames(1) is a handB of a walker),
      //  then we actually need to impose the no-jump constrained on the root of the kinematic chain!
      //  To prevent special case for the skeleton specifer, we use this ugly code to determine the root of
      //  the kinematic chain for frames(1) -- when frames(1) is a normal object, this should be just frames(1) itself
      rai::Frame* toBePicked = world[frames(1)];
      int s = conv_time2step(times(0), stepsPerPhase);
      toBePicked = timeSlices(s+k_order, toBePicked->ID);
      rai::Frame* rootOfPicked = toBePicked->getUpwardLink(NoTransformation, true);

      rai::Frame* prev = rootOfPicked->prev;
      if(prev && prev->joint && prev->joint->isStable){
        addObjective({times(0)}, FS_poseRel, {rootOfPicked->name, prev->parent->name}, OT_eq, {1e1}, NoArr, 1);
      }else{
        addObjective({times(0)}, FS_pose, {rootOfPicked->name}, OT_eq, {1e1}, NoArr, 1);
      }
      if(k_order>1) addObjective({times(0)}, make_shared<F_LinAngVel>(), {frames(1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
    }

    //Note: Why impose no-jump at end - why not let the next mode impose continuity?
    //  -> Only the current mode knows frame(0), and we want to impose no relative motion of object (frame(1)) to parent (frame(0))

    //-- no jump at end
    if(times(1)>=0) {
//      if(stepsPerPhase>3){
//        addObjective({times(1)}, FS_poseRel, {frames(1), frames(0)}, OT_eq, {1e1}, NoArr, 1, 0, +1); //two time slices no velocity -> no acceleration!
//      }else{
        addObjective({times(1)}, FS_poseRel, {frames(1), frames(0)}, OT_eq, {1e1}, NoArr, 1, 0, 0);
//      }
      if(k_order>1) addObjective({times(1)}, make_shared<F_LinAngVel>(), {frames(1)}, OT_eq, {1e0}, NoArr, 2, +1, +1); //no acceleration of the object
    }

    //---------------- create the switch and limits for the effective dof ---
    if(newMode==SY_stable) {
      rai::Frame* f = addSwitch(times(0), true, true, JT_free, SWInit_copy, frames(0), frames(1));
      if(f) { //limits?
        double maxsize = 0.;
        rai::Shape* from = world.getFrame(frames(0))->shape;
        if(from && from->type()!=rai::ST_marker) {
          if(from->type()==rai::ST_sphere || from->type()==rai::ST_cylinder || from->type()==rai::ST_ssCylinder) {
            maxsize += 2.*from->size(0);
          } else {
            maxsize += absMax(from->size);
          }
        } else if(from) {
          CHECK_EQ(from->type(), ST_marker, "");
        }
        rai::Shape* to = world.getFrame(frames(1))->shape;
        if(to && to->type()!=rai::ST_marker) {
          if(to->type()==rai::ST_sphere || to->type()==rai::ST_cylinder || to->type()==rai::ST_ssCylinder) {
            maxsize += 2.*to->size(0);
          } else {
            maxsize += absMax(to->size);
          }
        }
        if(maxsize>1e-4) {
          f->joint->limits = {
            -.9*maxsize, .9*maxsize,
              -.9*maxsize, .9*maxsize,
              -.9*maxsize, .9*maxsize,
//            0., 1.1, -.5,.5, -.5,.5, -.5,.5 }; //no limits on rotation
              -1.1, 1.1, -1.1, 1.1, -1.1, 1.1, -1.1, 1.1
            }; //no limits on rotation
        }
//        f->joint->q0.clear(); // = zeros(7); f->joint->q0(3)=1.; //.clear();
      }
      //sample heuristic
      f->joint->sampleUniform=opt.sampleRate_stable;
      f->joint->q0 = f->joint->calcDofsFromConfig();
    } else if(newMode==SY_stableZero) {
      addSwitch(times(0), true, true, JT_rigid, SWInit_zero, frames(0), frames(1));
    } else if(newMode==SY_stableOn) {
      Transformation rel = 0;
      //relTransformOn(world, frames(0), frames(1));
      rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(frames(0))) + shapeSize(world.getFrame(frames(1)))));
      rai::Frame* f = addSwitch(times(0), true, true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
      //f->joint->setGeneric("xyc");
      if(f) {
        //limits?
        rai::Shape* on = world.getFrame(frames(0))->shape;
        CHECK_EQ(on->type(), rai::ST_ssBox, "")
        f->joint->limits = {
          -.5*on->size(0), .5*on->size(0),
            -.5*on->size(1), .5*on->size(1),
            -RAI_2PI, RAI_2PI
          };
        //init heuristic
        f->joint->sampleUniform=1.;
        f->joint->q0 = zeros(3);
      }
    } else if(newMode==SY_stableOnX) {
      Transformation rel = 0;
      rel.pos.set(.5*(shapeSize(world.getFrame(frames(0)), 0) + shapeSize(world.getFrame(frames(1)), 2)), 0., 0.);
      rel.rot.addY(.5*RAI_PI);
      rai::Frame* f = addSwitch(times(0), true, true, JT_transXYPhi, SWInit_zero, frames(0), frames(1), rel);
      if(f) {
        //limits?
        rai::Shape* on = world.getFrame(frames(0))->shape;
        CHECK_EQ(on->type(), rai::ST_ssBox, "")
        f->joint->limits = {
          -.5*on->size(2), .5*on->size(2),
            -.5*on->size(1), .5*on->size(1),
            -RAI_2PI, RAI_2PI
          };
        //init heuristic
        f->joint->sampleUniform=1.;
        f->joint->q0 = zeros(3);
      }
    } else if(newMode==SY_stableOnY) {
      Transformation rel = 0;
      rel.pos.set(0., 0, -.5*(shapeSize(world.getFrame(frames(0)), 2) + shapeSize(world.getFrame(frames(1)), 1)));
      rel.rot.addX(.5*RAI_PI);
      rai::Frame* f = addSwitch(times(0), true, true, JT_generic, SWInit_zero, frames(0), frames(1), rel);
      f->joint->setGeneric("xzb");
      if(f) {
        //limits?
        rai::Shape* on = world.getFrame(frames(1))->shape;
        CHECK_EQ(on->type(), rai::ST_ssBox, "")
        f->joint->limits = {
          -.5*on->size(0), .5*on->size(0),
            -.5*on->size(2), .5*on->size(2),
            -RAI_2PI, RAI_2PI
          };
        //init heuristic
        f->joint->sampleUniform=1.;
        f->joint->q0 = zeros(f->joint->dim);
      }
    } else if(newMode==SY_stableYPhi) {
      Transformation rel = 0;
//      rel.pos.set(0, 0, -.5*(shapeSize(world.getFrame(frames(0))) + shapeSize(world.getFrame(frames(1)))));
      addSwitch(times(0), true, true, JT_transY, SWInit_copy, frames(0), frames(1), rel);
    } else NIY;

    if(!opt.mimicStable && newMode!=SY_stableZero) {
      // ensure the DOF is constant throughout its existance
      if((times(1)<0. && stepsPerPhase*times(0)<T) || stepsPerPhase*times(1)>stepsPerPhase*times(0)+1) {
        addObjective({times(0), times(1)}, make_shared<F_qZeroVel>(), {frames(1)}, OT_eq, {1e1}, NoArr, 1, +1, -1);
      }
    }

  } else if(newMode==SY_dynamic) {
    if(frames.N==1) {
      addSwitch(times(0), true, false, JT_free, SWInit_copy, world.frames.first()->name, frames(-1));
    } else {
      addSwitch(times(0), true, false, JT_free, SWInit_copy, frames(0), frames(1));
    }
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    if(firstSwitch) {
      if(stepsPerPhase>3) {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      } else {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
    if(k_order>1) {
      //... and physics starting from [-1,0,+1], ... until [-2,-1,-0]
      addObjective(times, make_shared<F_NewtonEuler>(), {frames(1)}, OT_eq, {1e2}, NoArr, 2, +1, 0);
    }
  } else if(newMode==SY_dynamicOn) {
    CHECK_EQ(frames.N, 2, "");
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(frames(0))) + shapeSize(world.getFrame(frames(1)))));

    addSwitch(times(0), true, false, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    if(firstSwitch) {
      if(stepsPerPhase>3) {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      } else {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }

    if(k_order>1) {
      addObjective(times, make_shared<F_NewtonEuler>(false), {frames(1)}, OT_eq, {1e2}, NoArr, 2, +1, 0);
    }
  } else if(newMode==SY_quasiStaticOn) {
    CHECK_GE(frames.N, 2, "");
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(frames(0))) + shapeSize(world.getFrame(frames(1)))));
//    addSwitch(times(0), true, JT_transXYPhi, SWInit_copy, frames(0), frames(1), rel);
    rai::Frame* f = addSwitch(times(0), true, make_shared<KinematicSwitch>(SW_joint, JT_transXYPhi, frames(0), frames(1), world, SWInit_copy, 0, rel));
    if(f) {
      //limits?
      rai::Shape* on = world.getFrame(frames(0))->shape;
      if(on->type()==rai::ST_ssBox) {
        f->joint->limits = {
          -.5*on->size(0), .5*on->size(0),
            -.5*on->size(1), .5*on->size(1),
            -RAI_2PI, RAI_2PI
          };
        //init heuristic
        f->joint->sampleUniform=1.;
        f->joint->q0 = zeros(3);
        rai::Frame* p=f->prev;
        while(p && p->joint && p->joint->type==JT_transXYPhi) {
          p->joint->limits = f->joint->limits;
          p = p->prev;
        }
      }
    }

    //-- no jump at start
    if(firstSwitch) {
      if(stepsPerPhase>3) {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, +1); //overlaps with Newton-Euler -> requires forces!
      } else {
        addObjective({times(0)}, FS_pose, {frames(1)}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
#if 0
    addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(0., false), {frames(1)}, OT_eq, {1e2}, NoArr, 1, +1, 0);
#else
    //eq for 3DOFs only
    shared_ptr<Objective> o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(1)}, OT_eq, {}, NoArr, 1, +1, 0);
    o->feat->scale=1e1 * arr({3, 6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective(times, make_shared<F_NewtonEuler_DampedVelocities>(false), {frames(1)}, OT_sos, {}, NoArr, 1, +1, 0);
    o->feat->scale=1e0 * arr({3, 6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
  } else {
    LOG(-1) <<"newMode=" <<newMode;
    NIY;
  }
}

void KOMO::addContact_slide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else //old, based on PairCollision
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e1});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e2});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
//  addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_sos, {1e0}, NoArr, 1, +1, +0);
}

void KOMO::addContact_stick(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //-- on-surface (distance) constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1}); //left-poa dist zero
  addObjective({startTime, endTime}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1}); //right-poa dist zero
#else
  addObjective({startTime, endTime}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1}); //left-right dist zero

  //-- force constraints
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  if(k_order>0) {
    addObjective({startTime, endTime}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_eq, {1e0}, NoArr, 1, +1, 0);
  } else {
    addObjective({startTime, endTime}, make_shared<F_fex_ForceInFrictionCone>(.8), {from, to}, OT_ineq, {1e0});
  }

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  if(k_order>0){
    addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
    addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, k_order, +2, +0);
  }
}

void KOMO::addContact_ComplementarySlide(double startTime, double endTime, const char* from, const char* to) {
  addSwitch({startTime}, true, make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  if(endTime>0.) addSwitch(endTime, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<F_fex_ForceIsComplementary>(), {from, to}, OT_eq, {1e2});
  addObjective({startTime, endTime}, make_shared<F_fex_NormalVelIsComplementary>(0., 0.), {from, to}, OT_eq, {1e2}, NoArr, 1, +1);
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {from, to}, OT_ineq, {1e1});

  //regularization
  addObjective({startTime, endTime}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 2, +3, +0);
  addObjective({startTime, endTime}, make_shared<F_fex_POA>(), {from, to}, OT_sos, {1e-2}, NoArr, 1, +1, +0);
//  addObjective({startTime, endTime}, make_shared<TM_Contact_POAzeroRelVel>(world, from, to), OT_sos, {1e-1}, NoArr, 1, +1, +0);
}

void KOMO::addContact_elasticBounce(double time, const char* from, const char* to, double elasticity, double stickiness) {
  addSwitch(time, true,  make_shared<rai::KinematicSwitch>(rai::SW_addContact, rai::JT_none, from, to, world));
  addSwitch(time, false, make_shared<rai::KinematicSwitch>(rai::SW_delContact, rai::JT_none, from, to, world));

  //constraints
#ifdef RAI_USE_FUNCTIONALS //new, based on functionals
  addObjective({time}, make_shared<F_fex_POASurfaceDistance>(rai::_left), {from, to}, OT_eq, {1e1});
  addObjective({time}, make_shared<F_fex_POASurfaceDistance>(rai::_right), {from, to}, OT_eq, {1e1});
#else
  addObjective({time}, make_shared<F_fex_POAContactDistances>(), {from, to}, OT_ineq, {1e1});
#endif
  addObjective({time}, FS_pairCollision_negScalar, {from, to}, OT_eq, {1e1});
  if(stickiness<=0.) addObjective({time}, make_shared<F_fex_ForceIsNormal>(), {from, to}, OT_eq, {1e1});
  addObjective({time}, make_shared<F_fex_ForceIsPositive>(), {from, to}, OT_ineq, {1e1});
  if(!elasticity && stickiness>=1.) {
    addObjective({time}, make_shared<F_fex_POAzeroRelVel>(), {from, to}, OT_eq, {1e1}, NoArr, 2, +1, +1);
  } else {
    addObjective({time}, make_shared<F_fex_ElasticVel>(elasticity, stickiness), {from, to}, OT_eq, {1e1}, NoArr, 2, +1, +1);
  }

  //regularization
  addObjective({time}, make_shared<F_fex_Force>(), {from, to}, OT_sos, {1e-4});
}

shared_ptr<Objective> KOMO::addControlObjective(const arr& times, uint order, double scale, const arr& target, int deltaFromStep, int deltaToStep) {
  auto F = getCtrlFramesAndScale(world);
  //F.scale *= sqrt(tau); NO!! The Feature::finiteDifference does this automatically, depending on the timeIntegral flag!

  CHECK_GE(k_order, order, "");
  shared_ptr<Objective> o = addObjective(times, make_shared<F_qItself>(F.frames, (order==0)), {}, OT_sos, scale*F.scale, target, order, deltaFromStep, deltaToStep);
  o->feat->timeIntegral=1;
  return o;
}

void KOMO::addQuaternionNorms(const arr& times, double scale, bool hard) {
  addObjective(times, make_shared<F_qQuaternionNorms>(), {"ALL"}, (hard?OT_eq:OT_sos), {scale}, NoArr);
}

void KOMO::setSlow(double startTime, double endTime, double prec, bool hardConstrained) {
  if(stepsPerPhase>2) { //otherwise: no velocities
    uintA selectedBodies;
    for(rai::Frame* f:world.frames) if(f->joint && f->joint->dim>0 && f->joint->dim<7 && f->joint->type!=rai::JT_tau && f->joint->active && f->joint->H>0.) {
        selectedBodies.append(uintA{f->ID, f->parent->ID});
      }
    selectedBodies.reshape(selectedBodies.N/2, 2);
    shared_ptr<Feature> feat = make_shared<F_qItself>(selectedBodies);
    if(!hardConstrained) addObjective({startTime, endTime}, feat, {}, OT_sos, {prec}, NoArr, 1);
    else addObjective({startTime, endTime}, feat, {}, OT_eq, {prec}, NoArr, 1);
  }
  //#    _MinSumOfSqr_qItself_vel(MinSumOfSqr qItself){ order=1 time=[0.98 1] scale=3e0 } #slow down
}

void KOMO::setSlowAround(double time, double delta, double prec, bool hardConstrained) {
  setSlow(time-delta, time+delta, prec, hardConstrained);
}

void KOMO::add_collision(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin, true, false), {"ALL"}, OT_eq, {prec}, NoArr);
  } else { //cost term
    addObjective({}, make_shared<F_AccumulatedCollisions>(margin, true, false), {"ALL"}, OT_sos, {prec}, NoArr);
  }
}

void KOMO::add_jointLimits(bool hardConstraint, double margin, double prec) {
  if(hardConstraint) { //interpreted as hard constraint (default)
    addObjective({}, make_shared<F_qLimits>(), {"ALL"}, OT_ineq, {prec}, {-margin});
  } else { //cost term
    NIY;
  }
}

void KOMO::setLiftDownUp(double time, const char* endeff, double timeToLift) {
  if(stepsPerPhase>2 && timeToLift>0.) { //velocities down and up
    addObjective({time-timeToLift, time-.5*timeToLift}, make_shared<F_Position>(), {endeff}, OT_sos, {1e0}, {0., 0., -.2}, 1); //move down
//    addObjective({time-timeToLift/3,  time+timeToLift/3}, make_shared<TM_Default>(TMT_pos, world, endeff), OT_sos, {3e0}, {0.,0.,0.}, 1); //move down
    addObjective({time+.5*timeToLift, time+timeToLift}, make_shared<F_Position>(), {endeff}, OT_sos, {1e0}, {0., 0., .2}, 1); // move up
  }
}

//===========================================================================
//
// config
//

void KOMO::setIKOpt() {
  opt.sparse = false;
  setTiming(1., 1, 1., 1);
  addControlObjective({}, 1, 1e-1);
  addQuaternionNorms();
}

void KOMO::setConfiguration_qAll(int t, const arr& q) {
  pathConfig.setDofState(q, pathConfig.getDofs(timeSlices[k_order+t], true, false));
}

arr KOMO::getConfiguration_qAll(int t) {
  return pathConfig.getDofState(pathConfig.getDofs(timeSlices[k_order+t], true, false));
}

arr KOMO::getConfiguration_qOrg(int t) {
  return pathConfig.getDofState(pathConfig.getDofs(pathConfig.getFrames(orgJointIndices + timeSlices(k_order+t, 0)->ID), true, true)); //also inactive ones, as the orgJointIndices are explicit
}

void KOMO::setConfiguration_qOrg(int t, const arr& q) {
  pathConfig.setDofState(q, pathConfig.getDofs(pathConfig.getFrames(orgJointIndices + timeSlices(k_order+t, 0)->ID), true, true)); //also inactive ones, as the orgJointIndices are explicit
}

arr KOMO::getConfiguration_dofs(uint t, const uintA& dofIndices) {
  return pathConfig.getDofState(pathConfig.getDofs(pathConfig.getFrames(dofIndices + timeSlices(k_order+t, 0)->ID), true, true)); //also inactive ones, as the orgJointIndices are explicit
}

void KOMO::setConfiguration_X(int t, const arr& X) {
  pathConfig.setFrameState(X, timeSlices[k_order+t]);
}

void KOMO::initOrg() {
  arr X = world.getFrameState();
  for(uint t=0; t<T; t++) {
    pathConfig.setFrameState(X, timeSlices[k_order+t]);
  }
  for(Dof* d:pathConfig.activeDofs) {
    if(d->fex()) d->setDofs(zeros(d->dim));
  }
}

void KOMO::initRandom(int verbose) {
  pathConfig.setRandom(timeSlices.d1, verbose); //opt.verbose);
  x = pathConfig.getJointState();
}

arr KOMO::getConfiguration_X(int t) {
  return pathConfig.getFrameState(timeSlices[k_order+t]);
}

void KOMO::getConfiguration_full(Configuration& C, int t, int verbose) {
#if 1
  C.clear();
  FrameL F = timeSlices[k_order+t].copy();
  for(uint i=0; i<F.N; i++) {
    rai::Frame* f = F(i);
    f->ensure_X();
    if(f->parent && !F.contains(f->parent)) F.append(f->parent); //note: this is recursive, as appending to looing over F itself!
  }
  //pathConfig.checkConsistency();
  C.addCopy(F, {}); //, pathConfig.getDofs(F, false));
  C.frames.reshape(-1);
  //C.checkConsistency();
#else
  //note: the alternative would be to copy the frames komo.timeSlices[step] into a new config
  CHECK_EQ(k_order, 1, "");
  C.copy(world);
  for(std::shared_ptr<rai::KinematicSwitch>& sw:switches) {
    int s = sw->timeOfApplication;
    if(s<=t) {
      if(verbose) { LOG(0) <<"applying switch:"; sw->write(cout, C.frames); cout <<endl; }
      sw->apply(C.frames);
    }
  }
  arr X = getConfiguration_X(t);
  C.setFrameState(X, C.frames({0, X.d0-1}));
#endif
}

arr KOMO::getPath_qOrg() {
  arr q = getConfiguration_qOrg(0);
  q.resizeCopy(T, q.N);
  for(uint t=1; t<T; t++) {
    q[t] = getConfiguration_qOrg(t);
  }
  return q;
}

arr KOMO::getPath_X() {
  arr X(T, timeSlices.d1, 7);
  for(uint t=0; t<T; t++) {
    X[t] = getConfiguration_X(t);
  }
  return X;
}

arrA KOMO::getPath_qAll() {
  arrA q(T);
  for(uint t=0; t<T; t++) q(t) = getConfiguration_qAll(t);
  return q;
}

arr KOMO::getPath_tau() {
  arr X(T);
  for(uint t=0; t<T; t++) {
    pathConfig.kinematicsTau(X(t), NoArr, timeSlices(t+k_order, 0));
  }
  return X;
}

arr KOMO::getPath_times() {
  arr tau = getPath_tau();
  return integral(tau);
}

arr KOMO::getPath_energies() {
  F_Energy E;
  E.order=1;
  arr X(T), y;
  for(uint t=0; t<T; t++) {
    NIY//    E.phi2(y, NoArr, cat(configurations(t+k_order-1)->frames, configurations(t+k_order)->frames).reshape(2,-1));
    X(t) = y.scalar();
  }
  return X;
}

void KOMO::initWithConstant(const arr& q) {
  for(uint t=0; t<T; t++) {
    setConfiguration_qOrg(t, q);
  }
  run_prepare(0.);
}

void setQByPairs(rai::Configuration& C, const FrameL& F, arr q) {
  uint m=0;
  FrameL sel;
  for(uint i=0; i<F.N; i+=2) {
    rai::Frame* a = F.elem(i+0);
    rai::Frame* b = F.elem(i+1);
    if(a->parent==b) { sel.append(a); m += a->joint->dim; }
    else if(b->parent==a) { sel.append(b); for(uint k=0; k<b->joint->dim; k++) q.elem(m++) *= -1; }
    else HALT("a and b are not linked");
  }
  CHECK_EQ(m, q.N, "");
  C.setJointState(q, sel);
}

void getDofsAndSignFromFramePairs(DofL& dofs, arr& signs, const FrameL& F) {
  signs.clear();
  dofs.resize(F.N/2);
  for(uint i=0; i<F.N/2; i++) {
    rai::Frame* a = F.elem(2*i+0);
    rai::Frame* b = F.elem(2*i+1);
    if(a->parent==b) { dofs(i)=a->joint; signs.append(ones(a->joint->dim)); }
    else if(b->parent==a) { dofs(i)=b->joint; signs.append(-ones(b->joint->dim)); }
    else HALT("a and b are not linked");
  }
}

void KOMO::initPhaseWithDofsPath(uint t_phase, const uintA& dofIDs, const arr& path_org, bool autoResamplePath) {
  arr path;
  if(autoResamplePath && path_org.d0!=stepsPerPhase) {
    path = path_resampleLinear(path_org, stepsPerPhase);
  } else {
    path.referTo(path_org);
  }
  CHECK_EQ(path.d0, stepsPerPhase, "given path is of wrong length");
  for(uint tt=0; tt<path.d0-1; tt++) {
    pathConfig.setJointState(path[tt], dofIDs + (k_order+stepsPerPhase*t_phase+tt)*timeSlices.d1);
  }
}

uintA KOMO::initWithWaypoints(const arrA& waypoints, uint waypointStepsPerPhase, bool interpolate, double qHomeInterpolate, int verbose) {
  //-- compute in which steps (configuration time slices) the waypoints are imposed
  uintA steps(waypoints.N);
  for(uint i=0; i<steps.N; i++) {
    steps(i) = conv_time2step(conv_step2time(i, waypointStepsPerPhase), stepsPerPhase);
  }

  if(verbose>0) {
    view(true, STRING("initWithWaypoints - before"));
  }

  //-- set the path piece-wise CONSTANT at waypoints and the subsequent steps (each waypoint may have different dimension!...)
  if(!opt.mimicStable) { //depends on sw->isStable -> mimic !!
    for(uint i=0; i<steps.N; i++) {
      uint Tstop=T;
      if(i+1<steps.N && steps(i+1)<T) Tstop=steps(i+1);
      for(uint t=steps(i); t<Tstop; t++) {
        if(waypoints(i).nd==1) {
          try {
            setConfiguration_qAll(t, waypoints(i));
          } catch(...) {}
        } else {
          setConfiguration_X(t, waypoints(i));
        }
      }
    }
  } else {
    for(uint i=0; i<steps.N; i++) {
      if(steps(i)<T) setConfiguration_qAll(steps(i), waypoints(i));
    }
    //set intermediate constant
    for(uint i=0; i<steps.N; i++) {
      uint Tstart = 1;
      if(i) Tstart = steps(i-1)+1;
      for(uint t=Tstart; t<steps(i); t++) {
        for(rai::Frame* f:timeSlices[k_order+t]) {
          if(f->joint && f->joint->active && !f->joint->mimic && f->prev) {
            f->C.setDofState(f->prev->joint->getDofState(), {f->joint});
          }
        }
      }
    }
  }

  if(verbose>0) {
    view(true, STRING("initWithWaypoints - after keyframes->constant"));
  }

  //-- interpolate w.r.t. non-switching frames within the intervals
  if(interpolate) {
#if 1
    auto F = getCtrlFramesAndScale(world);
    arr qHome = world.getDofHomeState(world.activeDofs);
    //F.frames.reshape(1,-1,2); F_qItself qfeat;
    arr signs;
    DofL dofs;
    for(uint i=0; i<steps.N; i++) {
      uint t0=0; if(i) t0 = steps(i-1);
      uint t1=steps(i);

      //interpolate the controlled DOFs, using qItself to also use the pair-wise joint indexing for flipped joints (walker case)
#if 0
      if(t1-1<T) {
        uintA nonSwitched = getNonSwitchedFrames(timeSlices[k_order+t0], timeSlices[k_order+t1]);
        arr q0 = pathConfig.getJointState(timeSlices[k_order+t0].sub(nonSwitched));
        arr q1 = pathConfig.getJointState(timeSlices[k_order+t1].sub(nonSwitched));
        for(uint t=t0+1; t<t1; t++) {
          double phase = double(t-t0)/double(t1-t0);
          arr q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0); //p = p0 + phase * (p1-p0);
          pathConfig.setJointState(q, timeSlices[k_order+t].sub(nonSwitched));
          //view(true, STRING("interpolating: step:" <<i <<" t: " <<j));
        }
      }
#else
      if(t1-1<T) {
        getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t0, 0)->ID));
        arr q0 = signs%pathConfig.getDofState(dofs);
        getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t1, 0)->ID));
        arr q1 = signs%pathConfig.getDofState(dofs);
        //TODO: check if all dofs are rotational joints!
        makeMod2Pi(q0, q1);
        pathConfig.setDofState(signs%q1, dofs);
        //      arr q0 = qfeat.eval(pathConfig.getFrames(F.frames + timeSlices(k_order+t0,0)->ID));  q0.J_reset();
        //      arr q1 = qfeat.eval(pathConfig.getFrames(F.frames + timeSlices(k_order+t1,0)->ID));  q1.J_reset();
        for(uint t=t0+1; t<t1; t++) {
          double phase = double(t-t0)/double(t1-t0);
          arr q = q0 + (.5*(1.-cos(RAI_PI*phase))) * (q1-q0); //p = p0 + phase * (p1-p0);
          if(qHomeInterpolate>0.) { //also interpolate to qHome
            q += qHomeInterpolate*(.5*(1.-cos(RAI_2PI*phase)))*(qHome-q);
          }
          getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t, 0)->ID));
          pathConfig.setDofState(signs%q, dofs);
          //        setQByPairs(pathConfig, pathConfig.getFrames(F.frames + timeSlices(k_order+t,0)->ID), q);
          //view(true, STRING("interpolating: step:" <<i <<" t: " <<t));
        }
      }
#endif

      //motion profile for switched object positions
#if 0 //that doesn't work for walking!!
      if(t1-1<T) {
        uintA switched = getSwitchedFrames(timeSlices[k_order+t0], timeSlices[k_order+t1]);
        for(uint k:switched) {
          if(timeSlices(k_order+t0, k)->joint && timeSlices(k_order+t0, k)->joint->isStable) continue; //don't interpolate stable joints
          arr p0 = timeSlices(k_order+t0, k)->getPosition();
          arr p1 = timeSlices(k_order+t1, k)->getPosition();
          for(uint t=t0+1; t<t1; t++) {
            double phase = double(t-t0)/double(t1-t0);
            arr p = p0 + (.5*(1.-cos(RAI_PI*phase))) * (p1-p0); //p = p0 + phase * (p1-p0);
            timeSlices(k_order+t, k)->setPosition(p);
            //view(true, STRING("interpolating: step:" <<i <<" t: " <<j));
          }
        }
      }
#endif
    }
#endif

    if(verbose>0) {
      view(true, STRING("initWithWaypoints - done"));
    }
  }

  run_prepare(0.);

  return steps;
}

void KOMO::initWithPath_qOrg(const arr& q) {
  CHECK_EQ(q.d0, T, "");
  for(uint t=0; t<T; t++) {
    setConfiguration_qOrg(t, q[t]);
  }
}

void KOMO::straightenCtrlFrames_mod2Pi() {
  auto F = getCtrlFramesAndScale(world);
  arr signs;
  DofL dofs;
  for(uint t=0; t<T-1; t++) {
    getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t, 0)->ID));
    arr q0 = signs%pathConfig.getDofState(dofs);
    getDofsAndSignFromFramePairs(dofs, signs, pathConfig.getFrames(F.frames + timeSlices(k_order+t+1, 0)->ID));
    arr q1 = signs%pathConfig.getDofState(dofs);
    //TODO: check if all dofs are rotational joints!
    makeMod2Pi(q0, q1);
    pathConfig.setDofState(signs%q1, dofs);
  }
}

void KOMO::addWaypointsInterpolationObjectives(const arrA& waypoints, uint waypointStepsPerPhase) {

  uintA steps = initWithWaypoints(waypoints, waypointStepsPerPhase, false);

  for(uint k=0; k<steps.N; k++) {
    uint t0=0; if(k) t0 = steps(k-1);
    uint t1=steps(k);

    for(uint i=0; i<timeSlices.d1; i++) {
      rai::Transformation A = timeSlices(k_order+t0, i)->ensure_X();
      rai::Transformation B = timeSlices(k_order+t1, i)->ensure_X();
      rai::Transformation X;
      if(A==B) {
      } else {
        for(uint t=t0; t<=t1; t++) {
          double phase = double(t-t0)/double(t1-t0);
          X.setInterpolate(phase, A, B);
          {
            std::shared_ptr<Feature> feat = make_shared<F_Position>();
            feat->setFrameIDs({i}) .setTarget(X.pos.getArr());
            shared_ptr<struct Objective>  tmp = addObjective({0.}, feat, {}, OT_sos, NoArr, NoArr, -1, t, t);
          }
          {
            std::shared_ptr<Feature> feat = make_shared<F_Quaternion>();
            feat->setFrameIDs({i}) .setTarget(X.rot.getArr4d());
            shared_ptr<struct Objective>  tmp = addObjective({0.}, feat, {}, OT_sos, NoArr, NoArr, -1, t, t);
          }
        }
      }
    }
  }
}

void KOMO::updateRootObjects(const Configuration& C) {
  //-- frame state of roots only, if objects moved:
  FrameL _roots = C.getRoots();
  {
    //also add rigid children of roots
    FrameL F;
    for(auto f:_roots) f->getRigidSubFrames(F, true);
    _roots.append(F);
  }
  uintA roots = framesToIndices(_roots);
  arr X0 = C.getFrameState(roots);
  world.setFrameState(X0, roots);
  //set t=0..T to new frame state:
  for(uint t=0; t<T; t++) pathConfig.setFrameState(X0, roots+timeSlices(k_order+t, 0)->ID);
  //shift the frame states within the prefix (t=-1 becomes equal to t=0, which is new state)
  for(int t=-k_order; t<0; t++) {
    arr Xt = pathConfig.getFrameState(roots+timeSlices(k_order+t+1, 0)->ID);
    pathConfig.setFrameState(Xt, roots+timeSlices(k_order+t, 0)->ID);
  }
}

void KOMO::updateAndShiftPrefix(const Configuration& C) {
  //-- joint state
  //set t=0 to new joint state:
  setConfiguration_qOrg(0, C.getJointState());
  //shift the joint state within prefix (t=-1 becomes equal to t=0, which is new state)
  for(int t=-k_order; t<0; t++) setConfiguration_qOrg(t, getConfiguration_qOrg(t+1));

  updateRootObjects(C);
}

void KOMO::reset() {
  dual.clear();
  featureValues.clear();
  featureJacobians.clear();
  featureTypes.clear();
  timeTotal=timeCollisions=timeKinematics=timeNewton=timeFeatures=0.;
}

std::shared_ptr<SolverReturn> KOMO::optimize(double addInitializationNoise, int splineKnots, const rai::OptOptions options) {
  run_prepare(addInitializationNoise);

  if(opt.verbose>1) cout <<"===KOMO::optimize===\n" <<report(true, false, false) <<endl; //reportProblem();

#if 0
  run(options);
#else
  NLP_Solver sol;
  if(splineKnots<=0){
    sol.setProblem(nlp());
    sol.setInitialization(x);
  }else{
    sol.setProblem(nlp_spline(splineKnots));
  }
  sol.setOptions(options);
  sol.opt.set_verbose(rai::MAX(opt.verbose-2, 0));

  timeTotal -= rai::cpuTime();
  std::shared_ptr<SolverReturn> ret = sol.solve();
  timeTotal += rai::cpuTime();

  if(opt.verbose>0) {
    cout <<"=== KOMO optimization time:" <<timeTotal
         <<" (kin:" <<timeKinematics <<" coll:" <<timeCollisions <<" feat:" <<timeFeatures <<" newton: " <<timeNewton <<")"
         <<" setJointStateCount:" <<Configuration::setJointStateCount
         <<"\n  sos:" <<ret->sos <<" ineq:" <<ret->ineq <<" eq:" <<ret->eq
         <<"\n  solver return: " <<*ret <<endl;
  }
  if(opt.verbose>1) cout <<report(false, opt.verbose>2) <<endl;
  return ret;
#endif
}

void KOMO::run_prepare(double addInitializationNoise) {
  //ensure the configurations are setup
  CHECK(timeSlices.N, "path config is not setup");
  //if(!timeSlices.nd) setupPathConfig();
//  if(!switchesWereApplied) retrospectApplySwitches(); //should be applied immediately!

  //ensure the decision variable is in sync from the configurations
  x = pathConfig.getJointState();

  //add noise
  if(addInitializationNoise>0.) {
    rndGauss(x, addInitializationNoise, true);

    {
      arr bounds = getBounds();
      boundClip(x, bounds);
    }

    pathConfig.setJointState(x);
  }
}

Graph KOMO::report(bool specs, bool listObjectives, bool plotOverTime) {
  Graph G;
  if(specs) {
    Graph& g = G.addSubgraph("specs");
    g.add("x_dim", x.N);
    g.add("dual_dim", dual.N);
    g.add("T", T);
    g.add("k", k_order);
    g.add("phases", double(T)/stepsPerPhase);
    g.add("slicesPerPhase", stepsPerPhase);
    g.add("enableCollisions", computeCollisions);
    g.add("tau", tau);
    g.add("#slices", timeSlices.d0);
    g.add("#totalDOF", pathConfig.getJointStateDimension());
    g.add("#frames", pathConfig.frames.N);
    g.add("#pathQueries", pathConfig.setJointStateCount);
  }

  arr err = info_objectiveErrorTraces();
  arr errTotals = sum(err, 0);

  {
    Graph* g_ob = &G;
//    if(specs) g_ob = &G.addSubgraph("objectives");
    uint cId=0;
    for(shared_ptr<Objective>& c:objectives) {
      if(c->groundings.N && !c->groundings.elem(-1)->active) continue;
      Graph* g=0;
      if(listObjectives) g = &g_ob->addSubgraph(STRING('o' <<cId++));
      if(g) {
        g->add<String>("name", c->feat->typeString());
        uintA frameIDs = c->feat->frameIDs;
        if(frameIDs.N<=3) {
          g->add<StringA>("frames", framesToNames(world.getFrames(frameIDs)));
        } else {
          g->add<StringA>("frames", {STRING("#" <<frameIDs.N)});
        }
      }

      if(featureValues.N){
        if(g) g->add<double>("err", errTotals(cId-1));
      }

      if(g) {
        g->add<double>("order", c->feat->order);
        g->add<String>("type", Enum<ObjectiveType>(c->type).name());
        if(c->feat->scale.N) g->add<arr>("scale", c->feat->scale);
        if(c->feat->target.N) g->add<arr>("target", c->feat->target);
        g->add<arr>("times", c->times);
        if(c->times.N && c->times.elem(0)==-10.) {
          g->add<arr>("slices", c->times);
        } else {
          int fromStep, toStep;
          conv_times2steps(fromStep, toStep, c->times, stepsPerPhase, T, +0, +0);
          g->add<arr>("slices", {(double)fromStep, (double)toStep});
        }
      }
    }
  }

  bool sortByError=true;
  if(featureValues.N && listObjectives && sortByError){
    std::sort(G.p, G.p+G.N, [](Node* a, Node *b){
      double* A = a->as<Graph>().find<double>("err");
      double* B = b->as<Graph>().find<double>("err");
      if(!A || !B) return true;
      return *A < *B;
    });
    G.index();
  }

  //  for(std::shared_ptr<KinematicSwitch>& sw:switches) {
//    os <<"    ";
//    if(sw->timeOfApplication+k_order >= timeSlices.d0) {
////      LOG(-1) <<"switch time " <<sw->timeOfApplication <<" is beyond time horizon " <<T;
//      sw->write(os, {});
//    } else {
//      sw->write(os, timeSlices[sw->timeOfApplication+k_order]);
//    }
//    os <<endl;
//  }

  if(featureValues.N) {
    Graph& g = G.addSubgraph("totals");
    arr totals = zeros(OT_ineqP+1);
    CHECK_EQ(objectives.N, errTotals.N, "");
    for(uint i=0;i<errTotals.N;i++){
      totals(objectives(i)->type) += errTotals(i);
    }
    g.add<double>("sos", totals(OT_sos));
    g.add<double>("ineq", totals(OT_ineq));
    g.add<double>("eq", totals(OT_eq));
    g.add<double>("f", totals(OT_f));
  }

  if(plotOverTime) {
    //-- write a gnuplot file
    ofstream fil("z.komoData");
    //first line: legend
    for(auto c:objectives) fil <<c->name <<' ';
    fil <<endl;
    //rest: just the matrix
    err.write(fil, nullptr, nullptr, "  ");
    fil.close();

    ofstream fil2("z.komoReport.plt");
    fil2 <<"set key autotitle columnheader" <<endl;
    fil2 <<"set title 'komo report'" <<endl;
    fil2 <<"plot 'z.komoData' \\" <<endl;
    for(uint i=1; i<=objectives.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u (($0+1)/" <<stepsPerPhase <<"):"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
    fil2 <<endl;
    fil2.close();

    gnuplot("load 'z.komoReport.plt'");
  }

  return G;
}

arr KOMO::info_objectiveErrorTraces(){
//  CHECK(featureValues.N, "KOMO hasn't been queried yet");
  arr err = zeros(T, objectives.N);
  if(!featureValues.N) return err;
  uint M=0;
  for(shared_ptr<Objective>& c:objectives) {
    for(GroundedObjective* ob: c->groundings) {
      uint d = ob->feat->dim(ob->frames);
      int i = ob->objId;
      uint t = ob->timeSlices.last();
      CHECK_GE(i, 0, "");
      for(uint j=0; j<d; j++) {
        double e=0.;
        if(ob->type==OT_sos) e = sqr(featureValues(M+j));
        else if(ob->type==OT_ineq) e = MAX(0., featureValues(M+j));
        else if(ob->type==OT_eq) e = fabs(featureValues(M+j));
        else if(ob->type==OT_f) e = featureValues(M+j);
        else if(ob->type==OT_ineqB) e = MAX(0., featureValues(M+j));
        else NIY;
        err(t, i) += e;
      }
      M += d;
    }
  }
  CHECK_EQ(M, featureValues.N, "");
  return err;
}

StringA KOMO::info_objectiveNames(){
  StringA objectiveNames(objectives.N);
  for(uint i=0;i<objectives.N;i++){
    objectiveNames(i) = objectives(i)->name;
  }
  return objectiveNames;
}

str KOMO::info_sliceErrors(uint t, const arr& errorTraces){
  arr err = errorTraces[t];
  uintA rank;
  rank.setStraightPerm(err.N);
  std::sort(rank.p, rank.p+rank.N, [&](const uint& a, const uint& b){ return err(a)>err(b); });
  str txt;
  for(uint i:rank){
    if(err(i)<1e-4) break;
//    if(objectives(i)->type>=OT_ineq){
      txt <<"t:" <<t <<" #" <<i <<objectives(i)->name <<' ' <<rai::Enum<ObjectiveType>(objectives(i)->type) <<' ' <<err(i) <<endl;
//    }
  }
  return txt;
}

str KOMO::info_sliceCollisions(uint t, double belowMargin){
  //similar to Configuration::getTotalPenetration
  uint nFrames = world.frames.N;
  CHECK_EQ(nFrames, timeSlices.d1, "");
  str collisions;

  for(const Proxy& p:pathConfig.proxies) {
    if(p.d<belowMargin) {
      uint ta=p.a->ID / nFrames;
      uint tb=p.b->ID / nFrames;
      CHECK_EQ(ta, tb, "collisions across time slices??");
      if(ta==t+k_order || tb==t+k_order){
        collisions <<p.a->name <<'-' <<p.b->name <<": " <<p.d <<' ' <<ta <<' ' <<tb <<'\n';
      }
    }
  }

  return collisions;
}

arr KOMO::info_errorTotals(const arr& errorTraces){
  arr totals = zeros(OT_ineqP+1);
  arr errTotals = sum(errorTraces, 0);
  CHECK_EQ(objectives.N, errTotals.N, "");
  for(uint i=0;i<errTotals.N;i++) totals(objectives(i)->type) += errTotals(i);
  return totals;
}

void KOMO::checkGradients() {
  CHECK(T, "");
#if 1
  shared_ptr<NLP> CP = nlp();
  arr x0 = pathConfig.getJointState();
  CP->checkJacobian(x0, 1e-6, featureNames);
#else
  double tolerance=1e-4;

  shared_ptr<NLP> CP = nlp();

  VectorFunction F = [CP](const arr& x) -> arr{
    arr phi, J;
    CP->evaluate(phi, J, x);
    phi.J() = J;
    return phi;
  };

  arr x0 = pathConfig.getJointState();
  //    checkJacobian(F, x0, tolerance);
  arr J;
  arr JJ=finiteDifferenceJacobian(F, x0, J);
  bool succ=true;
  double mmd=0.;
  for(uint i=0; i<J.d0; i++) {
    uint j;
    double md=maxDiff(J[i], JJ[i], &j);
    if(md>mmd) mmd=md;
    if(md>tolerance && md>fabs(J(i, j))*tolerance) {
      if(featureNames.N) {
        LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<*/' ' <<featureNames(i) <<" -- max diff=" <<md <<" |"<<J(i, j)<<'-'<<JJ(i, j)<<"| (stored in files z.J_*)";
      } else {
        LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<' ' <<komo_problem.featureNames(i) <<*/" -- max diff=" <<md <<" |"<<J(i, j)<<'-'<<JJ(i, j)<<"| (stored in files z.J_*)";
      }
      J[i] >>FILE("z.J_analytical");
      JJ[i] >>FILE("z.J_empirical");
      //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
      //HALT("");
      //        return;
      rai::wait();
      succ=false;
    }
  }
  if(succ) cout <<"jacobianCheck -- SUCCESS (max diff error=" <<mmd <<")" <<endl;
#endif
}

int KOMO::view(bool pause, const char* txt) {
  pathConfig.get_viewer()->updateConfiguration(pathConfig, timeSlices);
//  pathConfig.get_viewer()->recopyMeshes(pathConfig);
//  return pathConfig.view(pause, txt);
  pathConfig.get_viewer()->phaseOffset = 1.-double(k_order);
  pathConfig.get_viewer()->phaseFactor = 1./double(stepsPerPhase);
  if(featureValues.N){
    pathConfig.get_viewer()->sliceTexts.resize(T+k_order);
    arr err = info_objectiveErrorTraces();
    for(uint t=0;t<T;t++){
      pathConfig.get_viewer()->sliceTexts(t+k_order) = info_sliceErrors(t, err);
      pathConfig.get_viewer()->sliceTexts(t+k_order) <<info_sliceCollisions(t);
    }
  }
  pathConfig.get_viewer()->ensure_gl().setTitle("KOMO Viewer");
  str text = txt;
  if(pause) text <<"\n[use SHIFT+scroll or arror keys to browse; press key to continue]";
  return pathConfig.get_viewer()->view(pause, text);
}

int KOMO::view_play(bool pause, const char* txt, double delay, const char* saveVideoPath) {
  str text = txt;
  if(pause) text <<"\n[use SHIFT+scroll or arror keys to browse; press key to continue]";
  view(false, text);
  return pathConfig.get_viewer()->playVideo(pause, delay*tau*T, saveVideoPath);
}

int KOMO::view_slice(uint t, bool pause){
  if(!pathConfig.get_viewer()->gl) view(false, 0);
  return pathConfig.get_viewer()->view_slice(t, pause);
}

void KOMO::view_close() { pathConfig.view_close(); }

std::shared_ptr<ConfigurationViewer> KOMO::get_viewer(){
  return pathConfig.get_viewer();
}

void KOMO::set_viewer(std::shared_ptr<rai::ConfigurationViewer>& _viewer){
  pathConfig.set_viewer(_viewer);
}

void KOMO::plotTrajectory() {
  ofstream fil("z.trajectories");
  StringA jointNames = world.getJointNames();
  //first line: legend
  for(auto s:jointNames) fil <<s <<' ';
  fil <<endl;

  fil <<getPath_qOrg().modRaw() <<endl;
  fil.close();

  ofstream fil2("z.trajectories.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'trajectories'" <<endl;
//  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot 'z.trajectories' \\" <<endl;
  for(uint i=1; i<=jointNames.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u (($0+1)/" <<stepsPerPhase <<"):"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
//  if(dual.N) for(uint i=0;i<objectives.N;i++) fil <<"  ,'' u (($0+1)/" <<stepsPerPhase <<"):"<<1+objectives.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  gnuplot("load 'z.trajectories.plt'");
}

void KOMO::plotPhaseTrajectory() {
  ofstream fil("z.phase");
  //first line: legend
  fil <<"phase" <<endl;

  arr X = getPath_times();

  X.reshape(T, 1);
  X.write(fil, nullptr, nullptr, "  ");
  fil.close();

  ofstream fil2("z.phase.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'phase'" <<endl;
  fil2 <<"set term qt 2" <<endl;
  fil2 <<"plot 'z.phase' u (($0+1)/" <<stepsPerPhase <<"):1 w l lw 3 lc 1 lt 1" <<endl;
  fil2 <<endl;
  fil2.close();

  gnuplot("load 'z.phase.plt'");
}

void KOMO::getSubProblem(uint phase, Configuration& C, arr& q0, arr& q1) {
  CHECK_EQ(stepsPerPhase, 1, "");
  getConfiguration_full(C, phase-1, 0);
  if(!phase) C.selectJoints(DofL{}, true);
  C.ensure_indexedJoints();
  DofL acts = C.activeDofs;
  for(rai::Dof* d:acts) {
    if(d->mimic==d) { d->mimicers.removeValue(d->mimic); d->mimic=0; }
    if(!d->joint() || d->isStable) {
      d->setActive(false);
    }
    if(d->frame->ats) {
      bool* activeKey = d->frame->ats->find<bool>("joint_active");
      if(activeKey && !(*activeKey)) d->setActive(false);
    }
  }
  q0 = C.getJointState();
  C.setFrameState(getConfiguration_X(phase), C.frames({0, world.frames.N-1}));
  q1 = C.getJointState();
  C.setJointState(q0);
  //  C.view(true);
}

//===========================================================================

rai::Frame* KOMO::addFrameDof(const char* name, const char* parent,
                              JointType jointType, bool stable,
                              const char* initName, rai::Frame* initFrame) {
  Frame* p0 = 0;
  if(parent && parent[0]) p0 = world[parent];

  //-- IN WORLD, NOT PATHCONFIG!

  // decide on a relative pose
  rai::Transformation rel = 0;
  if(initName){
    CHECK(!initFrame, "can't specify both, initName and initFrame!");
    initFrame = world.getFrame(initName, true);
  }
  if(initFrame) {
    if(p0){
      rel = initFrame->ensure_X()/p0->ensure_X();
    }else{
      rel = initFrame->ensure_X();
    }
  }

  // link to parent, potentially via inbetween frame
  {
    Frame* f = world.addFrame(name);
    if(rel.isZero()) {
      if(p0) f->setParent(p0, false);
    } else {
      if(jointType!=JT_none && jointType!=JT_rigid){
        Frame* r = world.addFrame(STRING(name<<"_origin"));
        if(p0){
          r->setParent(p0, false);
          r->setRelativePose(rel);
        }else{
          r->setPose(rel);
        }
        f->setParent(r, false);
      }else{
        if(p0){
          f->setParent(p0, false);
          f->setRelativePose(rel);
        }else{
          f->setPose(rel);
        }
      }
    }
  }

  //-- IN PATHCONFIG

  FrameL F, R;
  Frame* f0=0;
  for(uint s=0; s<timeSlices.d0; s++) { //apply switch on all configurations!
    Frame* f = pathConfig.addFrame(name, 0, 0, false);
    Frame* p = 0;
    if(p0){
      p=timeSlices(s, p0->ID);
      CHECK_EQ(p->name, parent, "");
    }
    if(rel.isZero()) {
      if(p) f->setParent(p, false);
    } else {
      if(jointType!=JT_none && jointType!=JT_rigid){
        Frame* r = pathConfig.addFrame(STRING(name<<"_origin"), 0, 0, false);
        if(p){
          r->setParent(p, false);
          r->setRelativePose(rel);
        }else{
          r->setPose(rel);
        }
        f->setParent(r, false);
        R.append(r);
      }else{
        if(p){
          f->setParent(p, false);
          f->setRelativePose(rel);
        }else{
          f->setPose(rel);
        }
      }
    }
    if(initFrame) f->setPose(initFrame->getPose());
    if(jointType!=JT_none) {
      f->setJoint(jointType);
      f->joint->isPartBreak=false;
      if(stable){
        if(f0) f->joint->setMimic(f0->joint);
        else f->setAutoLimits();
        f->joint->isStable=true;
      }else{
        f->setAutoLimits();
      }
    }
//    f->setShape(ST_marker, {.3});  f->setColor({1., 0., 1., .5});
    if(!f0) f0=f;
    F.append(f);
  }

  CHECK_EQ(F.N, timeSlices.d0, "");
  if(rel.isZero() || jointType==JT_none || jointType==JT_rigid) {
    timeSlices.insColumns(-1);
    for(uint s=0; s<timeSlices.d0; s++) timeSlices(s, -1) = F(s);
  } else {
    CHECK_EQ(R.N, timeSlices.d0, "");
    timeSlices.insColumns(-1, 2);
    for(uint s=0; s<timeSlices.d0; s++) { timeSlices(s, -2) = F(s);  timeSlices(s, -1) = R(s); }
  }
  CHECK_EQ(timeSlices.d1, world.frames.N, "");
  if(timeSlices.N==pathConfig.frames.N) {
    pathConfig.frames = timeSlices;
    uint i=0;
    for(Frame* f: pathConfig.frames) f->ID = i++;
  }
  return f0;
}

rai::Frame* KOMO::applySwitch(const KinematicSwitch& sw) {
#if 0 //for debugging
  cout <<"APPLYING SWITCH:\n" <<*sw <<endl;
  cout <<world.frames(sw.fromId)->name <<"->" <<world.frames(sw.toId)->name <<endl;
  sw.apply(world.frames);
  listWriteNames(world.frames(sw.toId)->getPathToRoot(), cout);
  listWriteNames(world.frames(sw.toId)->children, cout);
#endif
  int s = sw.timeOfApplication+(int)k_order;
  if(s<0) s=0;
  int sEnd = int(k_order+T);
  //    if(sw.timeOfTermination>=0)  sEnd = sw.timeOfTermination+(int)k_order;
  CHECK(s<=sEnd, "s:" <<s <<" sEnd:" <<sEnd);
  if(s==sEnd) return 0;
  rai::Frame* f0=0, *f=0;
  for(; s<sEnd; s++) { //apply switch on all configurations!
    f = sw.apply(timeSlices[s].noconst());
    if(!f0) {
      f0=f;
    } else {
      if(sw.symbol==SW_addContact) {
        rai::ForceExchange* ex0 = f0->forces.last();
        rai::ForceExchange* ex1 = f->forces.last();
        ex1->poa = ex0->poa;
      } else {
        f->set_Q() = f0->get_Q(); //copy the relative pose (switch joint initialization) from the first application
        if(opt.mimicStable) {
          /*CRUCIAL CHANGE!*/
          if(sw.isStable) f->joint->setMimic(f0->joint);
        }
      }
    }
  }
  if(sw.isStable && opt.mimicStable) return f0;
  return f;
}

void KOMO::selectJointsBySubtrees(const StringA& roots, const arr& times, bool notThose) {
  uintA rootIds = world.getFrameIDs(roots);

  world.selectJointsBySubtrees(world.getFrames(rootIds), notThose);

  FrameL allRoots;

  if(!times.N) {
    for(uint s=0; s<timeSlices.d0; s++) allRoots.append(pathConfig.getFrames(rootIds+s*timeSlices.d1));
  } else {
    int tfrom = conv_time2step(times(0), stepsPerPhase);
    int tto   = conv_time2step(times(1), stepsPerPhase);
    for(int t=tfrom; t<=tto; t++) allRoots.append(pathConfig.getFrames(rootIds+(t+k_order)*timeSlices.d1));
  }
  pathConfig.selectJointsBySubtrees(allRoots, notThose);
  pathConfig.ensure_q();
  pathConfig.checkConsistency();
}

//===========================================================================

void KOMO::setupPathConfig() {
  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t

  CHECK(world.frames.N, "you need to call setConfig before");
  CHECK(stepsPerPhase, "you need to call setTiming before");

  CHECK(timeSlices.d0 != k_order+T, "why setup again?");
  CHECK(!pathConfig.frames.N, "why setup again?");

  //computeMeshNormals(world.frames, true);
  //computeMeshGraphs(world.frames, true);

  rai::Configuration C;
  C.copy(world, true);
  C.setTaus(tau);

//  if(computeCollisions) {
//    CHECK(!fcl, "");
//    fcl = C.fcl();
//    fcl->mode = fcl->_broadPhaseOnly;
//  }

  for(uint s=0; s<k_order+T; s++) {
    pathConfig.addCopy(C.frames, C.otherDofs);
    if(s==0){ for(rai::Frame* f:pathConfig.frames) f->time = -tau*k_order; } //set times of prefix negative
  }
  timeSlices = pathConfig.frames;

  //deactivate prefix dofs
  pathConfig.calc_indexedActiveJoints();
  uint firstID = timeSlices(k_order, 0)->ID;
  for(Dof* dof:pathConfig.activeDofs) {
    if(dof->frame->ID < firstID) {
      if(!dof->mimicers.N) dof->active=false;
      else {
        bool act=false;
        for(Dof* m:dof->mimicers) if(m->active && m->frame->ID>=firstID) { act=true; break; }
        if(!act) dof->active=false;
      }
    }
  }
  pathConfig.calc_indexedActiveJoints();
//  for(uint t=0; t<T; ++t){
//    for(auto* f:timeSlices[t + k_order]) {
//      if(f->joint && f->joint->active){
//        activeJoints.append(f);
//      }
//    }
//  }
//  pathConfig.selectJoints(activeJoints);

  x = pathConfig.getJointState();
  //pathConfig.checkConsistency();

  if(!x.N){
    LOG(0) <<"you created a KOMO problem with zero dofs -- that's likely a bug";
  }
}

void KOMO::set_x(const arr& x, const uintA& selectedConfigurationsOnly) {
  CHECK_EQ(timeSlices.d0, k_order+T, "configurations are not setup yet");

  timeKinematics -= rai::cpuTime();

  if(!selectedConfigurationsOnly.N) {
    pathConfig.setJointState(x);
  } else {
    pathConfig.setJointState(x, timeSlices.sub(selectedConfigurationsOnly+k_order));
    HALT("this is untested...");
  }

  timeKinematics += rai::cpuTime();

  if(computeCollisions) {
    if(!fcl) {
      fcl = world.fcl();
      fcl->mode = fcl->_broadPhaseOnly;
    }
    timeCollisions -= rai::cpuTime();
    pathConfig.proxies.clear();
    arr X;
    uintA collisionPairs;
    for(uint s=k_order; s<timeSlices.d0; s++) {
      X = pathConfig.getFrameState(timeSlices[s]);
      {
        fcl->step(X);
        collisionPairs = fcl->collisions;
      }
      collisionPairs += timeSlices.d1 * s; //fcl returns frame IDs related to 'world' -> map them into frameIDs within that time slice
      pathConfig.addProxies(collisionPairs);
    }
    pathConfig._state_proxies_isGood=true;
    pathConfig.ensure_proxies(); //expensive!!
    timeCollisions += rai::cpuTime();
  }
}

std::shared_ptr<NLP> KOMO::nlp() {
  return make_shared<KOMO_NLP>(*this);
}

std::shared_ptr<NLP> KOMO::nlp_spline(uint numCtrlPoints, uint degree) {
  return make_shared<KOMO_Spline_NLP>(*this, numCtrlPoints, degree);
}

std::shared_ptr<NLP> KOMO::nlp_sub(const rai::Array<GroundedObjective*>& objs, const DofL& dofs){
  return make_shared<KOMO_SubNLP>(*this, objs, dofs);
}

shared_ptr<NLP_Factored> KOMO::nlp_FactoredTime() {
  rai::Array<DofL> dofs(T);
  for(rai::Dof* d: pathConfig.activeDofs) { //go through all active dofs and sort them in time slices
    if(d->mimic) continue; //remove mimics
    int t = int(d->frame->ID/timeSlices.d1) - int(k_order);
    if(t>=int(T)) t = int(d->frame->parent->ID/timeSlices.d1) - int(k_order);
    CHECK(t<int(T), "hmm");
    if(t>=0) dofs(t).append(d);
  }
  return make_shared<Conv_KOMO_FactoredNLP>(*this, dofs);
}

shared_ptr<NLP_Factored> KOMO::nlp_FactoredParts() {
  rai::Array<DofL> dofs = pathConfig.getPartsDofs();
  return make_shared<Conv_KOMO_FactoredNLP>(*this, dofs);
}

StringA KOMO::getCollisionPairs(double belowMargin) {
  //similar to Configuration::getTotalPenetration
  uint nFrames = world.frames.N;
  CHECK_EQ(nFrames, timeSlices.d1, "");
  intAA collisions(nFrames);

  for(const Proxy& p:pathConfig.proxies) {
    //early check: if proxy is way out of collision, don't bother computing it precise
    if(p.d > p.a->shape->radius()+p.b->shape->radius()+.01+belowMargin) continue;
    //exact computation
    if(!p.collision)((Proxy*)&p)->calc_coll();
    double d = p.collision->getDistance();
    if(d<belowMargin) {
//      cout <<"KOMO collision pair: " <<p.a->name <<"--" <<p.b->name <<" : " <<p.d <<endl;
      uint i=p.a->ID % nFrames;
      uint j=p.b->ID % nFrames;
      if(j<i) { int a=i; i=j; j=a; }
      collisions(i).setAppendInSorted(j);
    }
  }

  StringA cols;
  for(uint i=0; i<collisions.N; i++) {
    for(int j:collisions(i)) {
      cols.append(world.frames.elem(i)->name);
      cols.append(world.frames.elem(j)->name);
    }
  }
  cols.reshape(-1, 2);
//  cout <<"KOMO collision pairs: " <<cols;
  return cols;
}

//===========================================================================

/* x_{-1} = x_{time=0}
 * x_{9}: phase=1 (for stepsPerPhase=10 */
int conv_time2step(double time, uint stepsPerPhase) {
  return (floor(time*double(stepsPerPhase) + .500001))-1;
}
double conv_step2time(int step, uint stepsPerPhase) {
  return double(step+1)/double(stepsPerPhase);
}
void conv_times2steps(int& fromStep, int& toStep, const arr& times, int stepsPerPhase, uint T,
                      int deltaFromStep, int deltaToStep) {
  //interpret times as always, single slice, interval, or tuples
  double fromTime=0, toTime=-1.;
  if(!times || !times.N) {
  } else if(times.N==1) {
    fromTime = toTime = times(0);
  } else {
    CHECK_EQ(times.N, 2, "");
    fromTime = times(0);
    toTime = times(1);
  }

  if(toTime>double(T)/stepsPerPhase+1. && toTime<1e6) {
    LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
  }

  CHECK_GE(stepsPerPhase, 0, "");

  //convert to steps
  fromStep = (fromTime<0.?T-1:conv_time2step(fromTime, stepsPerPhase));
  toStep   = (toTime<0.?T-1:conv_time2step(toTime, stepsPerPhase));

  //account for deltas
  if(deltaFromStep) fromStep+=deltaFromStep;
  if(deltaToStep) toStep+=deltaToStep;

  //clip
  if(fromStep<0) fromStep=0;
  if(toStep>=(int)T && T>0) toStep=T-1;
}

intA conv_times2tuples(const arr& times, uint order, int stepsPerPhase, uint T,
                       int deltaFromStep, int deltaToStep) {

  if(times.N && times.elem(0)==-10.) {
    intA configs(times.N-1);
    for(uint i=0; i<configs.N; i++) configs(i) = times(i+1);
    configs.reshape(-1, order+1);
    return configs;
  }

  int fromStep, toStep;
  conv_times2steps(fromStep, toStep, times, stepsPerPhase, T, deltaFromStep, deltaToStep);

  //create tuples
  intA configs;

  if(toStep>=fromStep)
    configs.resize(1+toStep-fromStep, order+1);
  else configs.resize(0, order+1);

  for(int t=fromStep; t<=toStep; t++)
    for(uint j=0; j<configs.d1; j++) configs(t-fromStep, j) = t+j-int(order);

  return configs;
}
