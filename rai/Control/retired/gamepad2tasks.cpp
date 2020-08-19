/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "gamepad2tasks.h"
#include "../Kin/frame.h"
#include "../Kin/F_qFeatures.h"
#include "../Kin/TM_default.h"
#include "../Kin/TM_proxy.h"

enum BUTTON {
  BTN_NONE = 0,
  BTN_A = 1,
  BTN_B = 2,
  BTN_X = 4,
  BTN_Y = 8,
  BTN_LB = 16,
  BTN_RB = 32,
  BTN_LT = 64,
  BTN_RT = 128,
  BTN_BACK = 256,
  BTN_START = 512,
  BTN_LSTICK = 1024,
  BTN_RSTICK = 2048,
};

inline bool stopButtons(const arr& gamepadState) {
  if(!gamepadState.N) return false;
  uint mode = uint(gamepadState(0));
  if(mode&BTN_LB || mode&BTN_RB || mode&BTN_LT || mode&BTN_RT) return true;
  return false;
}

Gamepad2Tasks::Gamepad2Tasks(TaskControlMethods& _TC, const rai::Configuration& K, const arr& _q0)
  : TC(_TC), q0(_q0),
    endeffR(nullptr), endeffL(nullptr), base(nullptr), torso(nullptr), head(nullptr), headAxes(nullptr), limits(nullptr), coll(nullptr), gripperL(nullptr), gripperR(nullptr) {

  robot = rai::getParameter<rai::String>("robot", "pr2");

  if(true || rai::getParameter<bool>("oldfashinedTaskControl", true)) {
    homing = new CtrlObjective("qHoming", make_shared<F_qItself>(), .5, 1., .2, 10.);
    homing->PD().setTarget(q0);
    endeffR = new CtrlObjective("pr2R", make_shared<TM_Default>(TMT_pos, K, "pr2R", NoVector, "base_footprint"), .5, .8, 1., 1.);
    endeffL = new CtrlObjective("pr2L", make_shared<TM_Default>(TMT_pos, K, "pr2L", NoVector, "base_footprint"), .5, .8, 1., 1.);
    //  base = new CtrlObjective("endeffBase", make_shared<TM_qItself>(MP.world, "worldTranslationRotation"), .2, .8, 1., 1.);
    //  torso = new CtrlObjective("torso_lift_link", make_shared<TM_Default>(TMT_pos, MP.world, "torso_lift_link_1"), .2, .8, 1., 1.);
    head = new CtrlObjective("endeffHead", make_shared<TM_Default>(TMT_gazeAt, K, "endeffHead", Vector_z, "base_footprint"), .5, 1., 1., 1.);
    if(robot=="pr2") headAxes = new CtrlObjective("endeffHead", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"head_pan_joint", "head_tilt_joint"}), K), .5, 1., 1., 1.);
    if(robot=="baxter") headAxes = new CtrlObjective("endeffHead", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"head_pan"}), K), .5, 1., 1., 1.);
    limits = new CtrlObjective("limits", make_shared<F_qLimits>(), .2, .8, 1., 1.);
    coll = new CtrlObjective("collisions", make_shared<TM_Proxy>(TMT_allP, uintA({0u}), .1), .2, .8, 1., 1.);
    if(robot=="pr2") {
      base = new CtrlObjective("endeffBase", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"worldTranslationRotation"}), K), .2, .8, 1., 1.);
      torso = new CtrlObjective("torso_lift_link", make_shared<TM_Default>(TMT_pos, K, "torso_lift_link_1"), .2, .8, 1., 1.);
      gripperL = new CtrlObjective("gripperL", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"l_gripper_joint"}), K), 2., .8, 1., 1.);
      gripperR = new CtrlObjective("gripperR", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"r_gripper_joint"}), K), 2., .8, 1., 1.);
    }
    if(robot=="baxter") {
      gripperL = new CtrlObjective("gripperL", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"l_gripper_l_finger_joint"}), K), 2., .8, 1., 1.);
      gripperR = new CtrlObjective("gripperR", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"r_gripper_l_finger_joint"}), K), 2., .8, 1., 1.);
    }
  } else {
    homing = new CtrlObjective("qHoming", make_shared<F_qItself>(), .5, 1., 0., 0.);
//    homing->PD().setGains(10., 2.);
    homing->PD().setTarget(q0);
    endeffR = new CtrlObjective("pr2R", make_shared<TM_Default>(TMT_pos, K, "pr2R", NoVector, "base_footprint"), 1., .1, 1., 1.);
    endeffL = new CtrlObjective("pr2L", make_shared<TM_Default>(TMT_pos, K, "pr2L", NoVector, "base_footprint"), .5, .8, 1., 1.);
    base = new CtrlObjective("endeffBase", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"worldTranslationRotation"}), K), .2, .8, 1., 1.);
    torso = new CtrlObjective("torso_lift_link", make_shared<TM_Default>(TMT_pos, K, "torso_lift_link_0"), .2, .8, 1., 1.);
    head = new CtrlObjective("endeffHead", make_shared<TM_Default>(TMT_gazeAt, K, "endeffHead", Vector_z, "base_footprint"), 1., .8, 1., 1.);
    headAxes = new CtrlObjective("endeffHead", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"head_pan_joint", "head_tilt_joint"}), K), .5, 1., 1., 1.);
    limits = new CtrlObjective("limits", make_shared<F_qLimits>(), .2, .8, 1., 1.);
    coll = new CtrlObjective("collisions", make_shared<TM_Proxy>(TMT_allP, uintA({0u}), .1), .2, .8, 1., 1.);
    gripperL = new CtrlObjective("gripperL", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"l_gripper_joint"}), K), 2., .8, 1., 1.);
    gripperR = new CtrlObjective("gripperR", make_shared<F_qItself>(F_qItself::byJointNames, StringA({"r_gripper_joint"}), K), 2., .8, 1., 1.);

    endeffR->PD().setGains(40., 2.);
    endeffL->PD().setGains(10., 1.); //endeffL->maxAcc=.5;
    headAxes->PD().setGains(10., 5.);
  }
  for(CtrlObjective* task: { homing, endeffR, endeffL, head, headAxes, limits, coll, gripperL, gripperR })
    task->active=false;

  if(robot=="pr2") {
    base->active=false;
    torso->active=false;
  }
}

rai::Array<CtrlObjective*> Gamepad2Tasks::getTasks() {
  if(robot=="pr2") { return { homing, endeffR, endeffL, base, torso, head, headAxes, limits, coll, gripperL, gripperR }; }
  else if(robot=="baxter") { return { homing, endeffR, endeffL, head, headAxes, limits, coll, gripperL, gripperR }; }
  else { NIY; }
}

double gamepadSignalMap(double x) {
  return rai::sign(x)*(exp(rai::sqr(x))-1.);
}

bool Gamepad2Tasks::updateTasks(arr& gamepadState, const rai::Configuration& K) {
  if(stopButtons(gamepadState)) return true;

  //for(ptr<CtrlObjective>& pdt:TC.tasks) pdt->active=false;

  HALT("change code: add a qNull here explicitly");
//  TC.qNullCostRef.PD().setGains(0., 10.); //nullspace qitself is not used for homing by default
//  TC.qNullCostRef.active=true;
//  TC.qNullCostRef.PD().setTarget(K.q);

//  homing->PD().setGains(0., 10.); //nullspace qitself is not used for homing by default
//  homing->active=true;
//  homing->PD().setTarget(MP.world.q);
  //  limits->active=true;
//  coll->active=true;

  if(gamepadState.N<6) return false;

  double gamepadRate=rai::getParameter<double>("gamepadRate", .2);
  for(uint i=1; i<gamepadState.N; i++) if(fabs(gamepadState(i))<0.05) gamepadState(i)=0.;
  double gamepadLeftRight   = -gamepadRate*gamepadSignalMap(gamepadState(4));
  double gamepadForwardBack = -gamepadRate*gamepadSignalMap(gamepadState(3));
  double gamepadUpDown      = -gamepadRate*gamepadSignalMap(gamepadState(2));
  double gamepadRotate      = -gamepadRate*gamepadSignalMap(gamepadState(1));

  uint mode = uint(gamepadState(0));

  enum {none, up, down, downRot, left, right} sel=none;
  if(fabs(gamepadState(5))>.5 || fabs(gamepadState(6))>.5) {
    if(fabs(gamepadState(5))>fabs(gamepadState(6))) {
      if(gamepadState(5)>0.) sel=right; else sel=left;
    } else {
      if(gamepadState(6)>0.) sel=down; else sel=up;
    }
  }

  switch(mode) {
    case 0: { //(NIL) motion rate control
      CtrlObjective* pdt=nullptr;
      switch(sel) {
        case right:  pdt=endeffR;  cout <<"effR control" <<endl;  break;
        case left:   pdt=endeffL;  cout <<"effL control" <<endl;  break;
//        case up:     pdt=torso;  cout <<"torso control" <<endl;  break;
        case up:     pdt=headAxes; cout <<"head control" <<endl;  break;
        case down:   pdt=base;  cout <<"base control" <<endl;  break;
        case none:   pdt=nullptr;  break;
        case downRot: break;
      }
      if(!pdt) break;
      pdt->active=true;
      if(!pdt->y.N || !pdt->v.N) {
        pdt->feat->__phi(pdt->y, NoArr, K);
      }
      rai::Vector vel(gamepadLeftRight, gamepadForwardBack, gamepadUpDown);
      if(sel==down) {
        vel.set(.5*gamepadLeftRight, .5*gamepadRotate, 2.*gamepadForwardBack);
        vel = K.getFrameByName("endeffBase") -> ensure_X().rot * vel;
      }
//      vel = MP.world.getShapeByName("endeffBase")->X.rot*vel;
      arr ve;
      ve = conv_vec2arr(vel);
      if(sel==up) {
        if(robot=="pr2") ve = ARR(ve(1), -ve(0));
        if(robot=="baxter") ve = ARR(ve(1));
      }
      pdt->PD().y_target = pdt->y + 0.01*ve;
      pdt->PD().v_target = ve; //setZero();
//      if(sel!=up)  MP.world.getShapeByName("mymarker")->rel.pos = pdt->PD().y_target;

      //-- left right: gaze control
//      if(head && (sel==left || sel==right)){
//        head->active=true;
//        dynamic_cast<TM_Default*>(&head->feat)->jvec = pdt->y;
//      }
      break;
    }
    case 1: { //homing
      cout <<"homing" <<endl;
      homing->PD().setTarget(q0);
      rai::Joint* j = K.getFrameByName("worldTranslationRotation")->joint;
      if(j) {
        arr b;
        base->feat->__phi(b, NoArr, K);
        if(b.N && j && j->qDim()) {
          for(uint i=0; i<j->qDim(); i++)
            homing->PD().y_target(j->qIndex+i) = b(i);
        }
      }
      homing->active = true;
      break;
    }
    case 4:
    case 8: { //open/close hand
      cout <<"open/close hand" <<endl;
      CtrlObjective* pdt=nullptr;
      switch(sel) {
        case right:  pdt=gripperR;  break;
        case left:   pdt=gripperL;  break;
        default:     pdt=nullptr;  break;
      }
      if(!pdt) break;
      if(robot=="pr2") {
        if(mode==8) pdt->PD().y_target=ARR(.08); else pdt->PD().y_target=ARR(.01);
      }
      if(robot=="baxter") {
        if(mode==8) pdt->PD().y_target=ARR(.1); else pdt->PD().y_target=ARR(0.);
      }
      pdt->active=true;
      break;
    }
//    case 2: { //(2) CRAZY tactile guiding
//      skin->active=true;
//      skin->y_prec = 5e1;
//      skin->y_target=ARR(.0, .0, .0, .0, .0, .0);
//      //ON SIMULATION: since it is set to (.01, .01, .01) this will always give a repelling force!
//      break;
//    }
//    case 256: { //(select)close hand
//      skin->active=true;
//      skin->y_target=ARR(.007, 0, .02, 0, .007, 0);
//      break;
//    }
//    case 512: { //(start)open hand
//      arr target = q->y;
//      target(8)=target(10)=target(12)=-.8;
//      target(9)=target(11)=target(13)= .6;
//      q->v_target = 1.*(target - q->y);
//      double vmax=.5, v=absMax(q->v_target);
//      if (v>vmax) q->v_target*=vmax/v;
//      break;
//    }
  }
  return false;
}

