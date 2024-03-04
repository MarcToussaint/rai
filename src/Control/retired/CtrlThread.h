/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlMsgs.h"

#include "../Core/thread.h"
#include "../Kin/kin.h"

struct ConstantReference {
  arr q, qDot, qDDot; //default references
  void defaultReferenceFunction(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, double time) {
    q_ref = q;
    qDot_ref = qDot;
    qDDot_ref = qDDot;
  }
};

struct ControlLoop {
  shared_ptr<ConstantReference> _ref;

  virtual void initialize(const arr& q_real, const arr& qDot_real) {}

  virtual void stepReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real) { NIY; }

  virtual void step(rai::CtrlCmdMsg& ctrlCmdMsg, const arr& q_real, const arr& qDot_real, double time) {
    ctrlCmdMsg.controlType=rai::ControlType::configRefs;
    if(!_ref) _ref = make_shared<ConstantReference>();
    stepReference(_ref->q, _ref->qDot, _ref->qDDot, q_real, qDot_real);
    ctrlCmdMsg.ref = std::bind(&ConstantReference::defaultReferenceFunction,
                               _ref,
                               std::placeholders::_1,
                               std::placeholders::_2,
                               std::placeholders::_3,
                               std::placeholders::_4);
  }

};

struct ControlThread : Thread {
  Var<rai::Configuration> ctrl_config;
  Var<rai::CtrlCmdMsg> ctrl_ref;
  Var<rai::CtrlStateMsg> ctrl_state;

  shared_ptr<ControlLoop> ctrlLoop;

  arr q_real, qdot_real, tauExternal; //< real state
  arr q0; //< homing pose
  //arr Hmetric;

  bool requiresInitialSync;
  int verbose;

  ControlThread(const rai::Configuration& C,
                const Var<rai::CtrlCmdMsg>& _ctrl_ref,
                const Var<rai::CtrlStateMsg>& _ctrl_state,
                const shared_ptr<ControlLoop>& _ctrlLoop)
    : Thread("ControlThread", .01),
      ctrl_ref(this, _ctrl_ref),
      ctrl_state(this, _ctrl_state),
      ctrlLoop(_ctrlLoop),
      requiresInitialSync(true),
      verbose(0) {
    ctrl_config.set() = C;

    double hyper = rai::getParameter<double>("hyperSpeed", -1.);
    if(hyper>0.) this->metronome.reset(.01/hyper);

    //memorize the "NULL position", which is the initial model position
    q0 = ctrl_config.get()->getJointState();
    q_real = q0;
    qdot_real = zeros(q0.N);

    //Hmetric = rai::getParameter<double>("Hrate", .1)*ctrl_config.get()->getHmetric();

    threadLoop();
  };
  ~ControlThread() {
    threadClose();
  }

  void step();
};

void ControlThread::step() {
  //-- initial initialization
  if(requiresInitialSync) {
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qDot;
      }
      ctrl_config.set()->setJointState(q_real);
      ctrlLoop->initialize(q_real, qdot_real);
      requiresInitialSync = false;
    } else {
      LOG(0) << "waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  rai::CtrlStateMsg ctrlStateMsg;
  {
    auto state = ctrl_state.get();
    ctrlStateMsg = ctrl_state();
    if(state->q.N) {
      q_real = state->q;
      qdot_real = state->qDot;
      tauExternal = state->tauExternal;
    }
  }

  //-- update kinematic world for controller
  ctrl_config.set()->setJointState(q_real);

  //-- call the given ctrlLoop
  rai::CtrlCmdMsg ctrlCmdMsg;
  ctrlLoop->step(ctrlCmdMsg, q_real, qdot_real, rai::realTime());
  ctrl_ref.set() = ctrlCmdMsg;
}
