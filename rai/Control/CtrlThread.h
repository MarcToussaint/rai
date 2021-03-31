#include "CtrlMsgs.h"

#include <Core/thread.h>
#include <Kin/kin.h>

struct ControlLoop {
  virtual void initialize(const arr& q_real, const arr& qDot_real) {}

  virtual void stepReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real){ NIY; }

  virtual void step(CtrlCmdMsg& ctrlCmdMsg, const arr& q_real, const arr& qDot_real){
    ctrlCmdMsg.controlType=ControlType::configRefs;
    stepReference(ctrlCmdMsg.qRef, ctrlCmdMsg.qDotRef, ctrlCmdMsg.qDDotRef, q_real, qDot_real);
  }
};


struct ControlThread : Thread {
  Var<rai::Configuration> ctrl_config;
  Var<CtrlCmdMsg> ctrl_ref;
  Var<CtrlStateMsg> ctrl_state;

  shared_ptr<ControlLoop> ctrlLoop;

  arr q_real, qdot_real, tauExternal; //< real state
  arr q0; //< homing pose
  //arr Hmetric;

  bool requiresInitialSync;
  int verbose;

  ControlThread(const Var<rai::Configuration>& _ctrl_config,
                const Var<CtrlCmdMsg>& _ctrl_ref,
                const Var<CtrlStateMsg>& _ctrl_state,
                const shared_ptr<ControlLoop>& _ctrlLoop)
      : Thread("ControlThread", .01),
      ctrl_config(this, _ctrl_config),
      ctrl_ref(this, _ctrl_ref),
      ctrl_state(this, _ctrl_state),
      ctrlLoop(_ctrlLoop),
      requiresInitialSync(true),
      verbose(0)
  {

    double hyper = rai::getParameter<double>("hyperSpeed", -1.);
    if(hyper>0.) this->metronome.reset(.01/hyper);

    //memorize the "NULL position", which is the initial model position
    q0 = ctrl_config.get()->getJointState();
    q_real = q0;
    qdot_real = zeros(q0.N);

    //Hmetric = rai::getParameter<double>("Hrate", .1)*ctrl_config.get()->getHmetric();

    threadLoop();
  };
  ~ControlThread(){
      threadClose();
  }

  void step();
};


void ControlThread::step() {
  //-- initial initialization
  if(requiresInitialSync){
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qDot;
      }
      ctrl_config.set()->setJointState(q_real);
      ctrlLoop->initialize(q_real, qdot_real);
      requiresInitialSync = false;
    } else{
      LOG(0) << "waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  CtrlStateMsg ctrlStateMsg;
  {
    auto state = ctrl_state.get();
    ctrlStateMsg = ctrl_state();
    if(state->q.N){
      q_real = state->q;
      qdot_real = state->qDot;
      tauExternal = state->tauExternal;
    }
  }

  //-- update kinematic world for controller
  ctrl_config.set()->setJointState(q_real);

  //-- call the given ctrlLoop
  CtrlCmdMsg ctrlCmdMsg;
  ctrlLoop->step(ctrlCmdMsg, q_real, qdot_real);
  ctrl_ref.set() = ctrlCmdMsg;
}
