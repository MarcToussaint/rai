/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include "CtrlSolvers.h"
#include "CtrlTargets.h"
#include "../RosCom/baxter.h"
#include "../Kin/frame.h"

TaskControlThread::TaskControlThread(const Var<rai::Configuration>& _ctrl_config,
                                     const Var<CtrlMsg>& _ctrl_ref,
                                     const Var<CtrlMsg>& _ctrl_state,
                                     const Var<CtrlObjectiveL>& _ctrl_tasks)
  : Thread("TaskControlThread", .01),
    ctrl_config(this, _ctrl_config),
    ctrl_ref(this, _ctrl_ref),
    ctrl_state(this, _ctrl_state),
    ctrl_tasks(this, _ctrl_tasks),
    useSwift(true),
    requiresInitialSync(true),
    verbose(0) {

  useSwift = rai::getParameter<bool>("useSwift", true);
  kp_factor = rai::getParameter<double>("controller_kp_factor", 1.);
  kd_factor = rai::getParameter<double>("controller_kd_factor", 1.);
  ki_factor = rai::getParameter<double>("controller_ki_factor", .2);

  double hyper = rai::getParameter<double>("hyperSpeed", -1.);
  if(hyper>0.) this->metronome.reset(.01/hyper);

  //memorize the "nullptr position", which is the initial model position
  q0 = ctrl_config.get()->getJointState();
  q_real = q_model = q0;
  qdot_real = qdot_model = zeros(q0.N);

  //initialize Kp and Kd
  Kp_base = zeros(q0.N);
  Kd_base = zeros(q0.N);
  for(rai::Joint* j:ctrl_config.get()->activeJoints) {
    arr* gains = j->frame->ats.find<arr>("gains");
    if(gains) {
      for(uint i=0; i<j->qDim(); i++) {
        Kp_base(j->qIndex+i)=gains->elem(0);
        Kd_base(j->qIndex+i)=gains->elem(1);
      }
    }
  }

  Hmetric = rai::getParameter<double>("Hrate", .1)*ctrl_config.get()->getCtrlMetric();

  threadLoop();
}

TaskControlThread::~TaskControlThread() {
  threadClose();
}

arr TaskControlThread::whatsTheForce(const ptr<CtrlObjective>& t) {
// arr tau = ctrl_state.get()->u_bias;
  NIY;
//  return pseudoInverse(~t->J_y)*torques_real;
}

void TaskControlThread::step() {
//  rai::Frame *transF = model_real.getFrameByName("worldTranslationRotation", false);
//  rai::Joint *trans = (transF?transF->joint:nullptr);

  if(requiresInitialSync) {
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qdot;
      }
      ctrl_config.set()->setJointState(q_real);
//      model_real.setJointState(q_real);
      q_model = q_real;
      qdot_model = qdot_real;
      requiresInitialSync = false;
    } else {
//      LOG(0) <<"waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  {
    auto state = ctrl_state.get();
    if(state->q.N) {
      q_real = state->q;
      qdot_real = state->qdot;
      torques_real = state->u_bias;
    }
  }

//  model_real.setJointState(q_real);
  if(true) {
    ctrl_config.set()->setJointState(q_real);
    q_model = q_real;
    qdot_model = qdot_real;
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  arr P_compliance;
  {
    auto K = ctrl_config.set();

    if(!(step_count%20)) {
      rai::String txt;
      txt <<"TaskControlThread ctrl_config " <<step_count;
      for(const CtrlObjective* t:ctrl_tasks.get()()) { txt <<'\n'; t->reportState(txt); }
      K->watch(false, txt); //only for debugging
    }

    ctrl_tasks.writeAccess();
    for(CtrlObjective* t: ctrl_tasks()) NIY; // t->update(.01, K);

    TaskControlMethods taskController(Hmetric);

    //-- get compliance projection matrix
    P_compliance = taskController.getComplianceProjection(ctrl_tasks());

    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;
    dq = taskController.inverseKinematics({K.data}, ctrl_tasks(), qdot_model, P_compliance); //don't include a null step
    if(dq.N) {
      double l = length(dq);
      if(l>maxQStep) dq *= maxQStep/l;
      q_model += dq;
    }

    q_real = q_model;

#if 0
    //set/test the new configuration
    K->setJointState(q_model); //DONT! the configuration should stay on real; use a separate one for safty checks
    if(useSwift) K->stepSwift();
    for(ptr<CtrlObjective>& t: ctrl_tasks()) t->update(.0, K); //update without time increment
    double cost = taskController.getIKCosts(ctrl_tasks());
//    IK_cost.set() = cost;

    //check the costs
    if(cost>1000.) { //reject!
      LOG(-1) <<"HIGH COST IK! " <<cost;
      q_model -= .9*dq;
      K->setJointState(q_model);
      if(useSwift) K->stepSwift();
      for(ptr<CtrlObjective>& t: ctrl_tasks()) t->update(.0, K); //update without time increment
    }
#endif

    if(verbose) taskController.reportCurrentState(ctrl_tasks());

    ctrl_tasks.deAccess();
  }

  //set gains

  //project gains if there are compliance tasks
  arr Kp = diag(Kp_base);  Kp *= kp_factor;
  arr Kd = diag(Kd_base);  Kd *= kd_factor;
//  arr Ki = diag(Kp_base);  Ki *= ki_factor;
  arr Ki = ARR(ki_factor);

//  Kp = M*Kp; DANGER!!
//  Kd = M*Kd;

  if(P_compliance.N) {
    Kp = P_compliance*Kp*P_compliance;
    Kd = P_compliance*Kd*P_compliance;
    Ki = P_compliance*diag(Kp_base*ki_factor)*P_compliance;
  }

  //TODO: construct force tasks
  //    //-- compute the force feedback control coefficients
  //    uint count=0;
  //    ctrl_tasks.readAccess();
  //    taskController.tasks = ctrl_tasks();
  //    for(CtrlObjective *t : taskController.tasks) {
  //      if(t->active && t->f_ref.N){
  //        count++;
  //        if(count!=1) HALT("you have multiple active force control tasks - NIY");
  //        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
  //      }
  //    }
  //    if(count==1) refs.Kp = .5;
  //    ctrl_tasks.deAccess();

  //-- output: set variables
  if(true) {
    CtrlMsg refs;
    refs.q = q_model;
    refs.qdot = qdot_model;
    refs.P_compliance = P_compliance;
    refs.fL_gamma = 1.;
    refs.Kp = Kp; //ARR(1.);
    refs.Kd = Kd; //ARR(1.);
    refs.Ki = Ki; //ARR(.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_model.N);
    refs.intLimitRatio = 1.;
    refs.qd_filt = .99;

    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrl_tasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, const ptr<Feature>& map,
                                    const ptr<CtrlMovingTarget>& ref) {
  NIY
#if 0
  ptr<CtrlObjective> t = make_shared<CtrlObjective>(name, map, ref);
  t->update(0., ctrl_config.get());
  ctrl_tasks.set()->append(t.get());
  return t;
#endif
}

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrl_tasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    const ptr<CtrlMovingTarget>& ref) {
  return addCtrlObjective(ctrl_tasks, ctrl_config, name,
                          symbols2feature(fs, frames, ctrl_config.get()),
                          ref);
}

ptr<CtrlObjective> addCtrlObjective(Var<CtrlObjectiveL>& ctrl_tasks,
                                    Var<rai::Configuration>& ctrl_config,
                                    const char* name, FeatureSymbol fs, const StringA& frames,
                                    double duration) {
  return addCtrlObjective(ctrl_tasks, ctrl_config, name,
                          symbols2feature(fs, frames, ctrl_config.get()),
                          make_shared<CtrlTarget_Sine>(arr(), duration));
}

ptr<CtrlObjective> addCompliance(Var<CtrlObjectiveL>& ctrl_tasks,
                                 Var<rai::Configuration>& ctrl_config,
                                 const char* name, FeatureSymbol fs, const StringA& frames,
                                 const arr& compliance) {
#if 0
  ptr<CtrlObjective> t = make_shared<CtrlObjective>(name, symbols2feature(fs, frames, ctrl_config.get()));
  t->compliance = compliance;
  t->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(t.get());
  return t;
#else
  NIY //have explicit compliance objectives, separate to CtrlObjective
#endif
}

void removeCtrlObjective(Var<CtrlObjectiveL>& ctrl_tasks, CtrlObjective* t) {
  ctrl_tasks.set()->removeValue(t);
}
