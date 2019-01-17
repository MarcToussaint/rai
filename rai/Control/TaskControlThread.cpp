/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include <RosCom/baxter.h>
#include <Kin/frame.h>

TaskControlThread::TaskControlThread(const rai::KinematicWorld& _model,
                  const Var<CtrlMsg>& _ctrl_ref,
                  const Var<CtrlMsg>& _ctrl_state)
  : Thread("TaskControlThread", .01),
    ctrl_ref(_ctrl_ref),
    ctrl_state(_ctrl_state),
//    model_ref(_model),
//    model_real(_model),
    taskController(NULL),
    useSwift(true),
    requiresInitialSync(true),
    verbose(0)
{
  
  useSwift = rai::getParameter<bool>("useSwift", true);
  kp_factor = rai::getParameter<double>("controller_kp_factor", 1.);
  kd_factor = rai::getParameter<double>("controller_kd_factor", 1.);
  ki_factor = rai::getParameter<double>("controller_ki_factor", .2);
  
  double hyper = rai::getParameter<double>("hyperSpeed", -1.);
  if(hyper>0.) this->metronome.reset(.01/hyper);

  //memorize the "NULL position", which is the initial model position
  model_ref.set() = _model;
  q0 = _model.getJointState();
  q_real = q_model = q0;
  qdot_real = qdot_model = zeros(q0.N);

  //initialize Kp and Kd
  Kp_base = zeros(q0.N);
  Kd_base = zeros(q0.N);
  for(rai::Joint *j:_model.fwdActiveJoints) {
    arr *gains = j->frame.ats.find<arr>("gains");
    if(gains) {
      for(uint i=0; i<j->qDim(); i++) {
        Kp_base(j->qIndex+i)=gains->elem(0);
        Kd_base(j->qIndex+i)=gains->elem(1);
      }
    }
  }

  taskController = make_shared<TaskControlMethods>(_model);

  threadLoop();
}

TaskControlThread::~TaskControlThread() {
  threadClose();
}

void TaskControlThread::step() {
//  rai::Frame *transF = model_real.getFrameByName("worldTranslationRotation", false);
//  rai::Joint *trans = (transF?transF->joint:NULL);
  
  if(requiresInitialSync){
    if(ctrl_state.getRevision()>1){
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qdot;
      }
      model_ref.set()->setJointState(q_real);
//      model_real.setJointState(q_real);
      q_model = q_real;
      qdot_model = qdot_real;
      requiresInitialSync = false;
    }else{
      LOG(0) <<"waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  {
    auto state = ctrl_state.get();
    if(state->q.N){
      q_real = state->q;
      qdot_real = state->qdot;
    }
  }
//  model_real.setJointState(q_real);
  if(false){
    model_ref.set()->setJointState(q_real);
    q_model = q_real;
    qdot_model = qdot_real;
  }


  //-- compute the feedback controller step and iterate to compute a forward reference
  arr P_compliance;
  {
    auto K = model_ref.set();

    if(!(step_count%20)) K->watch(false, STRING("TaskControlThread model_ref " <<step_count)); //only for debugging

    ctrlTasks.writeAccess();
    taskController->tasks = ctrlTasks();
    taskController->updateCtrlTasks(.01, K); //update with time increment
    
    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;
    dq = taskController->inverseKinematics(qdot_model); //don't include a null step
    if(dq.N){
      double l = length(dq);
      if(l>maxQStep) dq *= maxQStep/l;
      q_model += dq;
    }
    
    //set/test the new configuration
    K->setJointState(q_model, qdot_model);
    if(useSwift) K->stepSwift();
    taskController->updateCtrlTasks(0., K); //update without time increment
    double cost = taskController->getIKCosts();
//    IK_cost.set() = cost;
    
    //check the costs
    if(cost>1000.) { //reject!
      LOG(-1) <<"HIGH COST IK! " <<cost;
      q_model -= .9*dq;
      K->setJointState(q_model, qdot_model);
      if(useSwift) K->stepSwift();
      taskController->updateCtrlTasks(0., K); //update without time increment
    }
    
    if(verbose) taskController->reportCurrentState();
    
    // get compliance projection matrix
    P_compliance = taskController->getComplianceProjection();

    ctrlTasks.deAccess();
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
  //    ctrlTasks.readAccess();
  //    taskController->tasks = ctrlTasks();
  //    for(CtrlTask *t : taskController->tasks) {
  //      if(t->active && t->f_ref.N){
  //        count++;
  //        if(count!=1) HALT("you have multiple active force control tasks - NIY");
  //        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
  //      }
  //    }
  //    if(count==1) refs.Kp = .5;
  //    ctrlTasks.deAccess();
  
  //-- output: set variables
  if(true){
    CtrlMsg refs;
    refs.q =  q_model;
    refs.qdot = zeros(q_model.N);
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
    
//    ctrl_q_ref.set() = refs.q;
    
    //-- set base motion command as velocities
#if 0
    if(!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }
#endif

    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

CtrlTask *TaskControlThread::addCtrlTask(const char* name, const ptr<Feature>& map, const ptr<MotionProfile>& ref){
  CtrlTask *t = new CtrlTask(name, map, ref);
  t->update(0., model_ref.get());
  ctrlTasks.set()->append(t);
  return t;
}

CtrlTask *TaskControlThread::addCtrlTask(const char* name, FeatureSymbol fs, const StringA& frames,
                                    double decayTime, double dampingRatio, double maxVel, double maxAcc){
  CtrlTask *t = new CtrlTask(name, fs, frames, model_ref.get(), decayTime,  dampingRatio,  maxVel, maxAcc);
  t->update(0., model_ref.get());
  ctrlTasks.set()->append(t);
  return t;
}

CtrlTask* TaskControlThread::addCompliance(const char* name, FeatureSymbol fs, const StringA& frames, const arr& compliance){
  CtrlTask *t = new CtrlTask(name, ptr<Feature>(symbols2feature(fs, frames, model_ref.get())));
  t->complianceDirection = compliance;
  ctrlTasks.set()->append(t);
  return t;
}

void TaskControlThread::removeCtrlTask(CtrlTask* t){
  ctrlTasks.set()->removeValue(t);
}
