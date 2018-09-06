/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include <RosCom/baxter.h>
#include <Kin/frame.h>

TaskControlThread::TaskControlThread(const rai::KinematicWorld& world)
  : Thread("TaskControlThread", .01)
  , taskController(NULL)
  , useSwift(true)
  , requiresInitialSync(true)
  , verbose(false)
  {
  
  useSwift = rai::getParameter<bool>("useSwift", true);
  kp_factor = rai::getParameter<double>("controller_kp_factor", 1.);
  kd_factor = rai::getParameter<double>("controller_kd_factor", 1.);
  ki_factor = rai::getParameter<double>("controller_ki_factor", .2);
  
  double hyper = rai::getParameter<double>("hyperSpeed", -1.);
  if(hyper>0.) this->metronome.reset(.01/hyper);

  realWorld = world;
  modelWorld.set()() = realWorld;
  
  realWorld.calc_q();
  q0 = realWorld.q;
  Kp_base = zeros(realWorld.q.N);
  Kd_base = zeros(realWorld.q.N);

  for(rai::Joint *j:realWorld.fwdActiveJoints) {
    arr *gains = j->frame.ats.find<arr>("gains");
    if(gains) {
      for(uint i=0; i<j->qDim(); i++) {
        Kp_base(j->qIndex+i)=gains->elem(0);
        Kd_base(j->qIndex+i)=gains->elem(1);
      }
    }
  }
}

TaskControlThread::~TaskControlThread() {
  threadClose();
}

void TaskControlThread::open() {
  modelWorld.set() = realWorld;
  modelWorld.get()->getJointState(q_model, qdot_model);
  makeConvexHulls(modelWorld.set()->frames);
  
  taskController = new TaskControlMethods(modelWorld.get());
  
  requiresInitialSync=true;
}

void TaskControlThread::step() {
  rai::Frame *transF = realWorld.getFrameByName("worldTranslationRotation", false);
  rai::Joint *trans = (transF?transF->joint:NULL);
  
  //-- compute the feedback controller step and iterate to compute a forward reference
  arr CompProj;
  {
    modelWorld.writeAccess();
    ctrlTasks.writeAccess();

    //    if(!(step_count%20)) modelWorld().gl().update(); //only for debugging

    taskController->tasks = ctrlTasks();
    
    taskController->updateCtrlTasks(.01, modelWorld()); //update with time increment
    
    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;
    dq = taskController->inverseKinematics(qdot_model); //don't include a null step
    double l = length(dq);
    if(l>maxQStep) dq *= maxQStep/l;
    q_model += dq;
    
    //set/test the new configuration
    modelWorld().setJointState(q_model, qdot_model);
    if(useSwift) modelWorld().stepSwift();
    taskController->updateCtrlTasks(0., modelWorld()); //update without time increment
    double cost = taskController->getIKCosts();
    IK_cost.set() = cost;
    
    //check the costs
    if(cost>1000.) { //reject!
      LOG(-1) <<"HIGH COST IK! " <<cost;
      q_model -= .9*dq;
      modelWorld().setJointState(q_model, qdot_model);
      if(useSwift) modelWorld().stepSwift();
      taskController->updateCtrlTasks(0., modelWorld()); //update without time increment
    }
    
    if(verbose) taskController->reportCurrentState();
    
    // get compliance projection matrix
    CompProj = taskController->getComplianceProjection();
    
//    modelWorld().equationOfMotion(M,F);

    ctrlTasks.deAccess();
    modelWorld.deAccess();
  }
  
  //set gains
  
  //project gains if there are compliance tasks
  arr Kp = diag(Kp_base);  Kp *= kp_factor;
  arr Kd = diag(Kd_base);  Kd *= kd_factor;
//  arr Ki = diag(Kp_base);  Ki *= ki_factor;
  arr Ki = ARR(ki_factor);
  
//  Kp = M*Kp; DANGER!!
//  Kd = M*Kd;

  if(CompProj.N) {
    Kp = CompProj*Kp*CompProj;
    Kd = CompProj*Kd*CompProj;
    Ki = CompProj*diag(Kp_base*ki_factor)*CompProj;
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
  {
    CtrlMsg refs;
    refs.q =  q_model;
    refs.qdot = zeros(q_model.N);
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
    
    ctrl_q_ref.set() = refs.q;
    
    //-- set base motion command as velocities
    if(!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }
    
    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

void TaskControlThread::close() {
  delete taskController;
}
