/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include <Gui/opengl.h>
#include <RosCom/baxter.h>
#include <Kin/frame.h>

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1){
  if(lowPass.N!=signal.N){ lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

#ifdef MLR_ROS
struct sTaskControlThread{
   Var<sensor_msgs::JointState> jointState = Var<sensor_msgs::JointState>(NULL, "jointState");
};
#else
struct sTaskControlThread{};
#endif

TaskControlThread::TaskControlThread(const char* _robot, const mlr::KinematicWorld& world)
  : Thread("TaskControlThread", .01)
  , s(NULL)
  , taskController(NULL)
  , useRos(false)
  , useSwift(true)
  , requiresInitialSync(true)
  , syncMode(false)
  , verbose(false)
  , useDynSim(true)
  , compensateGravity(false)
  , compensateFTSensors(false)
{

  s = new sTaskControlThread();

  useSwift = mlr::getParameter<bool>("useSwift",true);
  useRos = mlr::getParameter<bool>("useRos",false);
  if(useRos && mlr::checkParameter<bool>("taskControllerNoUseRos"))
      useRos = false;
  useDynSim = !useRos; //mlr::getParameter<bool>("useDynSim", true);
  syncMode = mlr::getParameter<bool>("controller_syncMode", false);
  kp_factor = mlr::getParameter<double>("controller_kp_factor", 1.);
  kd_factor = mlr::getParameter<double>("controller_kd_factor", 1.);
  ki_factor = mlr::getParameter<double>("controller_ki_factor", .2);


  double hyper = mlr::getParameter<double>("hyperSpeed", -1.);
  if(!useRos && hyper>0.){
    this->metronome.reset(.01/hyper);
  }


  robot = mlr::getParameter<mlr::String>("robot");

  //-- deciding on the kinematic model. Priority:
  // 1) an explicit model is given as argument
  // 2) modelWorld has been set before
  // 3) the "robot" flag in cfg file
  // 4) the "robot" argument here


  if(&world) {
    realWorld = world;
    modelWorld.set()() = realWorld;
  } else {
    if(modelWorld.get()->q.N){ //modelWorld has been set before
      realWorld = modelWorld.get();
    }else{
      if(robot=="pr2") {
        realWorld.init(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
      } else if(robot=="baxter") {
        realWorld.init(mlr::mlrPath("data/baxter_model/baxter.ors").p);
      } else {
        HALT("robot not known!")
      }
      modelWorld.set()() = realWorld;
    }
  }

  if(robot!="pr2" && robot!="baxter" && robot!="none") {
    HALT("robot not known!")
  }

  q0 = realWorld.q;

  Kp_base = zeros(realWorld.q.N);
  Kd_base = zeros(realWorld.q.N);
  mlr::Joint *j;
  for(mlr::Frame* f:realWorld.frames) if((j=f->joint) && j->qDim()>0){
    arr *gains = f->ats.find<arr>("gains");
    if(gains){
      for(uint i=0;i<j->qDim();i++){
        Kp_base(j->qIndex+i)=gains->elem(0);
        Kd_base(j->qIndex+i)=gains->elem(1);
      }
    }
  }

  fRInitialOffset = ARR(-0.17119, 0.544316, -1.2, 0.023718, 0.00802182, 0.0095804);
}

TaskControlThread::~TaskControlThread(){
  threadClose();
  if(s) delete s; s=NULL;
}

void TaskControlThread::open(){
  modelWorld.set() = realWorld;
  modelWorld.get()->getJointState(q_model, qdot_model);
  makeConvexHulls(modelWorld.set()->frames);

  taskController = new TaskControlMethods(modelWorld.get());

  if(compensateGravity) {
    gc->learnGCModel();
  }
  if(compensateFTSensors) {
    gc->learnFTModel();
  }

  if(useRos) requiresInitialSync=true;

//    dynSim = new RTControllerSimulation(realWorld, 0.01, false, 0.);
//    dynSim->threadLoop();
}


void TaskControlThread::step(){
  mlr::Frame *transF = realWorld.getFrameByName("worldTranslationRotation", false);
  mlr::Joint *trans = (transF?transF->joint:NULL);

  //-- read real state
  if(useRos){
    bool succ=true;
    if(robot=="pr2"){
      ctrl_obs.waitForRevisionGreaterThan(0);
      pr2_odom.waitForRevisionGreaterThan(0);

      //update q_read from both, ctrl_obs and pr2_odom
      q_real = ctrl_obs.get()->q;
      qdot_real = ctrl_obs.get()->qdot;
      arr pr2odom = pr2_odom.get();
      if(q_real.N==realWorld.q.N && pr2odom.N==3){
        q_real({trans->qIndex, trans->qIndex+2}) = pr2odom;
      }
    }

    if(robot=="baxter"){
#ifdef MLR_ROS
      s->jointState.waitForRevisionGreaterThan(20);
      q_real = realWorld.q;
      succ = baxter_update_qReal(q_real, s->jointState.get(), realWorld);
#endif
      qdot_real = zeros(q_real.N);
    }

    if(robot=="none"){
        if(requiresInitialSync || syncMode){
            q_real = q_model;
            qdot_real = qdot_model;
        }
        requiresInitialSync = false;
    }

    ctrl_q_real.set() = q_real;

    if(succ && q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      realWorld.setJointState(q_real, qdot_real);

      if(requiresInitialSync || syncMode){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
      }
      requiresInitialSync = false;
    }else{
      LOG(-1) <<"** Waiting for ROS message on initial configuration.." <<endl;
      if(step_count>300){
        HALT("sync'ing real robot with simulated failed")
      }
      return;
    }
  }

  double lowPass=.01;
  if(!q_model_lowPass.N) q_model_lowPass = q_model;
  else{ q_model_lowPass *= 1.-lowPass; q_model_lowPass += lowPass * q_model; }

  //-- compute the feedback controller step and iterate to compute a forward reference
  arr CompProj;
//  arr M, F;
  {
    modelWorld.writeAccess();
//    if(!(step_count%20)) modelWorld().gl().update(); //only for debugging

    ctrlTasks.writeAccess();
    taskController->tasks = ctrlTasks();

    taskController->updateCtrlTasks(.01, modelWorld()); //update with time increment

    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;
    if(syncMode){
      dq = taskController->inverseKinematics(qdot_model, q_model_lowPass - q_model);
    }else{
      dq = taskController->inverseKinematics(qdot_model); //don't include a null step
    }
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
    if(cost>1000.){ //reject!
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

  if(CompProj.N){
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
    if (!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }

    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

void TaskControlThread::close(){
  delete taskController;
}
