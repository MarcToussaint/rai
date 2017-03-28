#include "TaskControlThread.h"
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1){
  if(lowPass.N!=signal.N){ lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

#ifdef MLR_ROS
struct sTaskControlThread{
   Access<sensor_msgs::JointState> jointState = Access<sensor_msgs::JointState>(NULL, "jointState");
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
  , syncModelStateWithReal(false)
  , verbose(false)
  , useDynSim(true)
  , compensateGravity(false)
  , compensateFTSensors(false)
{

  s = new sTaskControlThread();

  useSwift = mlr::getParameter<bool>("useSwift",true);
  useRos = mlr::getParameter<bool>("useRos",false);
  useDynSim = !useRos; //mlr::getParameter<bool>("useDynSim", true);

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

  if(robot != "pr2" && robot != "baxter") {
    HALT("robot not known!")
  }

  q0 = realWorld.q;

  Kp_base = zeros(realWorld.q.N);
  Kd_base = zeros(realWorld.q.N);
  for(mlr::Joint* j:realWorld.joints) if(j->qDim()>0){
    arr *gains = j->ats.find<arr>("gains");
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
  makeConvexHulls(modelWorld.set()->shapes);

  taskController = new TaskControlMethods(modelWorld.get());

  if(compensateGravity) {
    gc->learnGCModel();
  }
  if(compensateFTSensors) {
    gc->learnFTModel();
  }

  if(useRos) syncModelStateWithReal=true;

//    dynSim = new RTControllerSimulation(realWorld, 0.01, false, 0.);
//    dynSim->threadLoop();
}


void TaskControlThread::step(){
  mlr::Joint *trans = realWorld.getJointByName("worldTranslationRotation", false);

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

    ctrl_q_real.set() = q_real;

    if(succ && q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      realWorld.setJointState(q_real, qdot_real);

      if(syncModelStateWithReal){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        syncModelStateWithReal = false; //continue sync'n; or only once?
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

  //-- compute the feedback controller step and iterate to compute a forward reference
  arr CompProj;
  {
    modelWorld.writeAccess();
//    if(!(step_count%20)) modelWorld().gl().update(); //only for debugging

    ctrlTasks.writeAccess();
    taskController->tasks = ctrlTasks();

    taskController->updateCtrlTasks(.01, modelWorld()); //update with time increment

    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;
    if(syncModelStateWithReal){
      arr nullStep = .1 * (q0-q_model); //this is the 'null step' (point of zero-cost)
      double l = length(nullStep);
      if(l>maxQStep) nullStep *= maxQStep/l;
      dq = taskController->inverseKinematics(qdot_model, nullStep);
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

    ctrlTasks.deAccess();
    modelWorld.deAccess();
  }

  //project gains if there are compliance tasks
  arr Kp = diag(Kp_base);
  arr Kd = diag(Kd_base);
  if(CompProj.N){
    Kp = CompProj*Kp*CompProj;
    Kd = CompProj*Kd*CompProj;
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
    refs.Ki = ARR(.0); //ARR(0.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_model.N);
    refs.intLimitRatio = 0.7;
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
