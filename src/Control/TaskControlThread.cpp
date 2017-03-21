#include "TaskControlThread.h"
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1){
  if(lowPass.N!=signal.N){ lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

#ifdef MLR_ROS
struct sTaskControlThread{
   ACCESSname(sensor_msgs::JointState, jointState)
};
#else
struct sTaskControlThread{};
#endif

TaskControlThread::TaskControlThread(const char* _robot, const mlr::KinematicWorld& world)
  : Thread("TaskControlThread", .01)
  , s(NULL)
  , taskController(NULL)
  , useRos(false)
  , requiresInitialSync(true)
  , syncModelStateWithReal(false)
  , verbose(false)
  , useDynSim(true)
  , compensateGravity(false)
  , compensateFTSensors(false)
{

  s = new sTaskControlThread();
  useRos = mlr::getParameter<bool>("useRos",false);
  useDynSim = !useRos; //mlr::getParameter<bool>("useDynSim", true);

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
  static uint t=0;
  t++;

  mlr::Joint *trans = realWorld.getJointByName("worldTranslationRotation", false);
  bool resetCtrlTasksState = false;

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
        if(requiresInitialSync){ //this was the first initial sync!! -> reset ctrl tasks
          ctrlTasks.writeAccess();
          if(resetCtrlTasksState) taskController->resetCtrlTasksState();
          ctrlTasks.deAccess();
        }
        syncModelStateWithReal = false;
      }
      requiresInitialSync = false;
    }else{
      LOG(-1) <<"** Waiting for ROS message on initial configuration.." <<endl;
      if(t>300){
        HALT("sync'ing real robot with simulated failed")
      }
      return;
    }
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  {
    modelWorld.writeAccess();
//    if(!(step_count%20)) modelWorld().gl().update(); //only for debugging

    ctrlTasks.writeAccess();
    taskController->tasks = ctrlTasks();

    taskController->updateCtrlTasks(.01, modelWorld()); //update with time increment

    //-- compute IK step
    double maxQStep = 1e-1;
    arr dq = taskController->inverseKinematics(qdot_model); //, .01*(q0-q_model));
    double l = length(dq);
    if(l>maxQStep) dq *= maxQStep/l;
    q_model += dq;

    //set/test the new configuration
    modelWorld().setJointState(q_model, qdot_model);
    modelWorld().stepSwift();
    taskController->updateCtrlTasks(0., modelWorld()); //update without time increment
    double cost = taskController->getIKCosts();
    IK_cost.set() = cost;

    //check the costs
    if(cost>1000.){ //reject!
      LOG(-1) <<"HIGH COST IK! " <<cost;
      q_model -= .9*dq;
      modelWorld().setJointState(q_model, qdot_model);
      modelWorld().stepSwift();
      taskController->updateCtrlTasks(0., modelWorld()); //update without time increment
    }

    if(verbose) taskController->reportCurrentState();
    ctrlTasks.deAccess();
    modelWorld.deAccess();
  }

  //TODO: construct the compliance matrix..

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
    refs.Kp = ARR(1.);
    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.2);
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
