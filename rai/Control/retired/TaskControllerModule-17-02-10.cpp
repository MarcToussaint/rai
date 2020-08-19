/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include "../Gui/opengl.h"
#include "../RosCom/baxter.h"

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1) {
  if(lowPass.N!=signal.N) { lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

#ifdef RAI_ROS
struct sTaskControlThread {
  VARname(sensor_msgs::JointState, jointState)
};
#else
struct sTaskControlThread {};
#endif

TaskControlThread::TaskControlThread(const char* _robot, const rai::Configuration& world)
  : Thread("TaskControlThread", .01)
  , s(nullptr)
  , taskController(nullptr)
  , oldfashioned(true)
  , useRos(false)
  , requiresInitialSync(true)
  , syncModelStateWithReal(false)
  , verbose(false)
  , useDynSim(true)
  , compensateGravity(false)
  , compensateFTSensors(false) {

  self = make_unique<sTaskControlThread>();
  useRos = rai::getParameter<bool>("useRos", false);
  oldfashioned = rai::getParameter<bool>("oldfashinedTaskControl", true);
  useDynSim = !oldfashioned && !useRos; //rai::getParameter<bool>("useDynSim", true);

  robot = rai::getParameter<rai::String>("robot");

  //-- deciding on the kinematic model. Priority:
  // 1) an explicit model is given as argument
  // 2) modelWorld has been set before
  // 3) the "robot" flag in cfg file
  // 4) the "robot" argument here

  if(!!world) {
    realWorld = world;
    modelWorld.set()() = realWorld;
  } else {
    if(modelWorld.get()->q.N) { //modelWorld has been set before
      realWorld = modelWorld.get();
    } else {
      if(robot=="pr2") {
        realWorld.init(rai::raiPath("data/pr2_model/pr2_model.g").p);
      } else if(robot=="baxter") {
        realWorld.init(rai::raiPath("data/baxter_model/baxter.g").p);
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

  qSign.set()() = zeros(q0.N);

  fRInitialOffset = ARR(-0.17119, 0.544316, -1.2, 0.023718, 0.00802182, 0.0095804);

}

TaskControlThread::~TaskControlThread() {
}

void changeColor(void*) {  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2(void*) {  orsDrawColors=true; orsDrawAlpha=1.; }

void TaskControlThread::open() {
  //gc = new GravityCompensation(realWorld);
  if(compensateGravity) {
    //gc->loadBetas();
    gc->learnGCModel();
  }
  if(compensateFTSensors) {
    gc->learnFTModel();
  }

  modelWorld.set()() = realWorld;

  makeConvexHulls(modelWorld.set()->shapes);
// modelWorld.set() = realWorld;
  taskController = new TaskControlMethods(modelWorld.set()(), false);

  modelWorld.get()->getJointState(q_model, qdot_model);

  taskController->qNullCostRef.y_ref = q0;
  taskController->qNullCostRef.setGains(0., 1.);
  taskController->qNullCostRef.prec = rai::getParameter<double>("Hrate", .1)*modelWorld.get()->getHmetric();

#if 1
  modelWorld.writeAccess();
  modelWorld().gl().add(changeColor);
  modelWorld().gl().add(rai::glDrawGraph, &realWorld);
  modelWorld().gl().add(changeColor2);
  modelWorld.deAccess();
#endif

  if(useRos || !oldfashioned) syncModelStateWithReal=true;

  if(!oldfashioned && !useRos) {
    dynSim = new RTControllerSimulation(realWorld, 0.01, false, 0.);
    dynSim->threadLoop();
  }
}

void TaskControlThread::step() {
  static uint t=0;
  t++;

  rai::Joint* trans= realWorld.getJointByName("worldTranslationRotation", false);

  //-- read real state
  if(useRos || !oldfashioned) {
    bool succ=true;
    qdot_last = qdot_real;
    if(robot=="pr2") {
      ctrl_obs.waitForRevisionGreaterThan(0);
      if(useRos)  pr2_odom.waitForRevisionGreaterThan(0);
      q_real = ctrl_obs.get()->q;
      qdot_real = ctrl_obs.get()->qdot;
      arr pr2odom = pr2_odom.get();
      if(q_real.N==realWorld.q.N && pr2odom.N==3) {
        q_real({trans->qIndex, trans->qIndex+2}) = pr2odom;
      }

      if(qLastReading.d0 > 0) {
        qSign.writeAccess();
        for(uint i = 0; i < q_real.N; i++) {
          double si = sign(q_real(i)-qLastReading(i));
          if(si != qSign()(i) && si != 0) {
            qSign()(i) = si;
          }
        }
        //cout << qSign() << endl;
        qSign.deAccess();
      }
      qLastReading = q_real;
    }
    if(robot=="baxter" && useRos) {
#ifdef RAI_ROS
      self->jointState.waitForRevisionGreaterThan(20);
      q_real = realWorld.q;
      succ = baxter_update_qReal(q_real, self->jointState.get(), realWorld);
#endif
      qdot_real = zeros(q_real.N);
    }

    ctrl_q_real.set() = q_real;

    if(succ && q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N) { //we received a good reading
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithReal) {
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        q_history.prepend(q_real); q_history.reshape(q_history.N/q_real.N, q_real.N);
        if(q_history.d0>5) q_history.resizeCopy(5, q_real.N);

        if(q_history.d0>0) lowPassUpdate(q_lowPass, q_history[0]);
        if(q_history.d0>1) lowPassUpdate(qdot_lowPass, (q_history[0]-q_history[1])/.01);
        if(q_history.d0>2) lowPassUpdate(qddot_lowPass, (q_history[0]-2.*q_history[1]+q_history[2])/(.01*.01));
        //if(q_history.d0 > 1) cout << sign(q_model-q_history[1]) << endl;
        if(oldfashioned) syncModelStateWithReal = false;
      }
      requiresInitialSync = false;
    } else {
      cout <<"** Waiting for ROS message on initial configuration.." <<endl;
      if(t>10000) {
        HALT("sync'ing real robot with simulated failed")
      }
    }
  }

  //-- sync the model world with the AlvarMarkers
//  modelWorld.writeAccess();
//  AlvarMarkers alvarMarkers = ar_pose_marker.get();
//  syncMarkers(modelWorld(), alvarMarkers);
//  syncMarkers(realWorld, alvarMarkers);
//  modelWorld.deAccess();

  //-- display the model world (and in same gl, also the real world)
  if(!(t%5)) {
#if 1
    //modelWorld.set()->watch(false, STRING("model world state t="<<(double)t/100.));
#endif
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  CtrlMsg refs;
  if(oldfashioned) {

    //now operational space control
    modelWorld.writeAccess();
    ctrlTasks.readAccess();
    taskController->tasks = ctrlTasks();
#if 0
    for(uint tt=0; tt<10; tt++) {
      arr a = taskController->operationalSpaceControl();
      q_model += .001*qdot_model;
      qdot_model += .001*a;
      if(trans && fixBase.get()) {
        qdot_model(trans->qIndex+0) = 0;
        qdot_model(trans->qIndex+1) = 0;
        qdot_model(trans->qIndex+2) = 0;
      }
      taskController->setState(q_model, qdot_model);
    }
#else
    arr a = taskController->operationalSpaceControl();
    double tau = .01;
    q_model = modelWorld().q + tau*qdot_model + (.5*tau*tau)*a;
    qdot_model = modelWorld().qdot + tau*a;
    if(trans && fixBase.get()) {
      qdot_model(trans->qIndex+0) = 0.;
      qdot_model(trans->qIndex+1) = 0.;
      qdot_model(trans->qIndex+2) = 0.;
    }
    taskController->setState(q_model, qdot_model);
#endif
    if(verbose) taskController->reportCurrentState();
    ctrlTasks.deAccess();
    modelWorld.deAccess();

    //arr Kp, Kd, k, JCJ;
    //taskController->getDesiredLinAccLaw(Kp, Kd, k, JCJ);

    //Kp = .01 * JCJ;
    //Kp += .2*diag(ones(Kp.d0));

    ctrl_q_ref.set() = q_model;

    //-- first zero references
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

    //-- compute the force feedback control coefficients
    uint count=0;
    ctrlTasks.readAccess();
    taskController->tasks = ctrlTasks();
    for(CtrlObjective* t : taskController->tasks) {
      if(t->active && t->f_ref.N) {
        count++;
        if(count!=1) HALT("you have multiple active force control tasks - NIY");
        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
      }
    }
    if(count==1) refs.Kp = .5;
    ctrlTasks.deAccess();

  } else {

    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    taskController->tasks = ctrlTasks();

    modelWorld().stepSwift();

#if 0
    arr u_bias, Kp, Kd;
    arr M, F;
    taskController->world.equationOfMotion(M, F, false);
    arr u_mean = taskController->calcOptimalControlProjected(Kp, Kd, u_bias, M, F); // TODO: what happens when changing the LAWs?
    arr q_ref = zeros(q_model.N);
#else

    //-- compute desired acceleration law in q-space
    arr a, Kp, Kd, k;
    a = taskController->getDesiredLinAccLaw(Kp, Kd, k);
    checkNan(k);

    //-- translate to motor torques
    arr M, F;
    taskController->world.equationOfMotion(M, F, false);
#if 0 //-- limit the step and use q_ref? works only if Kp is invertible!!!!!
//    Kp += diag(1e-6, Kp.d0); //TODO: Danny removes this ;-) (z
    arr q_step = pseudoInverse(Kp)*(k-Kp*q_real);
    clip(q_step, -.1, .1);
    arr q_ref = q_real + q_step;
    arr u_bias = zeros(q_model.N);
#else //... or directly u_bias
    arr q_ref = zeros(q_model.N);
    arr u_bias = M*k + zeros(q_model.N); //+F returns nans TODO
#endif
    Kp = M*Kp;
    Kd = M*Kd;
    checkNan(Kp);

    //-- compute the error between expected change in velocity and true one
#if 0
    if(!a_last.N) a_last = a;
    if(!qdot_last.N) qdot_last = qdot_real;
    arr a_err = (qdot_real - qdot_last)/.01 - a_last;
    a_last = a;
    // integrate this error
    if(!aErrorIntegral.N) aErrorIntegral = JCJ * a_err;
    else aErrorIntegral += a_err;
    // add integral error to control bias
    u_bias -= .01 * M * aErrorIntegral;
#endif
#endif

    // F/T limit control
    arr K_ft, J_ft_inv, fRef;
    double gamma;
    taskController->calcForceControl(K_ft, J_ft_inv, fRef, gamma);

    if(verbose) {
      LOG(0) <<"************** Tasks Report **********";
      taskController->reportCurrentState();
    }

    modelWorld.deAccess();
    ctrlTasks.deAccess();

    refs.q =  q_ref;
    refs.qdot = zeros(q_model.N);
    refs.fR_gamma = gamma;
    refs.Kp = Kp;
    refs.Kd = Kd;
    refs.Ki.clear();
    refs.fL = zeros(6);//fRef;
    refs.fR = fRef;
    refs.KiFTR = K_ft;
    refs.J_ft_invR = J_ft_inv;

    if(compensateGravity) {
      //u_bias += gc->compensate(realWorld.getJointState(),{"l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint"
      // ,"l_wrist_flex_joint"});
      //u_bias += gc->compensate(realWorld.getJointState(), qSign.get()(),{"l_shoulder_pan_joint","l_shoulder_lift_joint","l_forearm_roll_joint","l_wrist_flex_joint"});
      u_bias += gc->compensate(realWorld.getJointState(), qSign.get()(), {"r_shoulder_pan_joint"
                               , "r_shoulder_lift_joint"
                               , "r_forearm_roll_joint"
                               , "r_wrist_flex_joint"
                                                                         });
    }
    refs.u_bias = u_bias;

    if(compensateFTSensors) {
      refs.fL_offset = gc->compensateFTL(realWorld.getJointState());
      refs.fR_offset = gc->compensateFTR(realWorld.getJointState()) + fRInitialOffset;
    } else {
      refs.fL_offset = zeros(6);
      refs.fR_offset = zeros(6);
    }

    refs.intLimitRatio = 0.7;
    refs.qd_filt = .99;
  }

  ctrl_q_ref.set() = refs.q;

  //-- send base motion command
  if(useRos) {
    if(!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }
  }

  //-- send the computed movement to the robot
  if(!requiresInitialSync && (useRos || useDynSim)) {
    ctrl_ref.set() = refs;
  }
}

void TaskControlThread::close() {
  delete taskController;
}
