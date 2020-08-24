/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pr2Interface.h"
#include "../RosCom/roscom.h"
#include "../RosCom/rosmacro.h"
#include "../Core/thread.h"
#include "../Gui/opengl.h"
#include "../Core/util.h"
#include <stdlib.h>

PR2Interface::PR2Interface() : Thread("PR2_Interface") {
  this->useROS = rai::getParameter<bool>("useRos", false);
}

void PR2Interface::step() {
  arr qRealWorld, qDotRealWorld;

  this->ctrl_obs.waitForNextRevision();
  CtrlMsg actMsg = ctrl_obs.get();
  qRealWorld = actMsg.q;
  qDotRealWorld = actMsg.qdot;

  AlvarMarkers alvarMarkers = this->ar_pose_markers.get();
  syncMarkers(*this->modelWorld, alvarMarkers);
  syncMarkers(*this->realWorld, alvarMarkers);

  this->realWorld->setJointState(qRealWorld, qDotRealWorld);
  this->realWorld->watch(false);

  arr qModelWorld, qDotModelWorld;

  transferQbetweenTwoWorlds(qModelWorld, qRealWorld, *this->modelWorld, *this->realWorld);
  transferQDotbetweenTwoWorlds(qDotModelWorld, qDotRealWorld, *this->modelWorld, *this->realWorld);

  this->modelWorld->setJointState(qModelWorld, qDotModelWorld);
  this->modelWorld->watch(false);

  if(this->controller->taskSpaceAccLaws.N < 1) {
    Feature* qItselfTask = new TM_qItself();
    LinTaskSpaceAccLaw* qItselfLaw = new LinTaskSpaceAccLaw(qItselfTask, this->modelWorld, "idle");
    qItselfLaw->setC(eye(qItselfLaw->getPhiDim())*10.0);
    qItselfLaw->setGains(eye(qItselfLaw->getPhiDim())*10.0, eye(qItselfLaw->getPhiDim())*1.0);
    qItselfLaw->setRef(qItselfLaw->getPhi(), zeros(qItselfLaw->getPhiDim()));
    this->controller->addLinTaskSpaceAccLaw(qItselfLaw);
  }

  //TODO if there is no law, maybe a jointSpace law should be there with low gains?
  arr u0, Kp, Kd;
  this->controller->calcOptimalControlProjected(Kp, Kd, u0); // TODO: what happens when changing the LAWs?

  arr K_ft, J_ft_inv, fRef;
  double gamma;
  this->controller->calcForceControl(K_ft, J_ft_inv, fRef, gamma);

  this->sendCommand(u0, Kp, Kd, K_ft, J_ft_inv, fRef, gamma);

  //cout << actMsg.fL(2) << endl;

  if(this->logState) {
    this->logT.append(rai::timerRead());
    this->logQObs.append(~qRealWorld);
    this->logQDotObs.append(~qDotRealWorld);
    this->logFLObs.append(~actMsg.fL);
    this->logFRObs.append(~actMsg.fR);
    this->logUObs.append(~actMsg.u_bias);
    for(LinTaskSpaceAccLaw* law : this->controller->taskSpaceAccLaws) {
      arr y, J, yDot, q, qDot;
      law->world->getJointState(q, qDot);
      law->getPhi(y, J);
      yDot = J*qDot;
      this->logMap[STRING(law->name << "Obs")].append(~y);
      this->logMap[STRING(law->name << "Ref")].append(~law->getRef());
      this->logMap[STRING(law->name << "DotObs")].append(~yDot);
      this->logMap[STRING(law->name << "DotRef")].append(~law->getDotRef());
      this->logMap[STRING(law->name << "Kp")].append(law->getKp());
      this->logMap[STRING(law->name << "Kd")].append(law->getKd());
      this->logMap[STRING(law->name << "C")].append(law->getC());
    }
    for(ConstrainedTaskLaw* law : this->controller->constrainedTaskLaws) {
      this->logMap[STRING(law->name << "Alpha")].append(law->getAlpha());
      this->logMap[STRING(law->name << "Force")].append(law->getForce());
      this->logMap[STRING(law->name << "Gamma")].append(law->getGamma());
      //TODO log FObs in TaskSpace
    }
  }
}

void PR2Interface::initialize(rai::Configuration* realWorld, rai::Configuration* realWorldSimulation, rai::Configuration* modelWorld, TaskSpaceController* controller) {

  cout << "TODO: nochmal eine World mehr" << endl;

  this->controller = controller;

  this->modelWorld = modelWorld;
  this->modelWorld->meldFixedJoints();
  this->modelWorld->gl().title = "Model World";
  this->modelWorld->watch(false);

  if(this->useROS) {
    this->realWorld = realWorld;
    this->realWorld->gl().title = "Real World";
    cout << "Trying to connect to PR2" << endl;

    rosCheckInit();
    new RosCom_Spinner();
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    new Subscriber<AlvarMarkers>("/ar_pose_marker", (Var<AlvarMarkers>&)ar_pose_markers);
    threadOpenModules(true);

    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    while(true) {
      this->ctrl_obs.data->waitForNextRevision(); // TODO why .var???
      cout << "REMOTE joint dimension = " << this->ctrl_obs.get()->q.N << endl;
      cout << "LOCAL  joint dimension = " << this->realWorld->q.N << endl;

      if(this->ctrl_obs.get()->q.N == this->realWorld->q.N && this->ctrl_obs.get()->qdot.N == this->realWorld->q.N) {
        cout << "Syncing successfull :-)" << endl;
        break;
      }
    }

    //TODO initMsg should contain gains?
    CtrlMsg initMsg;
    initMsg.fL = zeros(1);
    initMsg.KiFT.clear();
    initMsg.J_ft_inv.clear();
    initMsg.u_bias = zeros(this->realWorld->getJointStateDimension());

    /*arr Kp_base = zeros(this->realWorld->getJointStateDimension());
    arr Kd_base = zeros(this->realWorld->getJointStateDimension());
    for_list(rai::Joint, j, this->realWorld->joints) if(j->qDim()>0){
      arr *info;
      info = j->ats.getValue<arr>("gains");
      if(info){
        Kp_base(j->qIndex)=info->elem(0);
        Kd_base(j->qIndex)=info->elem(1);
      }
    }*/

    initMsg.Kp = ARR(0.0);
    initMsg.Kd = ARR(0.0);
    initMsg.Ki = ARR(0.0);
    initMsg.gamma = 1.;
    initMsg.velLimitRatio = .1;
    initMsg.effLimitRatio = 1.;
    initMsg.intLimitRatio = 0.8;

    initMsg.q = this->realWorld->getJointState();
    initMsg.qdot = zeros(this->realWorld->getJointStateDimension());

    this->ctrlMsg = initMsg;

    this->ctrl_ref.set() = ctrlMsg;

  } else {
    this->realWorld = realWorldSimulation;
    this->realWorld->gl().title = "Real World Simulated";
    this->dynamicSimulation = new DynamicSimulation();
    threadOpenModules(true);

    CtrlMsg initMsg;
    initMsg.fL = zeros(1);
    initMsg.KiFT.clear();
    initMsg.J_ft_inv.clear();
    initMsg.u_bias = zeros(this->realWorld->getJointStateDimension());
    initMsg.Kp = ARR(0.0);
    initMsg.Kd = ARR(0.0);
    initMsg.Ki = ARR(0.0);
    initMsg.gamma = 1.;
    initMsg.velLimitRatio = .1;
    initMsg.effLimitRatio = 1.;
    initMsg.intLimitRatio = 0.8;

    initMsg.q = this->realWorld->getJointState();
    initMsg.qdot = zeros(this->realWorld->getJointStateDimension());

    this->ctrlMsg = initMsg;

    this->ctrl_ref.set() = ctrlMsg;

    this->dynamicSimulation->initializeSimulation(new rai::Configuration(*this->realWorld));
    this->dynamicSimulation->startSimulation();
  }
  this->realWorld->watch(false);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%           init PR2 interface           %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}

void PR2Interface::initialize(rai::Configuration* realWorld, rai::Configuration* modelWorld, TaskSpaceController* controller) {
  this->initialize(realWorld, realWorld, modelWorld, controller);
}

void PR2Interface::startInterface() {
  this->realWorld->watch(true, "Press Enter to start everything :-) :-) :-)");
  this->realWorld->watch(false, "");

  this->threadLoop();
  rai::timerStart(true);
  rai::wait(1.0);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%        start PR2 interface             %" << endl;
  cout << "%           and LOOOOOOP!                %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}

void PR2Interface::sendCommand(const arr& u0, const arr& Kp, const arr& Kd, const arr& K_ft, const arr& J_ft_inv, const arr& fRef, const double& gamma) {
  arr u0RealWorld, KpRealWorld, KdRealWorld, qRefRealWorld, qDotRefRealWorld;
  transferU0BetweenTwoWorlds(u0RealWorld, u0, *this->realWorld, *this->modelWorld);
  transferKpBetweenTwoWorlds(KpRealWorld, Kp, *this->realWorld, *this->modelWorld);
  transferKdBetweenTwoWorlds(KdRealWorld, Kd, *this->realWorld, *this->modelWorld);
  transferQbetweenTwoWorlds(qRefRealWorld, zeros(modelWorld->getJointStateDimension()), *this->realWorld, *this->modelWorld);
  transferQDotbetweenTwoWorlds(qDotRefRealWorld, zeros(modelWorld->getJointStateDimension()), *this->realWorld, *this->modelWorld);

  if(this->torsoLiftRef.N == 1) {
    qRefRealWorld(this->realWorld->getJointByName("torso_lift_joint")->qIndex) = this->torsoLiftRef(0);
  }

  if(this->lGripperRef.N == 1) {
    qRefRealWorld(this->realWorld->getJointByName("l_gripper_joint")->qIndex) = this->lGripperRef(0);
  }

  if(this->rGripperRef.N == 1) {
    qRefRealWorld(this->realWorld->getJointByName("r_gripper_joint")->qIndex) = this->rGripperRef(0);
  }

  arr KiFtRealWorld;
  if(!!K_ft && &J_ft_inv && &fRef) {
    transferKI_ft_BetweenTwoWorlds(KiFtRealWorld, K_ft, *this->realWorld, *this->modelWorld);
    this->ctrlMsg.KiFT = KiFtRealWorld;
    this->ctrlMsg.J_ft_inv = J_ft_inv;
    this->ctrlMsg.fL = fRef;
  }

  if(!!gamma) {
    this->ctrlMsg.gamma = gamma;
  } else {
    this->ctrlMsg.gamma = 0.0;
  }

  this->ctrlMsg.u_bias = u0RealWorld;
  this->ctrlMsg.Kp = KpRealWorld;
  this->ctrlMsg.Kd = KdRealWorld;
  this->ctrlMsg.q = qRefRealWorld;
  this->ctrlMsg.qdot = qDotRefRealWorld;

  //this->ctrl_ref.writeAccess();
  this->ctrl_ref.set() = ctrlMsg;
  //this->ctrl_ref.deAccess();

  if(logState) {
    this->logU0.append(~u0RealWorld);
    this->logKp.append(KpRealWorld);
    this->logKd.append(KdRealWorld);
    this->logKiFt.append(~KiFtRealWorld); //TODO what if matrix is not 1xqDim dimensional?
    this->logQRef.append(~qRefRealWorld);
    this->logQDotRef.append(~qDotRefRealWorld);
    this->logJ_ft_inv.append(J_ft_inv);
    this->logFRef.append(~fRef);
  }
}

void PR2Interface::goToPosition(arr pos, rai::String shape, double executionTime, bool useMotionPlaner, rai::String name) {
  Feature* posMap = new TM_Default(TMT_pos, *this->modelWorld, shape);
  this->goToTask(posMap, pos, executionTime, useMotionPlaner, name);
}

void PR2Interface::goToTasks(rai::Array<LinTaskSpaceAccLaw*> laws, double executionTime, bool useMotionPlanner) {
  if(useMotionPlanner) {
    rai::Configuration copiedWorld(*this->modelWorld);
    KOMO MP(copiedWorld);

    MP.x0 = modelWorld->getJointState(); //TODO nix modelWorld, copiedWorld?

    Task* t;
    t = MP.addTask("transitions", new TM_Transition(MP.world));
    t->feat.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e0);

    t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
    t->setCostSpecs(0., MP.T, {0.}, 1.0);
    t = MP.addTask("qLimits", new LimitsConstraint());
    t->setCostSpecs(0., MP.T, {0.}, 1.);

    for(LinTaskSpaceAccLaw* law : laws) {
      t = MP.addTask(law->name, law->feat);
      t->setCostSpecs(MP.T-2, MP.T, law->getRef(), 10.0);
    }

    MotionProblemFunction MF(MP);

    arr traj = MP.getInitialization();

    OptOptions o;
    o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;

    optConstrained(traj, NoArr, Convert(MF), o); //TODO: which options?
    //OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2)

    //MP.costReport(); //TODO: make this parameter

    showTrajectory(traj, *this->modelWorld);

    Feature* qTask = new TM_qItself();
    LinTaskSpaceAccLaw* qLaw = new LinTaskSpaceAccLaw(qTask, this->modelWorld, "qLaw");
    qLaw->setC(eye(this->modelWorld->getJointStateDimension())*1000.0);
    qLaw->setGains(eye(this->modelWorld->getJointStateDimension())*25.0, eye(this->modelWorld->getJointStateDimension())*5.0);
    qLaw->setTrajectory(traj.d0, traj);

    this->controller->taskSpaceAccLaws.clear();
    controller->addLinTaskSpaceAccLaw(qLaw);
    controller->generateTaskSpaceSplines();
    this->executeTrajectory(executionTime);
    rai::wait(0.5);
  } else {
    NIY;
  }
}

void PR2Interface::goToJointState(arr jointState, double executionTime, bool useMotionPlaner, rai::String name) {
  Feature* qTask = new TM_qItself();
  this->goToTask(qTask, jointState, executionTime, useMotionPlaner, name);
}

void PR2Interface::goToTask(Feature* map, arr ref, double executionTime, bool useMotionPlaner, rai::String name) {
  LinTaskSpaceAccLaw* law = new LinTaskSpaceAccLaw(map, this->modelWorld, name);
  law->setRef(ref);
  rai::Array<LinTaskSpaceAccLaw*> laws;
  laws.append(law);
  this->goToTasks(laws, executionTime, useMotionPlaner);
}

void PR2Interface::executeTrajectory(double executionTime) {
  cout << "start executing trajectory" << endl;
  //rai::timerStart(true);
  double startTime = rai::timerRead();
  double time = 0.0;
  uint n = 0;

  while(true) {
    double s;
    if(time < executionTime) {
      s = time/executionTime;
    } else {
      s = 1;
      cout << "finished execution of trajectory" << endl;
      //logStateSave();
      break;
    }

    for(LinTaskSpaceAccLaw* law : this->controller->taskSpaceAccLaws) {
      law->setTargetEvalSpline(s);
    }
    n++;
    time = rai::timerRead() - startTime;
  }
}

void PR2Interface::moveTorsoLift(arr torsoLiftRef) {
  this->torsoLiftRef = torsoLiftRef;
}

void PR2Interface::moveLGripper(arr lGripperRef) {
  this->lGripperRef = lGripperRef;
}

void PR2Interface::moveRGripper(arr rGripperRef) {
  this->rGripperRef = rGripperRef;
}

void PR2Interface::logStateSave(rai::String name, rai::String folder) {
  cout << "start saving log " << name << endl;
  system(STRING("mkdir -p " << folder << "/" << name)); //linux specific :-)

  if(this->logT.N) write(LIST<arr>(this->logT), STRING(folder << "/" << name << "/" << "T" << ".dat"));
  if(this->logQObs.N) write(LIST<arr>(this->logQObs), STRING(folder << "/" << name << "/" << "qObs" << ".dat"));
  if(this->logQRef.N) write(LIST<arr>(this->logQRef), STRING(folder << "/" << name << "/" << "qRef" << ".dat"));
  if(this->logQDotObs.N)write(LIST<arr>(this->logQDotObs), STRING(folder << "/" << name << "/" << "qDotObs" << ".dat"));
  if(this->logQDotRef.N)write(LIST<arr>(this->logQDotRef), STRING(folder << "/" << name << "/" << "qDotRef" << ".dat"));
  if(this->logUObs.N)write(LIST<arr>(this->logUObs), STRING(folder << "/" << name << "/" << "uObs" << ".dat"));
  if(this->logU0.N)write(LIST<arr>(this->logU0), STRING(folder << "/" << name << "/" << "u0" << ".dat"));
  if(this->logKp.N)write(LIST<arr>(this->logKp), STRING(folder << "/" << name << "/" << "Kp" << ".dat"));
  if(this->logKd.N)write(LIST<arr>(this->logKd), STRING(folder << "/" << name << "/" << "Kd" << ".dat"));
  if(this->logKiFt.N)write(LIST<arr>(this->logKiFt), STRING(folder << "/" << name << "/" << "KiFt" << ".dat"));
  if(this->logJ_ft_inv.N)write(LIST<arr>(this->logJ_ft_inv), STRING(folder << "/" << name << "/" << "J_ft_inv" << ".dat"));
  if(this->logFRef.N)write(LIST<arr>(this->logFRef), STRING(folder << "/" << name << "/" << "fRef" << ".dat"));
  if(this->logFLObs.N)write(LIST<arr>(this->logFLObs), STRING(folder << "/" << name << "/" << "fLObs" << ".dat"));
  if(this->logFRObs.N)write(LIST<arr>(this->logFRObs), STRING(folder << "/" << name << "/" << "fRObs" << ".dat"));

  //TODO: log constrained law, log gamma, alpha etc.

  for(auto m : this->logMap) {
    write(LIST<arr>(m.second), STRING(folder << "/" << name << "/" << m.first << ".dat"));
  }
  cout << "finished saving log " << name << endl;
}

void PR2Interface::clearLog() {
  this->logT.clear();
  this->logQObs.clear();
  this->logQRef.clear();
  this->logQDotObs.clear();
  this->logQDotRef.clear();
  this->logUObs.clear();
  this->logU0.clear();
  this->logKp.clear();
  this->logKd.clear();
  this->logKiFt.clear();
  this->logJ_ft_inv.clear();
  this->logFRef.clear();
  this->logFLObs.clear();
  this->logFRObs.clear();
  logMap.clear();
  cout << "cleared logging" << endl;
}

REGISTER_MODULE(PR2Interface)

void showTrajectory(const arr& traj, rai::Configuration& _world, bool copyWorld, double delay, rai::String text) {
  rai::Configuration* world;
  if(copyWorld) {
    world = new rai::Configuration(_world);
  } else {
    world = &_world;
  }

  world->watch(true, STRING("Show trajectory. " << text << " "));
  for(uint i = 0; i < traj.d0; i++) {
    world->setJointState(traj[i]);
    world->watch(false);
    rai::wait(delay);
  }
  world->watch(true, "Finished. ");
  if(copyWorld) {
    delete world;
  }
}
