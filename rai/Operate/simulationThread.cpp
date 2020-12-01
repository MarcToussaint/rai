/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "simulationThread.h"
#ifdef RAI_ROS
#  include <RosCom/roscom.h>
#endif
//#include "SimulationThread_self.h"

struct SimulationThread_self {
#ifdef RAI_ROS
  //inputs (reactive variables / messages)
  Var<rai_msgs::MotionReference> ref; // communication variable with ROS
  Var<rai_msgs::StringA> command;

  //outputs
  Var<rai_msgs::arr> currentQ;
  Var<rai_msgs::arr> objectPoses;
  Var<rai_msgs::StringA> objectNames;

  RosCom ROS;
  std::shared_ptr<Subscriber<rai_msgs::MotionReference>> sub_ref; //subscriber
  std::shared_ptr<Subscriber<rai_msgs::StringA>> sub_command; //subscriber
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_currentQ;
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_objectPoses;
  std::shared_ptr<Publisher<rai_msgs::StringA>> pub_objectNames;
  std::shared_ptr<PublisherConv<std_msgs::Float64, double, conv_double2Float64>> pub_timeToGo;

  SimulationThread_self()
    : ref(),
      command(),
      currentQ(),
      objectPoses(),
      objectNames(),
      ROS("simulator") {

    //setup ros communication
    sub_ref = ROS.subscribe(ref);
    sub_command = ROS.subscribe(command);

    pub_objectPoses = ROS.publish(objectPoses);
    pub_currentQ = ROS.publish(currentQ);
  }
#endif
};

SimulationThread::SimulationThread(const rai::Configuration& _K, double dt, bool _pubSubToROS)
  : Thread("SimulationIO", dt),
    SIM(_K, dt),
    pubSubToROS(_pubSubToROS) {

  K.set() = _K;
  q0 = _K.getJointState();

  if(pubSubToROS) {
    self = new SimulationThread_self();
#ifdef RAI_ROS
    self->ROS.publish(self->pub_timeToGo, timeToGo);
#endif
  }

  threadLoop();
}

SimulationThread::~SimulationThread() {
  threadClose();
  if(self) delete self;
}

void SimulationThread::step() {
  //check for inputs:
#ifdef RAI_ROS
  if(pubSubToROS && self->ref.hasNewRevision()) {
    self->ref.readAccess();
    StringA joints = conv_stdStringVec2StringA(self->ref->joints);
    arr t = conv_arr2arr(self->ref->t);
    arr x = conv_arr2arr(self->ref->x);
    bool append = self->ref->append;
    cout <<"MotionReference revision=" <<self->ref.getRevision() <<' ' <<self->ref->revision <<endl;
    self->ref.deAccess();

    SIM.setUsedRobotJoints(joints);
    SIM.exec(x, t, append);
  }

  if(pubSubToROS && self->command.hasNewRevision()) {
    StringA cmd = conv_StringA2StringA(self->command.get());
    SIM.exec(cmd);
  }
#endif

  SIM.stepKin();

  K.set()->setFrameState(SIM.getFrameState());
  frameState.set() = SIM.getFrameState();
  jointState.set() = SIM.getJointState();
  timeToGo.set() = SIM.getTimeToGo();

  //publish:
#ifdef RAI_ROS
  if(pubSubToROS) {
    self->currentQ.set() = conv_arr2arr(SIM.getJointState());
    self->objectNames.set() = conv_StringA2StringA(SIM.getObjectNames());
    self->objectPoses.set() = conv_arr2arr(SIM.getObjectPoses());
  }
#endif
}

void SimulationThread::loop() {
  //loop
  Metronome tictac(dt);
  for(;;) {
    tictac.waitForTic();
    step();
  }
}

bool SimulationThread::executeMotion(const uintA& joints, const arr& path, const arr& times, double timeScale, bool append) {
  auto lock = stepMutex(RAI_HERE);
  SIM.setUsedRobotJoints(joints);
  SIM.exec(path, times*timeScale, append);
  return true;
}

void SimulationThread::execGripper(const rai::String& gripper, double position, double force) {
  auto lock = stepMutex(RAI_HERE);
  if(gripper=="pr2R") {
    //  komo->addObjective(0., 0., FS_accumulatedCollisions, {}, OT_eq, 1e0);
    //open gripper
    //  komo->addObjective(0.,0., FS_qItself, {"r_gripper_joint"}, OT_sos, 1e1, {.08} );
    //  komo->addObjective(0.,0., FS_qItself, {"r_gripper_l_finger_joint"}, OT_sos, 1e1, {.8} );

    SIM.setUsedRobotJoints(SIM.K.getFrameIDs({"r_gripper_joint", "r_gripper_l_finger_joint"}));
    SIM.exec({{1, 2}, {position, position*10.}}, {1.}, true);
    return;
  }
  if(gripper=="pandaL") {
    SIM.setUsedRobotJoints(SIM.K.getFrameIDs({"L_panda_finger_joint1"}));
    SIM.exec(arr({1, 1}, {position}), {1.}, true);
    return;
  }
  NIY
}

arr SimulationThread::getHomePose() { return q0; }

StringA SimulationThread::getJointNames() {
  auto lock = stepMutex(RAI_HERE);
  StringA joints = SIM.getJointNames();
  return joints;
}

void SimulationThread::attach(const char* a, const char* b) {
  auto lock = stepMutex(RAI_HERE);
  SIM.exec({"attach", a, b});
}

arr SimulationThread::getJointPositions(const uintA& joints) {
  auto lock = stepMutex(RAI_HERE);
  SIM.setUsedRobotJoints(joints);
  arr q = SIM.getJointState();
  return q;
}

void SimulationThread::addFile(const char* filename/*, const char* parentOfRoot, const rai::Transformation& relOfRoot*/) {
  auto lock = stepMutex(RAI_HERE);
  SIM.K.addFile(filename/*, parentOfRoot, relOfRoot*/);
  //SIM.K.calc_activeSets();
  //SIM.K.calc_fwdPropagateFrames();
  //SIM.K.checkConsistency();
  K.set() = SIM.K;
}
