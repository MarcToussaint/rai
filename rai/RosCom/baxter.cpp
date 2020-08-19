/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "baxter.h"

#ifdef RAI_ROS
#ifdef RAI_ROS_BAXTER

#include "roscom.h"
#include "../rai_msgs/baxter/HeadPanCommand.h"
#include "../rai_msgs/baxter/EndEffectorCommand.h"
#include "../rai_msgs/baxter/EndEffectorState.h"
#include "../rai_msgs/baxter/JointCommand.h"
#include "../RosCom/roscom.h"

#include "../Kin/frame.h"

struct sBaxterInterface {
  RosCom ROS;

  Var<sensor_msgs::JointState> state;
  Var<baxter_core_msgs::EndEffectorState> gripR;
  Var<baxter_core_msgs::EndEffectorState> gripL;

  ptr<ros::NodeHandle> nh;
  ros::Publisher pubL, pubR, pubLg, pubRg, pubHead, pubGripperR, pubGripperL;
  rai::Configuration baxterModel;

  std::shared_ptr<Subscriber<sensor_msgs::JointState>> sub_state;
  std::shared_ptr<Subscriber<baxter_core_msgs::EndEffectorState>> sub_gripR;
  std::shared_ptr<Subscriber<baxter_core_msgs::EndEffectorState>> sub_gripL;

  sBaxterInterface(bool useRosDefault) {
    baxterModel.addFile(rai::raiPath("../rai-robotModels/baxter/baxter.g"));

    if(rai::getParameter<bool>("useRos", useRosDefault)) {
      nh = make_shared<ros::NodeHandle>();
      rai::wait(.5);
      pubR = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
      pubL = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
      pubRg = nh->advertise<std_msgs::Empty>("robot/limb/right/suppress_gravity_compensation", 1);
      pubLg = nh->advertise<std_msgs::Empty>("robot/limb/left/suppress_gravity_compensation", 1);
      pubHead = nh->advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1);
      pubGripperR = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command", 1);
      pubGripperL = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 1);

      state.name() = "/robot/joint_states";
      gripR.name() = "/robot/end_effector/right_gripper/state";
      gripL.name() = "/robot/end_effector/left_gripper/state";
      ROS.subscribe(sub_state, state, true);
      ROS.subscribe(sub_gripR, gripR);
      ROS.subscribe(sub_gripL, gripL);

      rai::wait(.5);
    }
  }
};

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const rai::Configuration& baxterModel, const char* prefix) {
  baxter_core_msgs::JointCommand msg;
  msg.mode = 1;
  for(rai::Joint* j:baxterModel.activeJoints) if(j->frame->name.startsWith(prefix)) {
      msg.command.push_back(q_ref(j->qIndex));
      msg.names.push_back(j->frame->name.p);
    }
  return msg;
}

bool baxter_get_q_qdot_u(arr& q, arr& v, arr& u, const sensor_msgs::JointState& msg, const rai::Configuration& baxterModel) {
  uint n = msg.name.size();
  if(!n) return false;
  if(!!q && q.N!=baxterModel.q.N) q.resize(baxterModel.q.N).setZero();
  if(!!v && v.N!=baxterModel.q.N) v.resize(baxterModel.q.N).setZero();
  if(!!u && u.N!=baxterModel.q.N) u.resize(baxterModel.q.N).setZero();
  for(uint i=0; i<n; i++) {
    rai::Frame* f = baxterModel.getFrameByName(msg.name[i].c_str(), false);
    if(f) {
      rai::Joint* j = f->joint;
      if(j) {
        if(!!q) q(j->qIndex) = msg.position[i];
        if(!!v) v(j->qIndex) = msg.velocity[i];
        if(!!u) u(j->qIndex) = msg.effort[i];
      }
    }
  }
  return true;
}

baxter_core_msgs::HeadPanCommand getHeadMsg(const arr& q_ref, const rai::Configuration& baxterModel) {
  baxter_core_msgs::HeadPanCommand msg;
  rai::Joint* j = baxterModel.getFrameByName("head_pan")->joint;
  msg.target = q_ref(j->qIndex);
  msg.speed_ratio = 1.;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getElectricGripperMsg(const arr& q_ref, const rai::Configuration& baxterModel) {
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint* j = baxterModel.getFrameByName("r_gripper_l_finger_joint")->joint;
  rai::String str;

  bool position = bool(q_ref(j->qIndex));

//  str <<"{ \"position\":" <<1000.*q_ref(j->qIndex) <<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  str <<"{ \"position\":" << position<<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";

  msg.id = 65538;
  if(position) {
    msg.command = msg.CMD_GRIP; //CMD_GO;
  } else {
    msg.command = msg.CMD_RELEASE;
  }
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 1;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getVacuumGripperMsg(const arr& q_ref, const rai::Configuration& baxterModel) {
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint* j = baxterModel.getFrameByName("l_gripper_l_finger_joint")->joint;
  rai::String str;

  bool suction = bool(q_ref(j->qIndex));

  str <<"{ \"blowing\" : false,\n  \"suction\" : "<<suction<<",\n  \"vacuum\" : false,\n \"vacuum threshold\" : 46}";

  msg.id = 65537;
  if(suction) {
    msg.command = msg.CMD_GRIP;
  } else {
    msg.command = msg.CMD_RELEASE;
  }
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 6;
  return msg;
}

SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const rai::Configuration& kw, const Var<CtrlMsg>& _ctrl_ref)
  : Thread("SendPositionCommandsToBaxter"),
    ctrl_ref(nullptr, true),
    s(0) {

  self = make_unique<sBaxterInterface>(true);
  self->baxterModel = kw;
}

void SendPositionCommandsToBaxter::open() {
  this->threadLoop();
}

void SendPositionCommandsToBaxter::step() {
  if(s) {
    arr q_ref = ctrl_ref.get()->q;
    if(!q_ref.N) return;

    if(totalTorqueModeL)
      self->pubLg.publish(std_msgs::Empty());

    if(totalTorqueModeR)
      self->pubRg.publish(std_msgs::Empty());

    if(enablePositionControlL && !totalTorqueModeL)
      self->pubL.publish(conv_qRef2baxterMessage(q_ref, self->baxterModel, "left_"));

    if(enablePositionControlR && !totalTorqueModeR)
      self->pubR.publish(conv_qRef2baxterMessage(q_ref, self->baxterModel, "right_"));

    self->pubHead.publish(getHeadMsg(q_ref, self->baxterModel));
    self->pubGripperR.publish(getElectricGripperMsg(q_ref, self->baxterModel));
    self->pubGripperL.publish(getVacuumGripperMsg(q_ref, self->baxterModel));

  } else {
    close();
  }
}

void SendPositionCommandsToBaxter::close() {
  if(s) s.rewset()
  }

BaxterInterface::BaxterInterface(bool useRosDefault) : s(0) {
  self = make_unique<sBaxterInterface>(useRosDefault);
}

BaxterInterface::~BaxterInterface() {
}

arr BaxterInterface::get_q() {
  arr q;
  baxter_get_q_qdot_u(q, NoArr, NoArr, self->state.get(), self->baxterModel);
  return q;
}

arr BaxterInterface::get_qdot() {
  arr qdot;
  baxter_get_q_qdot_u(NoArr, qdot, NoArr, self->state.get(), self->baxterModel);
  return qdot;
}

arr BaxterInterface::get_u() {
  arr u;
  baxter_get_q_qdot_u(NoArr, NoArr, u, self->state.get(), self->baxterModel);
  return u;
}

bool BaxterInterface::get_grabbed(const std::string& whichArm) {
  bool grabbed;
  baxter_core_msgs::EndEffectorState msgs;

  if(whichArm=="left") {
    msgs = self->gripL.get();
  } else {
    msgs = self->gripR.get();
  }
  grabbed=bool(int(msgs.gripping));

  return grabbed;
}

bool BaxterInterface::get_opened(const std::string& whichArm) {
  bool opened, grabbed;
  baxter_core_msgs::EndEffectorState msgs;

  if(whichArm=="left") {
    msgs = self->gripL.get();
  } else {
    msgs = self->gripR.get();
  }
  grabbed=get_grabbed(whichArm);

  if(msgs.position>90 && !get_grabbed(whichArm)) {
    opened=true;
  } else {
    opened=false;
  }

  return opened;
}

void BaxterInterface::send_q(const arr& q_ref, bool enableL, bool enableR) {
  if(enableL)
    self->pubL.publish(conv_qRef2baxterMessage(q_ref, self->baxterModel, "left_"));

  if(enableR)
    self->pubR.publish(conv_qRef2baxterMessage(q_ref, self->baxterModel, "right_"));

  self->pubHead.publish(getHeadMsg(q_ref, self->baxterModel));
  self->pubGripperR.publish(getElectricGripperMsg(q_ref, self->baxterModel));
  self->pubGripperL.publish(getVacuumGripperMsg(q_ref, self->baxterModel));
}

#else

#ifdef RAI_ROS
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const rai::Configuration& baxterModel) { NICO }

SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const rai::Configuration& kw, const Var<CtrlMsg>& _ctrl_ref)
  : Thread("SendPositionCommandsToBaxter"),
    ctrl_ref(nullptr, _ctrl_ref, true),
    s(nullptr),
    baxterModel(kw) {
  NICO;
}
void SendPositionCommandsToBaxter::open() { NICO }
void SendPositionCommandsToBaxter::step() { NICO }
void SendPositionCommandsToBaxter::close() { NICO }

#endif

#endif
#else

struct sBaxterInterface {};

BaxterInterface::BaxterInterface(bool useRosDefault) { NICO }
BaxterInterface::~BaxterInterface() { NICO }

arr BaxterInterface::get_q() { NICO }
arr BaxterInterface::get_qdot() { NICO }
arr BaxterInterface::get_u() { NICO }

bool BaxterInterface::get_grabbed(const std::string& whichArm) { NICO }
bool BaxterInterface::get_opened(const std::string& whichArm) { NICO }

void BaxterInterface::send_q(const arr& q_ref, bool enableL, bool enableR) { NICO }

#endif

