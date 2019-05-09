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
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <RosCom/roscom.h>

#include <Kin/frame.h>

struct sBaxterInterface {
  RosCom ROS;

  Var<sensor_msgs::JointState> state;
  Var<baxter_core_msgs::EndEffectorState> gripR;
  Var<baxter_core_msgs::EndEffectorState> gripL;

  ptr<ros::NodeHandle> nh;
  ros::Publisher pubL, pubR, pubLg, pubRg, pubHead, pubGripperR, pubGripperL;
  rai::KinematicWorld baxterModel;

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
      ROS.subscribe(sub_gripR, gripR, true);
      ROS.subscribe(sub_gripL, gripL, true);

      rai::wait(.5);
    }
  }
};

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const rai::KinematicWorld& baxterModel, const char* prefix) {
  baxter_core_msgs::JointCommand msg;
  msg.mode = 1;
  for(rai::Joint *j:baxterModel.fwdActiveJoints) if(j->frame->name.startsWith(prefix)) {
      msg.command.push_back(q_ref(j->qIndex));
      msg.names.push_back(j->frame->name.p);
    }
  return msg;
}

bool baxter_get_q_qdot_u(arr& q, arr& v, arr& u, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel) {
  uint n = msg.name.size();
  if(!n) return false;
  if(!!q && q.N!=baxterModel.q.N) q.resize(baxterModel.q.N).setZero();
  if(!!v && v.N!=baxterModel.q.N) v.resize(baxterModel.q.N).setZero();
  if(!!u && u.N!=baxterModel.q.N) u.resize(baxterModel.q.N).setZero();
  for(uint i=0; i<n; i++) {
    rai::Frame *f = baxterModel.getFrameByName(msg.name[i].c_str(), false);
    if(f){
      rai::Joint *j = f->joint;
      if(j) {
        if(!!q) q(j->qIndex) = msg.position[i];
        if(!!v) v(j->qIndex) = msg.velocity[i];
        if(!!u) u(j->qIndex) = msg.effort[i];
      }
    }
  }
  return true;
}

baxter_core_msgs::HeadPanCommand getHeadMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel) {
  baxter_core_msgs::HeadPanCommand msg;
  rai::Joint *j = baxterModel.getFrameByName("head_pan")->joint;
  msg.target = q_ref(j->qIndex);
  msg.speed_ratio = 1.;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getElectricGripperMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel) {
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint *j = baxterModel.getFrameByName("r_gripper_l_finger_joint")->joint;
  rai::String str;
  
  bool position = bool(q_ref(j->qIndex));

//  str <<"{ \"position\":" <<1000.*q_ref(j->qIndex) <<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  str <<"{ \"position\":" << position<<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  
  msg.id = 65538;
  if(position){
    msg.command = msg.CMD_GRIP; //CMD_GO;
  }else{
    msg.command = msg.CMD_RELEASE;
  }
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 1;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getVacuumGripperMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel) {
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint *j = baxterModel.getFrameByName("l_gripper_l_finger_joint")->joint;
  rai::String str;
  
  bool suction = bool(q_ref(j->qIndex));

  str <<"{ \"blowing\" : false,\n  \"suction\" : "<<suction<<",\n  \"vacuum\" : false,\n \"vacuum threshold\" : 46}";
  
  msg.id = 65537;
  if(suction){
    msg.command = msg.CMD_GRIP;
  }else{
    msg.command = msg.CMD_RELEASE;
  }
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 6;
  return msg;
}

SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const rai::KinematicWorld& kw, const Var<CtrlMsg>& _ctrl_ref)
  : Thread("SendPositionCommandsToBaxter"),
    ctrl_ref(NULL, true),
    s(0) {

  s = new sBaxterInterface(true);
  s->baxterModel = kw;
}

void SendPositionCommandsToBaxter::open() {
  this->threadLoop();
}

void SendPositionCommandsToBaxter::step() {
  if(s) {
    arr q_ref = ctrl_ref.get()->q;
    if(!q_ref.N) return;
    
    if(totalTorqueModeL)
      s->pubLg.publish(std_msgs::Empty());
      
    if(totalTorqueModeR)
      s->pubRg.publish(std_msgs::Empty());
      
    if(enablePositionControlL && !totalTorqueModeL)
      s->pubL.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "left_"));
      
    if(enablePositionControlR && !totalTorqueModeR)
      s->pubR.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "right_"));
      
    s->pubHead.publish(getHeadMsg(q_ref, s->baxterModel));
    s->pubGripperR.publish(getElectricGripperMsg(q_ref, s->baxterModel));
    s->pubGripperL.publish(getVacuumGripperMsg(q_ref, s->baxterModel));

  } else {
    close();
  }
}

void SendPositionCommandsToBaxter::close() {
  if(s) delete s;
}

BaxterInterface::BaxterInterface(bool useRosDefault) : s(0){
  s = new sBaxterInterface(useRosDefault);
}

BaxterInterface::~BaxterInterface(){
  delete s;
}

arr BaxterInterface::get_q(){
  arr q;
  baxter_get_q_qdot_u(q, NoArr, NoArr, s->state.get(), s->baxterModel);
  return q;
}

arr BaxterInterface::get_qdot(){
  arr qdot;
  baxter_get_q_qdot_u(NoArr, qdot, NoArr, s->state.get(), s->baxterModel);
  return qdot;
}

arr BaxterInterface::get_u(){
  arr u;
  baxter_get_q_qdot_u(NoArr, NoArr, u, s->state.get(), s->baxterModel);
  return u;
}

bool BaxterInterface::get_grabbed(const std::string& whichArm){
  bool grabbed;
  baxter_core_msgs::EndEffectorState msgs;

  if(whichArm=="left"){
    msgs = s->gripL.get();
  }
  else{
    msgs = s->gripR.get();
  }
  grabbed=bool(int(msgs.gripping));

  return grabbed;
}

bool BaxterInterface::get_opened(const std::string& whichArm){
  bool opened, grabbed;
  baxter_core_msgs::EndEffectorState msgs;

  if(whichArm=="left"){
    msgs = s->gripL.get();
  }
  else{
    msgs = s->gripR.get();
  }
  grabbed=get_grabbed(whichArm);

  if(msgs.position>90 && !get_grabbed(whichArm)){
    opened=true;
  }
  else{
    opened=false;
  }

  return opened;
}

void BaxterInterface::send_q(const arr& q_ref, bool enableL, bool enableR){
  if(enableL)
    s->pubL.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "left_"));

  if(enableR)
    s->pubR.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "right_"));

  s->pubHead.publish(getHeadMsg(q_ref, s->baxterModel));
  s->pubGripperR.publish(getElectricGripperMsg(q_ref, s->baxterModel));
  s->pubGripperL.publish(getVacuumGripperMsg(q_ref, s->baxterModel));
}

#else

#ifdef RAI_ROS
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel) { NICO }

SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const rai::KinematicWorld& kw, const Var<CtrlMsg>& _ctrl_ref)
  : Thread("SendPositionCommandsToBaxter"),
    ctrl_ref(NULL, _ctrl_ref, true),
    s(NULL),
    baxterModel(kw) {
  NICO;
}
void SendPositionCommandsToBaxter::open() { NICO }
void SendPositionCommandsToBaxter::step() { NICO }
void SendPositionCommandsToBaxter::close() { NICO }
#endif

#endif
#endif

