/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "baxter.h"

#ifdef RAI_ROS_BAXTER

#include "roscom.h"
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>
#include <RosCom/roscom.h>

#include <Kin/frame.h>

struct sBaxterInterface {
  RosCom ROS;

  Var<sensor_msgs::JointState> state;

  ros::NodeHandle nh;
  ros::Publisher pubL, pubR, pubLg, pubRg, pubHead, pubGripper;
  rai::KinematicWorld baxterModel;

  std::shared_ptr<Subscriber<sensor_msgs::JointState>> sub_state;

  sBaxterInterface()
    : state("/robot/joint_states"){
    baxterModel.addFile(rai::raiPath("../rai-robotModels/baxter/baxter.g"));

    if(rai::getParameter<bool>("useRos",false)) {
      rai::wait(.5);
      pubR = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
      pubL = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
      pubRg = nh.advertise<std_msgs::Empty>("robot/limb/right/suppress_gravity_compensation", 1);
      pubLg = nh.advertise<std_msgs::Empty>("robot/limb/left/suppress_gravity_compensation", 1);
      pubHead = nh.advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1);
      pubGripper = nh.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 1);

      ROS.subscribe(sub_state, state, true);
      rai::wait(.5);
    }
  }
};

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const rai::KinematicWorld& baxterModel, const char* prefix) {
  baxter_core_msgs::JointCommand msg;
  msg.mode = 1;
  for(rai::Joint *j:baxterModel.fwdActiveJoints) if(j->frame.name.startsWith(prefix)) {
      msg.command.push_back(q_ref(j->qIndex));
      msg.names.push_back(j->frame.name.p);
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

baxter_core_msgs::EndEffectorCommand getGripperMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel) {
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint *j = baxterModel.getFrameByName("l_gripper_l_finger_joint")->joint;
  rai::String str;
  
  double position = q_ref(j->qIndex) / (j->limits(1) - j->limits(0)) * 100.0;
  
//  str <<"{ \"position\":" <<1000.*q_ref(j->qIndex) <<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  str <<"{ \"position\":" << position<<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  
  //cout <<str <<endl;
  
  msg.id = 65538;
  msg.command = msg.CMD_GO;
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 1;
  return msg;
}

SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const rai::KinematicWorld& kw)
  : Thread("SendPositionCommandsToBaxter"),
    ctrl_ref(NULL, "ctrl_ref", true),
    s(0) {

  s = new sBaxterInterface;
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
    s->pubGripper.publish(getGripperMsg(q_ref, s->baxterModel));
  } else {
    close();
  }
}

void SendPositionCommandsToBaxter::close() {
  if(s) delete s;
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

BaxterInterface::BaxterInterface() : s(0){
  s = new sBaxterInterface;
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

void BaxterInterface::send_q(const arr& q_ref, bool enableL, bool enableR){
  if(enableL)
    s->pubL.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "left_"));

  if(enableR)
    s->pubR.publish(conv_qRef2baxterMessage(q_ref, s->baxterModel, "right_"));

  s->pubHead.publish(getHeadMsg(q_ref, s->baxterModel));
  s->pubGripper.publish(getGripperMsg(q_ref, s->baxterModel));
}
