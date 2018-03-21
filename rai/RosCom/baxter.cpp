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

struct sSendPositionCommandsToBaxter{
  ros::NodeHandle nh;
  ros::Publisher pubL, pubR, pubLg, pubRg, pubHead, pubGripper;
  rai::KinematicWorld baxterModel;
};

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const rai::KinematicWorld& baxterModel, const char* prefix){
  baxter_core_msgs::JointCommand msg;
  msg.mode = 1;
  for(rai::Joint *j:baxterModel.joints) if(j->name.startsWith(prefix)){
    msg.command.push_back(q_ref(j->qIndex));
    msg.names.push_back(j->name.p);
  }
  return msg;
}

bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return false;
  for(uint i=0;i<n;i++){
    rai::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j) qReal(j->qIndex) = msg.position[i];
  }
  return true;
}

bool baxter_get_q_qdot_u(arr& q, arr& v, arr& u, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return false;
  if(&q && q.N!=baxterModel.q.N) q.resize(baxterModel.q.N).setZero();
  if(&v && v.N!=baxterModel.q.N) v.resize(baxterModel.q.N).setZero();
  if(&u && u.N!=baxterModel.q.N) u.resize(baxterModel.q.N).setZero();
  for(uint i=0;i<n;i++){
    rai::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j){
      if(&q) q(j->qIndex) = msg.position[i];
      if(&v) v(j->qIndex) = msg.velocity[i];
      if(&u) u(j->qIndex) = msg.effort[i];
    }
  }
  return true;
}

arr baxter_getEfforts(const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return arr();
  arr u(baxterModel.q.N);
  u.setZero();
  for(uint i=0;i<n;i++){
    rai::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j) u(j->qIndex) = msg.effort[i];
  }
  return u;
}

baxter_core_msgs::HeadPanCommand getHeadMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel){
  baxter_core_msgs::HeadPanCommand msg;
  rai::Joint *j = baxterModel.getJointByName("head_pan");
  msg.target = q_ref(j->qIndex);
  msg.speed_ratio = 1.;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getGripperMsg(const arr& q_ref, const rai::KinematicWorld& baxterModel){
  baxter_core_msgs::EndEffectorCommand msg;
  rai::Joint *j = baxterModel.getJointByName("l_gripper_l_finger_joint");
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
    s(NULL),
    baxterModel(kw){

    if(rai::getParameter<bool>("useRos",false)){
      s = new sSendPositionCommandsToBaxter;
      s->pubR = s->nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
      s->pubL = s->nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
      s->pubRg = s->nh.advertise<std_msgs::Empty>("robot/limb/right/suppress_gravity_compensation", 1);
      s->pubLg = s->nh.advertise<std_msgs::Empty>("robot/limb/left/suppress_gravity_compensation", 1);
      s->pubHead = s->nh.advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1);
      s->pubGripper = s->nh.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 1);
    }

}

void SendPositionCommandsToBaxter::open(){
  this->threadLoop();
}

void SendPositionCommandsToBaxter::step(){
  if(s){
    arr q_ref = ctrl_ref.get()->q;
    if(!q_ref.N) return;

    if (totalTorqueModeL)
      s->pubLg.publish(std_msgs::Empty());

    if (totalTorqueModeR)
      s->pubRg.publish(std_msgs::Empty());

    if (enablePositionControlL && !totalTorqueModeL)
      s->pubL.publish(conv_qRef2baxterMessage(q_ref, baxterModel, "left_"));

    if (enablePositionControlR && !totalTorqueModeR)
      s->pubR.publish(conv_qRef2baxterMessage(q_ref, baxterModel, "right_"));

    s->pubHead.publish(getHeadMsg(q_ref, baxterModel));
    s->pubGripper.publish(getGripperMsg(q_ref, baxterModel));
  }
  else
  {
    close();
  }
}

void SendPositionCommandsToBaxter::close(){
  if(s) delete s;
}

#else

#ifdef RAI_ROS
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel){ NICO }
#endif

#endif
