#include "baxter.h"

#ifdef MLR_ROS

#include "roscom.h"
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>

struct sSendPositionCommandsToBaxter{
  ros::NodeHandle nh;
  ros::Publisher pubL, pubR, pubLg, pubRg, pubHead, pubGripper;
  ors::KinematicWorld baxterModel;
};

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const ors::KinematicWorld& baxterModel, const char* prefix){
  baxter_core_msgs::JointCommand msg;
  msg.mode = 1;
  for(ors::Joint *j:baxterModel.joints) if(j->name.startsWith(prefix)){
    msg.command.push_back(q_ref(j->qIndex));
    msg.names.push_back(j->name.p);
  }
  return msg;
}

bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return false;
  for(uint i=0;i<n;i++){
    ors::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j) qReal(j->qIndex) = msg.position[i];
  }
  return true;
}

bool baxter_get_q_qdot_u(arr& q, arr& v, arr& u, const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return false;
  if(&q && q.N!=baxterModel.q.N) q.resize(baxterModel.q.N).setZero();
  if(&v && v.N!=baxterModel.q.N) v.resize(baxterModel.q.N).setZero();
  if(&u && u.N!=baxterModel.q.N) u.resize(baxterModel.q.N).setZero();
  for(uint i=0;i<n;i++){
    ors::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j){
      if(&q) q(j->qIndex) = msg.position[i];
      if(&v) v(j->qIndex) = msg.velocity[i];
      if(&u) u(j->qIndex) = msg.effort[i];
    }
  }
  return true;
}

arr baxter_getEfforts(const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel){
  uint n = msg.name.size();
  if(!n) return arr();
  arr u(baxterModel.q.N);
  u.setZero();
  for(uint i=0;i<n;i++){
    ors::Joint *j = baxterModel.getJointByName(msg.name[i].c_str(), false);
    if(j) u(j->qIndex) = msg.effort[i];
  }
  return u;
}

baxter_core_msgs::HeadPanCommand getHeadMsg(const arr& q_ref, const ors::KinematicWorld& baxterModel){
  baxter_core_msgs::HeadPanCommand msg;
  ors::Joint *j = baxterModel.getJointByName("head_pan");
  msg.target = q_ref(j->qIndex);
  msg.speed_ratio = 1.;
  return msg;
}

baxter_core_msgs::EndEffectorCommand getGripperMsg(const arr& q_ref, const ors::KinematicWorld& baxterModel){
  baxter_core_msgs::EndEffectorCommand msg;
  ors::Joint *j = baxterModel.getJointByName("l_gripper_l_finger_joint");
  mlr::String str;
  str <<"{ \"position\":" <<1000.*q_ref(j->qIndex) <<", \"dead zone\":5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0 }";
  //cout <<str <<endl;

  msg.id = 65538;
  msg.command = msg.CMD_GO;
  msg.args = str.p;
  msg.sender = "foo";
  msg.sequence = 1;
  return msg;
}


SendPositionCommandsToBaxter::SendPositionCommandsToBaxter(const ors::KinematicWorld& kw)
  : Module("SendPositionCommandsToBaxter"),
    ctrl_ref(this, "ctrl_ref", true),
    s(NULL),
    baxterModel(kw){
}

void SendPositionCommandsToBaxter::open(){
  if(mlr::getParameter<bool>("useRos",false)){
    s = new sSendPositionCommandsToBaxter;
    s->pubR = s->nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
    s->pubL = s->nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
    s->pubRg = s->nh.advertise<std_msgs::Empty>("robot/limb/right/suppress_gravity_compensation", 1);
    s->pubLg = s->nh.advertise<std_msgs::Empty>("robot/limb/left/suppress_gravity_compensation", 1);
    s->pubHead = s->nh.advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1);
    s->pubGripper = s->nh.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 1);
  }
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
}

void SendPositionCommandsToBaxter::close(){
  if(s) delete s;
}

#endif
