/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "roscom.h"
#include "util.h"

#ifdef RAI_ROS
#include <ros/ros.h>
#include <ros_msg/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include "../Geo/geo.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

void PerceptionObjects2Ors::step() {
  perceptionObjects.readAccess();
  modelWorld.readAccess();

  for(visualization_msgs::Marker& marker : perceptionObjects().markers) {
    rai::String name;
    name <<"obj" <<marker.id;
    rai::Shape* s = modelWorld->getFrameByName(name)->shape;
    if(!s) {
      rai::Frame* f = new rai::Frame(modelWorld());
      s = new rai::Shape(*f);
      if(marker.type==marker.CYLINDER) {
        s->type() = rai::ST_cylinder;
        s->size(3) = .5*(marker.scale.x+marker.scale.y);
        s->size(2) = marker.scale.z;
      } else if(marker.type==marker.POINTS) {
        s->type() = rai::ST_mesh;
        s->mesh().V = conv_points2arr(marker.points);
        s->mesh().C = conv_colors2arr(marker.colors);
      } else NIY;
    }
  }

  perceptionObjects.deAccess();
  modelWorld.deAccess();
}

//===========================================================================
// RosCom_Spinner
//struct sRosCom_Spinner{
//};

//void RosCom_Spinner::open(){
//  rosCheckInit();
//}

//void RosCom_Spinner::step(){
//  ros::spinOnce();
//}

//void RosCom_Spinner::close(){}

//===========================================================================
// CosCom_ControllerSync
//struct sRosCom_ControllerSync{
//  RosCom_ControllerSync *base;
//  ros::NodeHandle nh;
//  ros::Subscriber sub_jointState;
////  ros::Subscriber sub_odom;
//  ros::Publisher pub_jointReference;

//  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
//    //  cout <<"** joinstState_callback" <<endl;
//    CtrlMsg m(conv_stdvec2arr(msg->q), conv_stdvec2arr(msg->qdot), conv_stdvec2arr(msg->fL), conv_stdvec2arr(msg->fR), conv_stdvec2arr(msg->u_bias), conv_stdvec2arr(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
//    base->ctrl_obs.set() = m;
//  }
////  void odom_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
////    //  cout <<"** joinstState_callback" <<endl;
////    CtrlMsg m(conv_stdvec2arr(msg->q), conv_stdvec2arr(msg->qdot), conv_stdvec2arr(msg->fL), conv_stdvec2arr(msg->fR), conv_stdvec2arr(msg->u_bias), conv_stdvec2arr(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
////    base->ctrl_obs.set() = m;
////  }
//};

//void RosCom_ControllerSync::open(){
//  rosCheckInit();
//  self = make_unique<sRosCom_ControllerSync>();
//  self->base=this;
//  self->sub_jointState = self->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom_ControllerSync::joinstState_callback, s);
////  self->sub_odom = self->nh.subscribe("/robot_pose_ekf/odom_combined", 1, &sRosCom_ControllerSync::joinstState_callback, s);
//  self->pub_jointReference = self->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
//  //  self->sub_jointState = self->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
//  //  self->pub_jointReference = self->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
//}

//void RosCom_ControllerSync::step(){
//  CtrlMsg m = ctrl_ref.get();
//  if(!m.q.N) return;
//  marc_controller_pkg::JointState jointRef;
//  jointRef.q = conv_arr2stdvec(m.q);
//  jointRef.qdot= conv_arr2stdvec(m.qdot);
//  jointRef.fL = conv_arr2stdvec(m.fL);
//  jointRef.u_bias = conv_arr2stdvec(m.u_bias);
//  jointRef.Kp = conv_arr2stdvec(m.Kp);
//  jointRef.Kd = conv_arr2stdvec(m.Kd);
//  jointRef.Ki = conv_arr2stdvec(m.Ki);
//  jointRef.KiFT = conv_arr2stdvec(m.KiFT);
//  jointRef.J_ft_inv = conv_arr2stdvec(m.J_ft_inv);
//  jointRef.velLimitRatio = m.velLimitRatio;
//  jointRef.effLimitRatio = m.effLimitRatio;
//  jointRef.intLimitRatio = m.intLimitRatio;
//  jointRef.gamma = m.gamma;
//  self->pub_jointReference.publish(jointRef);
//}

//void RosCom_ControllerSync::close(){
//  self->nh.shutdown();
//  self.reset();
//}

//===========================================================================
// Helper function so sync ors with the real PR2
void initialSyncJointStateWithROS(rai::Configuration& world,
                                  Var<CtrlMsg>& ctrl_obs, bool useRos) {

  if(not useRos) { return; }

  //-- wait for first q observation!
  cout << "** Waiting for ROS message of joints for initial configuration.." << endl
       << "   If nothing is happening: is the controller running?" << endl;

  for(uint trials = 0; trials < 20; trials++) {
    ctrl_obs.waitForNextRevision();
    cout << "REMOTE joint dimension=" << ctrl_obs.get()->q.N << endl;
    cout << "LOCAL  joint dimension=" << world.q.N << endl;

    if(ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      cout << "** Updating world state" << endl;
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
    cout << "retrying..." << endl;
  }
  HALT("sync'ing real PR2 with simulated failed");
}

void syncJointStateWitROS(rai::Configuration& world,
                          Var<CtrlMsg>& ctrl_obs, bool useRos) {

  if(not useRos) { return; }

  for(uint trials = 0; trials < 2; trials++) {
    ctrl_obs.waitForNextRevision();

    if(ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
  }
  HALT("sync'ing real PR2 with simulated failed");
}

//===========================================================================
// RosCom_Spinner
struct sRosCom_Spinner {
};

void RosCom_Spinner::open() {
  rosCheckInit();
}

void RosCom_Spinner::step() {
  ros::spinOnce();
}

void RosCom_Spinner::close() {}

//===========================================================================
// CosCom_ControllerSync
struct sRosCom_ControllerSync {
  RosCom_ControllerSync* base;
  ros::NodeHandle nh;
  ros::Subscriber sub_jointState;
//  ros::Subscriber sub_odom;
  ros::Publisher pub_jointReference;

  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg) {
    //  cout <<"** joinstState_callback" <<endl;
    CtrlMsg m(conv_stdvec2arr(msg->q), conv_stdvec2arr(msg->qdot), conv_stdvec2arr(msg->fL), conv_stdvec2arr(msg->fR), conv_stdvec2arr(msg->u_bias), conv_stdvec2arr(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
    base->ctrl_obs.set() = m;
  }
//  void odom_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
//    //  cout <<"** joinstState_callback" <<endl;
//    CtrlMsg m(conv_stdvec2arr(msg->q), conv_stdvec2arr(msg->qdot), conv_stdvec2arr(msg->fL), conv_stdvec2arr(msg->fR), conv_stdvec2arr(msg->u_bias), conv_stdvec2arr(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
//    base->ctrl_obs.set() = m;
//  }
};

void RosCom_ControllerSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_ControllerSync>();
  self->base=this;
  self->sub_jointState = self->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom_ControllerSync::joinstState_callback, s);
//  self->sub_odom = self->nh.subscribe("/robot_pose_ekf/odom_combined", 1, &sRosCom_ControllerSync::joinstState_callback, s);
  self->pub_jointReference = self->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
  //  self->sub_jointState = self->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
  //  self->pub_jointReference = self->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
}

void RosCom_ControllerSync::step() {
  CtrlMsg m = ctrl_ref.get();
  if(!m.q.N) return;
  marc_controller_pkg::JointState jointRef;
  jointRef.q = conv_arr2stdvec(m.q);
  jointRef.qdot= conv_arr2stdvec(m.qdot);
  jointRef.fL = conv_arr2stdvec(m.fL);
  jointRef.u_bias = conv_arr2stdvec(m.u_bias);
  jointRef.Kp = conv_arr2stdvec(m.Kp);
  jointRef.Kd = conv_arr2stdvec(m.Kd);
  jointRef.Ki = conv_arr2stdvec(m.Ki);
  jointRef.KiFT = conv_arr2stdvec(m.KiFT);
  jointRef.J_ft_inv = conv_arr2stdvec(m.J_ft_inv);
  jointRef.velLimitRatio = m.velLimitRatio;
  jointRef.effLimitRatio = m.effLimitRatio;
  jointRef.intLimitRatio = m.intLimitRatio;
  jointRef.gamma = m.gamma;
  self->pub_jointReference.publish(jointRef);
}

void RosCom_ControllerSync::close() {
  self->nh.shutdown();
  self.reset();
}

//===========================================================================
// Helper function so sync ors with the real PR2
void initialSyncJointStateWithROS(rai::Configuration& world,
                                  Var<CtrlMsg>& ctrl_obs, bool useRos) {

  if(not useRos) { return; }

  //-- wait for first q observation!
  cout << "** Waiting for ROS message of joints for initial configuration.." << endl
       << "   If nothing is happening: is the controller running?" << endl;

  for(uint trials = 0; trials < 20; trials++) {
    ctrl_obs.data->waitForNextRevision();
    cout << "REMOTE joint dimension=" << ctrl_obs.get()->q.N << endl;
    cout << "LOCAL  joint dimension=" << world.q.N << endl;

    if(ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      cout << "** Updating world state" << endl;
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
    cout << "retrying..." << endl;
  }
  HALT("sync'ing real PR2 with simulated failed");
}

void syncJointStateWitROS(rai::Configuration& world,
                          Var<CtrlMsg>& ctrl_obs, bool useRos) {

  if(not useRos) { return; }

  for(uint trials = 0; trials < 2; trials++) {
    ctrl_obs.data->waitForNextRevision();

    if(ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
  }
  HALT("sync'ing real PR2 with simulated failed");
}

//===========================================================================
// RosCom_KinectSync
struct sRosCom_KinectSync {
  RosCom_KinectSync* base;
  ros::NodeHandle nh;
  ros::Subscriber sub_rgb;
  ros::Subscriber sub_depth;
  tf::TransformListener listener;

  void cb_rgb(const sensor_msgs::Image::ConstPtr& msg) {
    //  cout <<"** sRosCom_KinectSync callback" <<endl;
    base->kinect_rgb.set(conv_time2double(msg->header.stamp)) = conv_stdvec2arr(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_depth(const sensor_msgs::Image::ConstPtr& msg) {
    //  cout <<"** sRosCom_KinectSync callback" <<endl;
    byteA data = conv_stdvec2arr(msg->data);
    uint16A ref((const uint16_t*)data.p, data.N/2);
    ref.reshape(msg->height, msg->width);
    double time=conv_time2double(msg->header.stamp);
    base->kinect_depth.set(time) = ref;
    base->kinect_frame.set(time) = ros_getTransform("/base_link", msg->header.frame_id, listener);
  }
};

void RosCom_KinectSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_KinectSync>();
  self->base = this;
  self->sub_rgb = self->nh.subscribe("/kinect_head/rgb/image_color", 1, &sRosCom_KinectSync::cb_rgb, s);
  self->sub_depth = self->nh.subscribe("/kinect_head/depth/image_raw", 1, &sRosCom_KinectSync::cb_depth, s);
}

void RosCom_KinectSync::step() {
}

void RosCom_KinectSync::close() {
  self->nh.shutdown();
}

//===========================================================================
// RosCom_CamsSync
struct sRosCom_CamsSync {
  RosCom_CamsSync* base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const sensor_msgs::Image::ConstPtr& msg) {
    base->rgb_leftEye.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_right(const sensor_msgs::Image::ConstPtr& msg) {
    base->rgb_rightEye.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
};

void RosCom_CamsSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_CamsSync>();
  self->base = this;
  self->sub_left  = self->nh.subscribe("/wide_stereo/left/image_rect_color", 1, &sRosCom_CamsSync::cb_left, s);
  self->sub_right = self->nh.subscribe("/wide_stereo/right/image_rect_color", 1, &sRosCom_CamsSync::cb_right, s);
}

void RosCom_CamsSync::step() {
}

void RosCom_CamsSync::close() {
  self->nh.shutdown();
}

//===========================================================================
// RosCom_ArmCamsSync
struct sRosCom_ArmCamsSync {
  RosCom_ArmCamsSync* base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const sensor_msgs::Image::ConstPtr& msg) {
    base->rgb_leftArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_right(const sensor_msgs::Image::ConstPtr& msg) {
    base->rgb_rightArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
};

void RosCom_ArmCamsSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_ArmCamsSync>();
  self->base = this;
  self->sub_left  = self->nh.subscribe("/l_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_left, s);
  self->sub_right = self->nh.subscribe("/r_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_right, s);
}

void RosCom_ArmCamsSync::step() {
}

void RosCom_ArmCamsSync::close() {
  self->nh.shutdown();
}

//===========================================================================
// RosCom_ForceSensorSync
struct sRosCom_ForceSensorSync {
  RosCom_ForceSensorSync* base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    const geometry_msgs::Vector3& f=msg->wrench.force;
    const geometry_msgs::Vector3& t=msg->wrench.torque;
    base->wrenchL.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
  }
  void cb_right(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    const geometry_msgs::Vector3& f=msg->wrench.force;
    const geometry_msgs::Vector3& t=msg->wrench.torque;
    base->wrenchR.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
  }

};

void RosCom_ForceSensorSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_ForceSensorSync>();
  self->base = this;
  self->sub_left  = self->nh.subscribe("/ft_sensor/ft_compensated", 1, &sRosCom_ForceSensorSync::cb_left, s);  // /ft/l_gripper_motor
//  self->sub_right = self->nh.subscribe("/ft_sensor/r_ft_compensated", 1, &sRosCom_ForceSensorSync::cb_right, s); // /ft/r_gripper_motor
}

void RosCom_ForceSensorSync::step() {
}

void RosCom_ForceSensorSync::close() {
  self->nh.shutdown();
}

//===========================================================================
// RosCom_SoftHandSync
struct sRosCom_SoftHandSync {
  RosCom_SoftHandSync* base;
  ros::NodeHandle nh;
  ros::Publisher pub_shReference;
};

void RosCom_SoftHandSync::open() {
  rosCheckInit();
  self = make_unique<sRosCom_SoftHandSync>();
  self->base=this;
  self->pub_shReference = self->nh.advertise<std_msgs::String>("/softhand/grasp_ref", 1);
}

void RosCom_SoftHandSync::step() {
  SoftHandMsg shm = sh_ref.get();
  std_msgs::String refs;
  refs.data = shm.soft_hand_cmd.p;
  self->pub_shReference.publish(refs);
}

void RosCom_SoftHandSync::close() {
  self->nh.shutdown();
  self.reset();
}

//===========================================================================
#else // RAI_ROS no defined

void RosCom_Spinner::open() { NICO }
void RosCom_Spinner::step() { NICO }
void RosCom_Spinner::close() { NICO }

void RosCom_ControllerSync::open() { NICO }
void RosCom_ControllerSync::step() { NICO }
void RosCom_ControllerSync::close() { NICO }

void RosCom_ForceSensorSync::open() { NICO }
void RosCom_ForceSensorSync::step() { NICO }
void RosCom_ForceSensorSync::close() { NICO }
#endif

//REGISTER_MODULE(RosCom_Spinner)
//REGISTER_MODULE(RosCom_ControllerSync)
//REGISTER_MODULE(RosCom_KinectSync)
//REGISTER_MODULE(RosCom_HeadCamsSync)
//REGISTER_MODULE(RosCom_ArmCamsSync)

