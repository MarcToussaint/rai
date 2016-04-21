#ifdef MLR_ROS

#include "roscom.h"

bool rosOk(){
  return ros::ok();
}

void rosCheckInit(const char* node_name){
// TODO make static variables to singleton
  static Mutex mutex;
  static bool inited = false;

  if(mlr::getParameter<bool>("useRos", false)){
    mutex.lock();
    if(!inited) {
      ros::init(mlr::argc, mlr::argv, node_name, ros::init_options::NoSigintHandler);
      inited = true;
    }
    mutex.unlock();
  }
}

std_msgs::String conv_string2string(const mlr::String& str){
  std_msgs::String msg;
  if(str.N) msg.data = str.p;
  return msg;
}

mlr::String conv_string2string(const std_msgs::String& msg){
  return mlr::String(msg.data);
}

std_msgs::String conv_stringA2string(const StringA& strs){
  std_msgs::String msg;
  for(const mlr::String& str:strs)
    if(str.N){ msg.data += ", ";  msg.data += str.p; }
  return msg;
}

ors::Transformation conv_transform2transformation(const tf::Transform &trans){
  ors::Transformation X;
  X.setZero();
  tf::Quaternion q = trans.getRotation();
  tf::Vector3 t = trans.getOrigin();
  X.rot.set(q.w(), q.x(), q.y(), q.z());
  X.pos.set(t.x(), t.y(), t.z());
  return X;
}

ors::Transformation conv_pose2transformation(const geometry_msgs::Pose &pose){
  ors::Transformation X;
  X.setZero();
  auto& q = pose.orientation;
  auto& t = pose.position;
  X.rot.set(q.w, q.x, q.y, q.z);
  X.pos.set(t.x, t.y, t.z);
  return X;
}

geometry_msgs::Pose conv_transformation2pose(const ors::Transformation& transform){
  geometry_msgs::Pose pose;
  pose.position.x = transform.pos.x;
  pose.position.y = transform.pos.y;
  pose.position.z = transform.pos.z;
  pose.orientation.x = transform.rot.x;
  pose.orientation.y = transform.rot.y;
  pose.orientation.z = transform.rot.z;
  pose.orientation.w = transform.rot.w;
  return pose;
}

ors::Vector conv_point2vector(const geometry_msgs::Point& p){
  return ors::Vector(p.x, p.y, p.z);
}

ors::Quaternion conv_quaternion2quaternion(const geometry_msgs::Quaternion& q){
  return ors::Quaternion(q.w, q.x, q.y, q.z);
}

void conv_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped& pose){
  q.subRef(qIndex, qIndex+3) = conv_pose2transXYPhi(pose);
}

arr conv_pose2transXYPhi(const geometry_msgs::PoseWithCovarianceStamped& pose){
  auto& _quat=pose.pose.pose.orientation;
  auto& _pos=pose.pose.pose.position;
  ors::Quaternion quat(_quat.w, _quat.x, _quat.y, _quat.z);
  ors::Vector pos(_pos.x, _pos.y, _pos.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  return ARR(pos(0), pos(1), mlr::sign(rotvec(2)) * angle);
}

timespec conv_time2timespec(const ros::Time& time){
  return {time.sec, time.nsec};
}

double conv_time2double(const ros::Time& time){
  return (double)(time.sec) + 1e-9d*(double)(time.nsec);
}

ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener){
  ors::Transformation X;
  X.setZero();
  try{
    tf::StampedTransform transform;
    listener.lookupTransform(from, to, ros::Time(0), transform);
    X = conv_transform2transformation(transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}

ors::Transformation ros_getTransform(const std::string& from, const std_msgs::Header& to, tf::TransformListener& listener){
  ors::Transformation X;
  X.setZero();
  try{
    tf::StampedTransform transform;
    listener.waitForTransform(from, to.frame_id, to.stamp, ros::Duration(0.05));
    listener.lookupTransform(from, to.frame_id, to.stamp, transform);
    X = conv_transform2transformation(transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].x;
    P(i,1) = pts[i].y;
    P(i,2) = pts[i].z;
  }
  return P;
}

arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].r;
    P(i,1) = pts[i].g;
    P(i,2) = pts[i].b;
  }
  return P;
}


std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts){
  uint n=pts.d0;
  std::vector<geometry_msgs::Point> P(n);
  for(uint i=0;i<n;i++){
    P[i].x = pts(i, 0);
    P[i].y = pts(i, 1);
    P[i].z = pts(i, 2);
  }
  return P;
}

arr conv_wrench2arr(const geometry_msgs::WrenchStamped& msg){
  auto& f=msg.wrench.force;
  auto& t=msg.wrench.torque;
  return ARR(f.x, f.y, f.z, t.x, t.y, t.z);
}

byteA conv_image2byteA(const sensor_msgs::Image& msg){
  return conv_stdvec2arr<byte>(msg.data).reshape(msg.height, msg.width, 3);
}

uint16A conv_image2uint16A(const sensor_msgs::Image& msg){
  byteA data = conv_stdvec2arr<byte>(msg.data);
  uint16A ref((const uint16_t*)data.p, data.N/2);
  return ref.reshape(msg.height, msg.width);
}

CtrlMsg conv_JointState2CtrlMsg(const marc_controller_pkg::JointState& msg){
  return CtrlMsg(conv_stdvec2arr(msg.q), conv_stdvec2arr(msg.qdot), conv_stdvec2arr(msg.fL), conv_stdvec2arr(msg.fR),conv_stdvec2arr(msg.u_bias), conv_stdvec2arr(msg.fL_err), conv_stdvec2arr(msg.fR_err));

}

arr conv_JointState2arr(const sensor_msgs::JointState& msg){
  return conv_stdvec2arr(msg.position);
}

marc_controller_pkg::JointState conv_CtrlMsg2JointState(const CtrlMsg& ctrl){
  marc_controller_pkg::JointState jointState;
  if(!ctrl.q.N) return jointState;
  jointState.q = conv_arr2stdvec(ctrl.q);
  jointState.qdot= conv_arr2stdvec(ctrl.qdot);
  jointState.fL = conv_arr2stdvec(ctrl.fL);
  jointState.fR = conv_arr2stdvec(ctrl.fR);
  jointState.u_bias = conv_arr2stdvec(ctrl.u_bias);
  jointState.Kp = conv_arr2stdvec(ctrl.Kp);
  jointState.Kd = conv_arr2stdvec(ctrl.Kd);
  jointState.Ki = conv_arr2stdvec(ctrl.Ki);
  jointState.KiFTL = conv_arr2stdvec(ctrl.KiFTL);
  jointState.KiFTR = conv_arr2stdvec(ctrl.KiFTR);
  jointState.J_ft_invL = conv_arr2stdvec(ctrl.J_ft_invL);
  jointState.J_ft_invR = conv_arr2stdvec(ctrl.J_ft_invR);
  jointState.velLimitRatio = ctrl.velLimitRatio;
  jointState.effLimitRatio = ctrl.effLimitRatio;
  jointState.intLimitRatio = ctrl.intLimitRatio;
  jointState.fL_gamma = ctrl.fL_gamma;
  jointState.fR_gamma = ctrl.fR_gamma;
  jointState.qd_filt = ctrl.qd_filt;
  jointState.fL_offset = conv_arr2stdvec(ctrl.fL_offset);
  jointState.fR_offset = conv_arr2stdvec(ctrl.fR_offset);
  return jointState;
}

ors::KinematicWorld conv_MarkerArray2KinematicWorld(const visualization_msgs::MarkerArray& markers){
  ors::KinematicWorld world;
  tf::TransformListener listener;
  for(const visualization_msgs::Marker& marker:markers.markers){
    mlr::String name;
    name <<"obj" <<marker.id;
    ors::Shape *s = world.getShapeByName(name);
    if(!s){
      s = new ors::Shape(world, NoBody);
      s->name=name;
      if(marker.type==marker.CYLINDER){
        s->type = ors::cylinderST;
      }else if(marker.type==marker.POINTS){
        s->type = ors::meshST;
        s->mesh.V = conv_points2arr(marker.points);
        s->mesh.C = conv_colors2arr(marker.colors);
      }else NIY;
    }
    s->size[0] = marker.scale.x;
    s->size[1] = marker.scale.y;
    s->size[2] = marker.scale.z;
    s->size[3] = .25*(marker.scale.x+marker.scale.y);
    s->X = s->rel = ros_getTransform("base_link", marker.header, listener) * conv_pose2transformation(marker.pose);
  }
  return world;
}

std_msgs::Float32MultiArray conv_floatA2Float32Array(const floatA &x){
  std_msgs::Float32MultiArray msg;
  msg.data = conv_arr2stdvec<float>(x);
  return msg;
}


floatA conv_Float32Array2FloatA(const std_msgs::Float32MultiArray &msg){
  floatA x;
  x = conv_stdvec2arr<float>(msg.data);
  return x;

}

//===========================================================================
//
// OLD
//


void PerceptionObjects2Ors::step(){
  perceptionObjects.readAccess();
  modelWorld.readAccess();

  for(visualization_msgs::Marker& marker : perceptionObjects().markers){
    mlr::String name;
    name <<"obj" <<marker.id;
    ors::Shape *s = modelWorld->getShapeByName(name);
    if(!s){
      s = new ors::Shape(modelWorld(), NoBody);
      if(marker.type==marker.CYLINDER){
        s->type = ors::cylinderST;
        s->size[3] = .5*(marker.scale.x+marker.scale.y);
        s->size[2] = marker.scale.z;
      }else if(marker.type==marker.POINTS){
        s->type = ors::meshST;
        s->mesh.V = conv_points2arr(marker.points);
        s->mesh.C = conv_colors2arr(marker.colors);
      }else NIY;
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
//  s = new sRosCom_ControllerSync;
//  s->base=this;
//  s->sub_jointState = s->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom_ControllerSync::joinstState_callback, s);
////  s->sub_odom = s->nh.subscribe("/robot_pose_ekf/odom_combined", 1, &sRosCom_ControllerSync::joinstState_callback, s);
//  s->pub_jointReference = s->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
//  //  s->sub_jointState = s->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
//  //  s->pub_jointReference = s->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
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
//  s->pub_jointReference.publish(jointRef);
//}

//void RosCom_ControllerSync::close(){
//  s->nh.shutdown();
//  delete s;
//}

//===========================================================================
// Helper function so sync ors with the real PR2
void initialSyncJointStateWithROS(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  //-- wait for first q observation!
  cout << "** Waiting for ROS message of joints for initial configuration.." << endl
       << "   If nothing is happening: is the controller running?" << endl;

  for (uint trials = 0; trials < 20; trials++) {
    ctrl_obs.var->waitForNextRevision();
    cout << "REMOTE joint dimension=" << ctrl_obs.get()->q.N << endl;
    cout << "LOCAL  joint dimension=" << world.q.N << endl;

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      cout << "** Updating world state" << endl;
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
    cout << "retrying..." << endl;
  }
  HALT("sync'ing real PR2 with simulated failed");
}

void syncJointStateWitROS(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  for (uint trials = 0; trials < 2; trials++) {
    ctrl_obs.var->waitForNextRevision();

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
  }
  HALT("sync'ing real PR2 with simulated failed");
}

//===========================================================================
// RosCom_KinectSync
//struct sRosCom_KinectSync{
//  RosCom_KinectSync *base;
//  ros::NodeHandle nh;
//  ros::Subscriber sub_rgb;
//  ros::Subscriber sub_depth;
//  tf::TransformListener listener;

//  void cb_rgb(const sensor_msgs::Image::ConstPtr& msg){
//    //  cout <<"** sRosCom_KinectSync callback" <<endl;
//    base->kinect_rgb.set( conv_time2double(msg->header.stamp) ) = conv_stdvec2arr(msg->data).reshape(msg->height, msg->width, 3);
//  }
//  void cb_depth(const sensor_msgs::Image::ConstPtr& msg){
//    //  cout <<"** sRosCom_KinectSync callback" <<endl;
//    byteA data = conv_stdvec2arr(msg->data);
//    uint16A ref((const uint16_t*)data.p, data.N/2);
//    ref.reshape(msg->height, msg->width);
//    double time=conv_time2double(msg->header.stamp);
//    base->kinect_depth.set( time ) = ref;
//    base->kinect_frame.set( time ) = ros_getTransform("/base_link", msg->header.frame_id, listener);
//  }
//};

//void RosCom_KinectSync::open(){
//  rosCheckInit();
//  s = new sRosCom_KinectSync;
//  s->base = this;
//  s->sub_rgb = s->nh.subscribe("/kinect_head/rgb/image_color", 1, &sRosCom_KinectSync::cb_rgb, s);
//  s->sub_depth = s->nh.subscribe("/kinect_head/depth/image_raw", 1, &sRosCom_KinectSync::cb_depth, s);
//}

//void RosCom_KinectSync::step(){
//}

//void RosCom_KinectSync::close(){
//  s->nh.shutdown();
//}


////===========================================================================
//// RosCom_ArmCamsSync
//struct sRosCom_ArmCamsSync{
//  RosCom_ArmCamsSync *base;
//  ros::NodeHandle nh;
//  ros::Subscriber sub_left;
//  ros::Subscriber sub_right;
//  void cb_left(const sensor_msgs::Image::ConstPtr& msg){
//    base->rgb_leftArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
//  }
//  void cb_right(const sensor_msgs::Image::ConstPtr& msg){
//    base->rgb_rightArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
//  }
//};

//void RosCom_ArmCamsSync::open(){
//  rosCheckInit();
//  s = new sRosCom_ArmCamsSync;
//  s->base = this;
//  s->sub_left  = s->nh.subscribe("/l_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_left, s);
//  s->sub_right = s->nh.subscribe("/r_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_right, s);
//}

//void RosCom_ArmCamsSync::step(){
//}

//void RosCom_ArmCamsSync::close(){
//  s->nh.shutdown();
//}

////===========================================================================
//// RosCom_ForceSensorSync
//struct sRosCom_ForceSensorSync{
//  RosCom_ForceSensorSync *base;
//  ros::NodeHandle nh;
//  ros::Subscriber sub_left;
//  ros::Subscriber sub_right;
//  void cb_left(const geometry_msgs::WrenchStamped::ConstPtr& msg){
//    const geometry_msgs::Vector3 &f=msg->wrench.force;
//    const geometry_msgs::Vector3 &t=msg->wrench.torque;
//    base->wrenchL.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
//  }
//  void cb_right(const geometry_msgs::WrenchStamped::ConstPtr& msg){
//    const geometry_msgs::Vector3 &f=msg->wrench.force;
//    const geometry_msgs::Vector3 &t=msg->wrench.torque;
//    base->wrenchR.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
//  }

//};

//void RosCom_ForceSensorSync::open(){
//  rosCheckInit();
//  s = new sRosCom_ForceSensorSync;
//  s->base = this;
//  s->sub_left  = s->nh.subscribe("/ft_sensor/ft_compensated", 1, &sRosCom_ForceSensorSync::cb_left, s);  // /ft/l_gripper_motor
////  s->sub_right = s->nh.subscribe("/ft_sensor/r_ft_compensated", 1, &sRosCom_ForceSensorSync::cb_right, s); // /ft/r_gripper_motor
//}

//void RosCom_ForceSensorSync::step(){
//}

//void RosCom_ForceSensorSync::close(){
//  s->nh.shutdown();
//}

////===========================================================================
//// RosCom_SoftHandSync
//struct sRosCom_SoftHandSync{
//  RosCom_SoftHandSync *base;
//  ros::NodeHandle nh;
//  ros::Publisher pub_shReference;
//};

//void RosCom_SoftHandSync::open(){
//  rosCheckInit();
//  s = new sRosCom_SoftHandSync;
//  s->base=this;
//  s->pub_shReference = s->nh.advertise<std_msgs::String>("/softhand/grasp_ref", 1);
//}

//void RosCom_SoftHandSync::step(){
//  SoftHandMsg shm = sh_ref.get();
//  std_msgs::String refs;
//  refs.data = shm.soft_hand_cmd.p;
//  s->pub_shReference.publish(refs);
//}

//void RosCom_SoftHandSync::close(){
//  s->nh.shutdown();
//  delete s;
//}


//===========================================================================
//#else // MLR_ROS no defined

//void RosCom_Spinner::open(){ NICO }
//void RosCom_Spinner::step(){ NICO }
//void RosCom_Spinner::close(){ NICO }

//void RosCom_ControllerSync::open(){ NICO }
//void RosCom_ControllerSync::step(){ NICO }
//void RosCom_ControllerSync::close(){ NICO }

//void RosCom_ForceSensorSync::open(){ NICO }
//void RosCom_ForceSensorSync::step(){ NICO }
//void RosCom_ForceSensorSync::close(){ NICO }

#endif

//REGISTER_MODULE(RosCom_Spinner)
//REGISTER_MODULE(RosCom_ControllerSync)
//REGISTER_MODULE(RosCom_KinectSync)
//REGISTER_MODULE(RosCom_HeadCamsSync)
//REGISTER_MODULE(RosCom_ArmCamsSync)





