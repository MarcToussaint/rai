/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_ROS

#include "roscom.h"
#include "spinner.h"
#include "../Kin/frame.h"

#ifdef RAI_PCL
#  include <pcl/point_cloud.h>
#  include <pcl_conversions/pcl_conversions.h>
#endif

Singleton<RosCom_Spinner> singletonRosSpinner;

RosCom::RosCom(const char* node_name) {
  rosCheckInit(node_name);
  singletonRosSpinner();
//  spinner = new RosCom_Spinner(node_name);
  rai::wait(.1);
}

RosCom::~RosCom() {
}

bool rosOk() {
  return ros::ok();
}

void rosCheckInit(const char* node_name) {
// TODO make static variables to singleton
  static Mutex mutex;
  static bool inited = false;

  if(rai::getParameter<bool>("useRos", true)) {
    mutex.lock(RAI_HERE);
    if(!inited) {
      rai::String nodeName = rai::getParameter<rai::String>("rosNodeName", STRING(node_name));
      ros::init(rai::argc, rai::argv, nodeName.p, ros::init_options::NoSigintHandler);
      inited = true;
    }
    mutex.unlock();
  }
}

RosInit::RosInit(const char* node_name) {
  rosCheckInit(node_name);
}

std_msgs::String conv_string2string(const rai::String& str) {
  std_msgs::String msg;
  if(str.N) msg.data = str.p;
  return msg;
}

rai::String conv_string2string(const std_msgs::String& msg) {
  return rai::String(msg.data);
}

std_msgs::String conv_stringA2string(const StringA& strs) {
  std_msgs::String msg;
  for(const rai::String& str:strs)
    if(str.N) { msg.data += ", ";  msg.data += str.p; }
  return msg;
}

rai::Transformation conv_transform2transformation(const tf::Transform& trans) {
  rai::Transformation X;
  X.setZero();
  tf::Quaternion q = trans.getRotation();
  tf::Vector3 t = trans.getOrigin();
  X.rot.set(q.w(), q.x(), q.y(), q.z());
  X.pos.set(t.x(), t.y(), t.z());
  return X;
}

rai::Transformation conv_transform2transformation(const geometry_msgs::Transform& trans) {
  rai::Transformation X;
  X.setZero();
  X.rot.set(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z);
  X.pos.set(trans.translation.x, trans.translation.y, trans.translation.z);
  return X;
}

rai::Transformation conv_pose2transformation(const geometry_msgs::Pose& pose) {
  rai::Transformation X;
  X.setZero();
  auto& q = pose.orientation;
  auto& t = pose.position;
  X.rot.set(q.w, q.x, q.y, q.z);
  X.pos.set(t.x, t.y, t.z);
  return X;
}

geometry_msgs::Pose conv_transformation2pose(const rai::Transformation& transform) {
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

geometry_msgs::Transform conv_transformation2transform(const rai::Transformation& transformation) {
  geometry_msgs::Transform transform;
  transform.translation.x = transformation.pos.x;
  transform.translation.y  = transformation.pos.y;
  transform.translation.z  = transformation.pos.z;
  transform.rotation.x = transformation.rot.x;
  transform.rotation.y = transformation.rot.y;
  transform.rotation.z = transformation.rot.z;
  transform.rotation.w = transformation.rot.w;
  return transform;
}

rai::Vector conv_point2vector(const geometry_msgs::Point& p) {
  return rai::Vector(p.x, p.y, p.z);
}

rai::Quaternion conv_quaternion2quaternion(const geometry_msgs::Quaternion& q) {
  return rai::Quaternion(q.w, q.x, q.y, q.z);
}

void conv_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped& pose) {
  q({qIndex, qIndex+3}) = conv_pose2transXYPhi(pose);
}

arr conv_pose2transXYPhi(const geometry_msgs::PoseWithCovarianceStamped& pose) {
  auto& _quat=pose.pose.pose.orientation;
  auto& _pos=pose.pose.pose.position;
  rai::Quaternion quat(_quat.w, _quat.x, _quat.y, _quat.z);
  rai::Vector pos(_pos.x, _pos.y, _pos.z);

  double angle;
  rai::Vector rotvec;
  quat.getRad(angle, rotvec);
  return ARR(pos(0), pos(1), rai::sign(rotvec(2)) * angle);
}

timespec conv_time2timespec(const ros::Time& time) {
  return {time.sec, time.nsec};
}

double conv_time2double(const ros::Time& time) {
  return (double)(time.sec) + 1e-9d*(double)(time.nsec);
}

rai::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener) {
  rai::Transformation X;
  X.setZero();
  try {
    tf::StampedTransform transform;
    listener.lookupTransform(from, to, ros::Time(0), transform);
    X = conv_transform2transformation(transform);
  } catch(tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}

bool ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener, rai::Transformation& result) {
  result.setZero();
  try {
    tf::StampedTransform transform;
    listener.lookupTransform(from, to, ros::Time(0), transform);
    result = conv_transform2transformation(transform);
  } catch(tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  return true;
}

rai::Transformation ros_getTransform(const std::string& from, const std_msgs::Header& to, tf::TransformListener& listener, tf::Transform* returnRosTransform) {
  rai::Transformation X;
  X.setZero();
  try {
    tf::StampedTransform transform;
    listener.waitForTransform(from, to.frame_id, to.stamp, ros::Duration(0.05));
    listener.lookupTransform(from, to.frame_id, to.stamp, transform);
    X = conv_transform2transformation(transform);
    if(returnRosTransform) *returnRosTransform=transform;
  } catch(tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts) {
  uint n=pts.size();
  arr P(n, 3);
  for(uint i=0; i<n; i++) {
    P(i, 0) = pts[i].x;
    P(i, 1) = pts[i].y;
    P(i, 2) = pts[i].z;
  }
  return P;
}

arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts) {
  uint n=pts.size();
  arr P(n, 3);
  for(uint i=0; i<n; i++) {
    P(i, 0) = pts[i].r;
    P(i, 1) = pts[i].g;
    P(i, 2) = pts[i].b;
  }
  return P;
}

std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts) {
  uint n=pts.d0;
  std::vector<geometry_msgs::Point> P(n);
  for(uint i=0; i<n; i++) {
    P[i].x = pts(i, 0);
    P[i].y = pts(i, 1);
    P[i].z = pts(i, 2);
  }
  return P;
}

arr conv_wrench2arr(const geometry_msgs::WrenchStamped& msg) {
  auto& f=msg.wrench.force;
  auto& t=msg.wrench.torque;
  return ARR(f.x, f.y, f.z, t.x, t.y, t.z);
}

byteA conv_image2byteA(const sensor_msgs::Image& msg) {
  uint channels = msg.data.size()/(msg.height*msg.width);
  byteA img;
  if(channels==4) {
    img=conv_stdvec2arr<byte>(msg.data).reshape(msg.height, msg.width, 4);
  } else {
    img=conv_stdvec2arr<byte>(msg.data).reshape(msg.height, msg.width, 3);
    swap_RGB_BGR(img);
  }
  return img;
}

floatA conv_imageu162floatA(const sensor_msgs::Image& msg) {
  CHECK_EQ(msg.encoding, "16UC1", "wrong image encoding");
  byteA data = conv_stdvec2arr<byte>(msg.data);
  uint16A ref((const uint16_t*)data.p, data.N/2);
  //  uint16A ref((const uint16_t*)msg.data.data(), msg.data.size()/2);
  ref.reshape(msg.height, msg.width);
  floatA img(ref.d0, ref.d1);
  for(uint i=0; i<img.N; i++) img.elem(i) = 0.001f*float(ref.elem(i));
  return img;
}

floatA conv_imageFloat32_floatA(const sensor_msgs::Image& msg) {
  CHECK_EQ(msg.encoding, "32FC1", "wrong image encoding");
  floatA x((const float*)msg.data.data(), msg.data.size()/4);
  x.reshape(msg.height, msg.width);
  return x;
}

uint16A conv_image2uint16A(const sensor_msgs::Image& msg) {
  byteA data = conv_stdvec2arr<byte>(msg.data);
  uint16A ref((const uint16_t*)data.p, data.N/2);
  return ref.reshape(msg.height, msg.width);
}

floatA conv_laserScan2arr(const sensor_msgs::LaserScan& msg) {
  floatA data = conv_stdvec2arr<float>(msg.ranges);
  return data;
}

#ifdef RAI_PCL
PclC conv_pointcloud22pcl(const sensor_msgs::PointCloud2& msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg, pcl_pc2);
  LOG(0) <<"size=" <<pcl_pc2.data.size();
  PclC cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);
  LOG(0) <<"size=" <<cloud.size();
  return cloud;
}
#endif

CtrlMsg conv_JointState2CtrlMsg(const rai_msgs::JointState& msg) {
  return CtrlMsg(conv_stdvec2arr(msg.q), conv_stdvec2arr(msg.qdot), conv_stdvec2arr(msg.fL), conv_stdvec2arr(msg.fR), conv_stdvec2arr(msg.u_bias), conv_stdvec2arr(msg.fL_err), conv_stdvec2arr(msg.fR_err));

}

arr conv_JointState2arr(const sensor_msgs::JointState& msg) {
  return conv_stdvec2arr(msg.position);
}

rai_msgs::JointState conv_CtrlMsg2JointState(const CtrlMsg& ctrl) {
  rai_msgs::JointState jointState;
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

rai::Configuration conv_MarkerArray2Configuration(const visualization_msgs::MarkerArray& markers) {
  rai::Configuration world;
  tf::TransformListener listener;
  for(const visualization_msgs::Marker& marker:markers.markers) {
    rai::String name;
    name <<"obj" <<marker.id;
    rai::Shape* s = world.getFrameByName(name)->shape;
    if(!s) {
      rai::Frame* f = new rai::Frame(world);
      f->name=name;
      s = new rai::Shape(*f);
      if(marker.type==marker.CYLINDER) {
        s->type() = rai::ST_cylinder;
      } else if(marker.type==marker.POINTS) {
        s->type() = rai::ST_mesh;
        s->mesh().V = conv_points2arr(marker.points);
        s->mesh().C = conv_colors2arr(marker.colors);
      } else NIY;
    }
    s->size(0) = marker.scale.x;
    s->size(1) = marker.scale.y;
    s->size(2) = marker.scale.z;
    s->size(3) = .25*(marker.scale.x+marker.scale.y);
    s->frame.set_X() = ros_getTransform("/base_link", marker.header, listener) * conv_pose2transformation(marker.pose);
  }
  return world;
}

std_msgs::Float32MultiArray conv_floatA2Float32Array(const floatA& x) {
  std_msgs::Float32MultiArray msg;
  msg.data = conv_arr2stdvec<float>(x);
  msg.layout.dim.resize(x.nd);
  for(uint i=0; i<x.nd; i++) msg.layout.dim[i].size=x.dim(i);
  return msg;
}

std_msgs::Float64MultiArray conv_arr2Float64Array(const arr& x) {
  std_msgs::Float64MultiArray msg;
  msg.data = conv_arr2stdvec<double>(x);
  msg.layout.dim.resize(x.nd);
  for(uint i=0; i<x.nd; i++) msg.layout.dim[i].size=x.dim(i);
  return msg;
}

floatA conv_Float32Array2FloatA(const std_msgs::Float32MultiArray& msg) {
  floatA x = conv_stdvec2arr<float>(msg.data);
  uint nd = msg.layout.dim.size();
  if(nd==2) x.reshape(msg.layout.dim[0].size, msg.layout.dim[1].size);
  if(nd==3) x.reshape(msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size);
  if(nd>3) NIY;
  return x;
}

arr conv_Float32Array2arr(const std_msgs::Float32MultiArray& msg) {
  floatA f = conv_Float32Array2FloatA(msg);
  arr x;
  copy(x, f);
  return x;
}

rai_msgs::arr conv_arr2arr(const arr& x) {
  rai_msgs::arr y;
  y.dim = x.dim();
  y.data = x;
  return y;
}

arr conv_arr2arr(const rai_msgs::arr& x) {
  arr y;
  y = conv_stdvec2arr(x.data);
  y.reshape(conv_stdvec2arr(x.dim));
  return y;
}

StringA conv_StringA2StringA(const rai_msgs::StringA& x) {
  StringA y(x.data.size());
  for(uint i=0; i<y.N; i++) y.elem(i) = x.data[i].c_str();
  return y;
}

rai_msgs::StringA conv_StringA2StringA(const StringA& x) {
  rai_msgs::StringA y;
  y.data.resize(x.N);
  for(uint i=0; i<x.N; i++) y.data[i] = x.elem(i).p;
  return y;
}

std::vector<std::string> conv_StringA2stdStringVec(const StringA& x) {
  std::vector<std::string> y;
  y.resize(x.N);
  for(uint i=0; i<x.N; i++) y[i] = x.elem(i).p;
  return y;
}

StringA conv_stdStringVec2StringA(const std::vector<std::string>& x) {
  StringA y(x.size());
  for(uint i=0; i<y.N; i++) y.elem(i) = x[i].c_str();
  return y;
}

std_msgs::Float64 conv_double2Float64(const double& x) {
  std_msgs::Float64 y;
  y.data = x;
  return y;
}

//===========================================================================

visualization_msgs::Marker conv_Shape2Marker(const rai::Shape& sh) {
  visualization_msgs::Marker new_marker;
  new_marker.header.stamp = ros::Time::now();
  new_marker.header.frame_id = "map";
  new_marker.ns = "roopi";
  new_marker.id = sh.frame.ID;
  new_marker.action = visualization_msgs::Marker::ADD;
  new_marker.lifetime = ros::Duration();
  new_marker.pose = conv_transformation2pose(sh.frame.ensure_X());
  new_marker.color.r = 0.0f;
  new_marker.color.g = 1.0f;
  new_marker.color.b = 0.0f;
  new_marker.color.a = 1.0f;

#if 0
  switch(sh.type) {
    case rai::ST_box: {
      new_marker.type = visualization_msgs::Marker::CUBE;
      new_marker.scale.x = .001 * sh.size(0);
      new_marker.scale.y = .001 * sh.size(1);
      new_marker.scale.z = .001 * sh.size(2);
    } break;
    case rai::ST_mesh: {
      new_marker.type = visualization_msgs::Marker::POINTS;
      new_marker.points = conv_arr2points(sh.mesh.V);
      new_marker.scale.x = .001;
      new_marker.scale.y = .001;
      new_marker.scale.z = .001;
    } break;
//      ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
    default: break;
  }
#else
  new_marker.type = visualization_msgs::Marker::SPHERE;
  new_marker.scale.x = .1;
  new_marker.scale.y = .1;
  new_marker.scale.z = .1;

#endif

  return new_marker;
}

visualization_msgs::MarkerArray conv_Kin2Markers(const rai::Configuration& K) {
  visualization_msgs::MarkerArray M;
  for(rai::Frame* f : K.frames) M.markers.push_back(conv_Shape2Marker(*f->shape));
//  M.header.frame_id = "1";
  return M;
}

#endif

