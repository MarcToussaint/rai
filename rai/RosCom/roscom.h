/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#ifdef RAI_ROS

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include "../rai_msgs/MotionReference.h"
#include "../rai_msgs/StringA.h"
#include <std_msgs/Float64.h>

//===========================================================================

#include "../Core/thread.h"
#include "../Core/array.h"
#include "../Geo/geo.h"
#include "../Kin/kin.h"
#include "../Control/ctrlMsg.h"
#include "rai_msgs/JointState.h"
#ifdef RAI_PCL
#  include <Perception/pcl.h>
#endif

/*
 * TODO:
 * a single RosCom class; constructor checks rosInit and spawns singleton spinner
 * has publish/subscribe(var&) method, stores all pub/subs and cleans them on destruction
 */

//===========================================================================
//
// utils
//

void rosCheckInit(const char* node_name="rai_node");
bool rosOk();
struct RosInit { RosInit(const char* node_name="rai_node"); };

//-- ROS <--> RAI
std_msgs::String    conv_string2string(const rai::String&);
rai::String         conv_string2string(const std_msgs::String&);
std_msgs::String    conv_stringA2string(const StringA& strs);
rai::Transformation conv_transform2transformation(const tf::Transform&);
rai::Transformation conv_transform2transformation(const geometry_msgs::Transform&);
rai::Transformation conv_pose2transformation(const geometry_msgs::Pose&);
rai::Vector         conv_point2vector(const geometry_msgs::Point& p);
rai::Quaternion     conv_quaternion2quaternion(const geometry_msgs::Quaternion& q);
void                conv_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped& pose);
arr                 conv_pose2transXYPhi(const geometry_msgs::PoseWithCovarianceStamped& pose);
double              conv_time2double(const ros::Time& time);
timespec            conv_time2timespec(const ros::Time&);
arr                 conv_wrench2arr(const geometry_msgs::WrenchStamped& msg);
byteA               conv_image2byteA(const sensor_msgs::Image& msg);
uint16A             conv_image2uint16A(const sensor_msgs::Image& msg);
floatA              conv_imageu162floatA(const sensor_msgs::Image& msg);
floatA              conv_imageFloat32_floatA(const sensor_msgs::Image& msg);

floatA              conv_laserScan2arr(const sensor_msgs::LaserScan& msg);
#ifdef RAI_PCL
PclC                 conv_pointcloud22pcl(const sensor_msgs::PointCloud2& msg);
#endif
arr                 conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr                 conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
CtrlMsg             conv_JointState2CtrlMsg(const rai_msgs::JointState& msg);
arr                 conv_JointState2arr(const sensor_msgs::JointState& msg);
rai::Configuration conv_MarkerArray2Configuration(const visualization_msgs::MarkerArray& markers);
std_msgs::Float32MultiArray conv_floatA2Float32Array(const floatA&);
std_msgs::Float64MultiArray conv_arr2Float64Array(const arr&);
arr conv_arr2arr(const rai_msgs::arr& x);
rai_msgs::arr conv_arr2arr(const arr& x);
StringA conv_StringA2StringA(const rai_msgs::StringA& x);
rai_msgs::StringA conv_StringA2StringA(const StringA& x);
std::vector<std::string> conv_StringA2stdStringVec(const StringA& x);
StringA conv_stdStringVec2StringA(const std::vector<std::string>& x);
std_msgs::Float64 conv_double2Float64(const double& x);

//-- RAI -> ROS
geometry_msgs::Pose conv_transformation2pose(const rai::Transformation&);
geometry_msgs::Transform conv_transformation2transform(const rai::Transformation&);
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);
rai_msgs::JointState   conv_CtrlMsg2JointState(const CtrlMsg& ctrl);
floatA conv_Float32Array2FloatA(const std_msgs::Float32MultiArray&);
arr conv_Float32Array2arr(const std_msgs::Float32MultiArray& msg);
visualization_msgs::Marker conv_Shape2Marker(const rai::Shape& sh);
visualization_msgs::MarkerArray conv_Kin2Markers(const rai::Configuration& K);

//-- get transformations
rai::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);
rai::Transformation ros_getTransform(const std::string& from, const std_msgs::Header& to, tf::TransformListener& listener, tf::Transform* returnRosTransform=nullptr);
bool ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener, rai::Transformation& result);

struct SubscriberType { virtual ~SubscriberType() {} }; ///< if types derive from RootType, more tricks are possible

//===========================================================================
//
// subscribing a message directly into an Var
//

template<class msg_type>
struct Subscriber : SubscriberType {
  Var<msg_type>& var;
  ros::NodeHandle* nh=nullptr;
  ros::Subscriber sub;
  uint revision=0;
  Subscriber(Var<msg_type>& _var, const char* topic_name=nullptr)
    : var(_var) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!topic_name) topic_name = var.name();
//      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {var.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(msg_type).name() <<"> into var '" <<var.name() <<'\'';
      nh = new ros::NodeHandle;
      sub  = nh->subscribe(topic_name, 100, &Subscriber::callback, this);
    }
  }
  ~Subscriber() {
    if(nh) delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) { var.set() = *msg;  revision++; }
};

//===========================================================================
//
// subscribing a message into an RAI-type-var via a conv_* function
//

template<class msg_type>
struct Publisher : Thread {
  Var<msg_type> var;
  ros::NodeHandle* nh;
  ros::Publisher pub;
  rai::String topic_name;

  Publisher(const Var<msg_type>& _var)
    : Thread(STRING("Publisher_"<<_var.name()), -1.),
      var(this, _var, true),
      nh(nullptr) {
    if(rai::getParameter<bool>("useRos", true)) {
      rai::String topic_name = var.name();
      LOG(0) <<"publishing to topic '" <<topic_name <<"' <" <<typeid(msg_type).name() <<">";
      nh = new ros::NodeHandle;
      pub = nh->advertise<msg_type>(topic_name.p, 1);
      rai::wait(.1); //I hate this -- no idea why the publisher isn't ready right away..
    }
  }
  ~Publisher() {
    threadClose();
    pub.shutdown();
    if(nh) delete nh;
  }
  void open() {}
  void close() {}
  void step() {
    if(nh) {
      pub.publish(var.get()());
//      LOG(0) <<"publishing to topic '" <<topic_name <<"' revision " <<var.getRevision() <<" of variable '" <<var.name <<'\'';
    }
  }
};

//===========================================================================
//
// subscribing a message into an RAI-type-var via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConv : SubscriberType {
  Var<var_type> var;
  Var<rai::Transformation>* frame;
  ros::NodeHandle* nh;
  ros::Subscriber sub;
  tf::TransformListener* listener;
  SubscriberConv(Var<var_type>& _var, const char* topic_name=nullptr, Var<rai::Transformation>* _frame=nullptr)
    : var(nullptr, _var), frame(_frame), nh(nullptr), listener(nullptr) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!topic_name) topic_name = var.name();
      nh = new ros::NodeHandle;
      if(frame) listener = new tf::TransformListener;
//      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {var.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into var '" <<var.name() <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
    }
  }
  SubscriberConv(const char* topic_name, const char* var_name, Var<rai::Transformation>* _frame=nullptr)
    : var(nullptr, var_name), frame(_frame), nh(nullptr), listener(nullptr) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!topic_name) topic_name = var_name;
      nh = new ros::NodeHandle;
      if(frame) listener = new tf::TransformListener;
//      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {var.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into var '" <<var.name() <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
    }
  }
  ~SubscriberConv() {
    if(listener) delete listener;
    if(nh) delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) {
    double time=conv_time2double(msg->header.stamp);
    var.set(time) = conv(*msg);
    if(frame && listener) {
      frame->set(time) = ros_getTransform("/base_link", msg->header, *listener);
    }
  }
};

//===========================================================================
//
// subscribing a message into an RAI-type-var via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConvNoHeader : SubscriberType {
  Var<var_type> var;
  ros::NodeHandle* nh;
  ros::Subscriber sub;
  SubscriberConvNoHeader(Var<var_type>& _var, const char* topic_name=nullptr)
    : var(nullptr, _var), nh(nullptr) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!topic_name) topic_name = var.name();
      nh = new ros::NodeHandle;
//      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {var.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into var '" <<var.name() <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConvNoHeader::callback, this);
    }
  }
  SubscriberConvNoHeader(const char* var_name, const char* topic_name=nullptr)
    : var(nullptr, var_name), nh(nullptr) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!topic_name) topic_name = var_name;
      nh = new ros::NodeHandle;
//      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {var.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into var '" <<var.name() <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConvNoHeader::callback, this);
    }
  }

  ~SubscriberConvNoHeader() {
    if(nh) delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) {
    var.set() = conv(*msg);
  }
};

//===========================================================================
//
// subscribing a message into an RAI-type-var via a conv_* function
//

template<class msg_type, class var_type, msg_type conv(const var_type&)>
struct PublisherConv : Thread {
  Var<var_type> var;
  ros::NodeHandle* nh;
  ros::Publisher pub;
  rai::String topic_name;

  PublisherConv(const Var<var_type>& _var, const char* _topic_name=nullptr, double beatIntervalSec=-1.)
    : Thread(STRING("Publisher_"<<_var.name() <<"->" <<_topic_name), beatIntervalSec),
      var(this, _var, beatIntervalSec<0.),
      nh(nullptr),
      topic_name(_topic_name) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!_topic_name) topic_name = var.name();
      LOG(0) <<"publishing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> from var '" <<var.name() <<'\'';
      nh = new ros::NodeHandle;
      pub = nh->advertise<msg_type>(topic_name.p, 1);
      rai::wait(.1); //I hate this -- no idea why the publisher isn't ready right away..
    }
  }
  PublisherConv(const char* var_name, const char* _topic_name=nullptr, double beatIntervalSec=-1.)
    : Thread(STRING("Publisher_"<<var_name <<"->" <<_topic_name), beatIntervalSec),
      var(this, var_name, beatIntervalSec<0.),
      nh(nullptr),
      topic_name(_topic_name) {
    if(rai::getParameter<bool>("useRos", true)) {
      if(!_topic_name) topic_name = var_name;
      LOG(0) <<"publishing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> from var '" <<var.name() <<'\'';
      nh = new ros::NodeHandle;
      pub = nh->advertise<msg_type>(topic_name.p, 1);
      rai::wait(.1); //I hate this -- no idea why the publisher isn't ready right away..
    }
  }
  ~PublisherConv() {
    threadClose();
    pub.shutdown();
    if(nh) delete nh;
  }
  void open() {}
  void step() {
    if(nh) {
      pub.publish(conv(var.get()));
//      LOG(0) <<"publishing to topic '" <<topic_name <<"' revision " <<var.getRevision() <<" of variable '" <<var.name <<'\'';
    }
  }
  void close() {}
};

//===========================================================================

struct RosCom {
  RosCom(const char* node_name="rai_module");
  ~RosCom();
  template<class T, class P> void publish(std::shared_ptr<P>& pub, Var<T>& v, bool wait=true) {
    pub = make_shared<P>(v);
    if(wait) {
      while(!pub->pub.getNumSubscribers()) rai::wait(.05);
    }
  }
  template<class T, class S> void subscribe(std::shared_ptr<S>& sub, Var<T>& v, bool wait=true) {
    sub = make_shared<S>(v);
    if(wait) {
      while(!sub->sub.getNumPublishers()) rai::wait(.05);
    }
  }
  template<class T> std::shared_ptr<Subscriber<T>> subscribe(Var<T>& v) { return make_shared<Subscriber<T>>(v); }
  template<class T> std::shared_ptr<Publisher<T>> publish(Var<T>& v) { return make_shared<Publisher<T>>(v); }
};

#else

#include "../Core/util.h"
inline void rosCheckInit(const char* node_name="rai_node") { NICO }

#endif
