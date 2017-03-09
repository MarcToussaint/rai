#pragma once

#ifndef MLR_ROS
# error "Sorry, you can include this only when compiling against ROS"
#endif

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

//===========================================================================

#include <Core/thread.h>
#include <Core/array.h>
#include <Geo/geo.h>
#include <Kin/kin.h>
#include <Control/ctrlMsg.h>
#include <ros_msg/JointState.h>
#include <PCL/conv.h>

//===========================================================================
//
// utils
//



void rosCheckInit(const char* node_name="pr2_module");
bool rosOk();
struct RosInit{ RosInit(const char* node_name="mlr_module"); };

//-- ROS <--> MLR
std_msgs::String    conv_string2string(const mlr::String&);
mlr::String         conv_string2string(const std_msgs::String&);
std_msgs::String    conv_stringA2string(const StringA& strs);
mlr::Transformation conv_transform2transformation(const tf::Transform&);
mlr::Transformation conv_transform2transformation(const geometry_msgs::Transform&);
mlr::Transformation conv_pose2transformation(const geometry_msgs::Pose&);
mlr::Vector         conv_point2vector(const geometry_msgs::Point& p);
mlr::Quaternion     conv_quaternion2quaternion(const geometry_msgs::Quaternion& q);
void                conv_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose);
arr                 conv_pose2transXYPhi(const geometry_msgs::PoseWithCovarianceStamped &pose);
double              conv_time2double(const ros::Time& time);
timespec            conv_time2timespec(const ros::Time&);
arr                 conv_wrench2arr(const geometry_msgs::WrenchStamped& msg);
byteA               conv_image2byteA(const sensor_msgs::Image& msg);
uint16A             conv_image2uint16A(const sensor_msgs::Image& msg);
Pcl                 conv_pointcloud22pcl(const sensor_msgs::PointCloud2& msg);
arr                 conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr                 conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
CtrlMsg             conv_JointState2CtrlMsg(const marc_controller_pkg::JointState& msg);
arr                 conv_JointState2arr(const sensor_msgs::JointState& msg);
mlr::KinematicWorld conv_MarkerArray2KinematicWorld(const visualization_msgs::MarkerArray& markers);
std_msgs::Float32MultiArray conv_floatA2Float32Array(const floatA&);

//-- MLR -> ROS
geometry_msgs::Pose conv_transformation2pose(const mlr::Transformation&);
geometry_msgs::Transform conv_transformation2transform(const mlr::Transformation&);
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);
marc_controller_pkg::JointState   conv_CtrlMsg2JointState(const CtrlMsg& ctrl);
floatA conv_Float32Array2FloatA(const std_msgs::Float32MultiArray&);

//-- get transformations
mlr::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);
mlr::Transformation ros_getTransform(const std::string& from, const std_msgs::Header& to, tf::TransformListener& listener);
bool ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener, mlr::Transformation& result);


struct SubscriberType { virtual ~SubscriberType() {} }; ///< if types derive from RootType, more tricks are possible

//===========================================================================
//
// subscribing a message directly into an Access
//

template<class msg_type>
struct Subscriber : SubscriberType {
  Access_typed<msg_type>& access;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  Subscriber(const char* topic_name, Access_typed<msg_type>& _access)
    : access(_access) {
    if(mlr::getParameter<bool>("useRos", false)){
      nh = new ros::NodeHandle;
      registry().newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(msg_type).name() <<"> into access '" <<access.name <<'\'';
      sub  = nh->subscribe( topic_name, 100, &Subscriber::callback, this);
    }
  }
  ~Subscriber(){
    delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) { access.set() = *msg; }
};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConv : SubscriberType {
  Access_typed<var_type> access;
  Access_typed<mlr::Transformation> *frame;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  tf::TransformListener *listener;
  SubscriberConv(const char* topic_name, Access_typed<var_type>& _access, Access_typed<mlr::Transformation> *_frame=NULL)
    : access(NULL, _access), frame(_frame), listener(NULL) {
    if(mlr::getParameter<bool>("useRos")){
      nh = new ros::NodeHandle;
      if(frame) listener = new tf::TransformListener;
      registry().newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into access '" <<access.name <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
    }
  }
  SubscriberConv(const char* topic_name, const char* var_name, Access_typed<mlr::Transformation> *_frame=NULL)
    : access(NULL, var_name), frame(_frame) {
    if(mlr::getParameter<bool>("useRos")){
      nh = new ros::NodeHandle;
      if(frame) listener = new tf::TransformListener;
      registry().newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into access '" <<access.name <<'\'';
      sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
    }
  }
  ~SubscriberConv(){
    if(listener) delete listener;
    delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) {
    double time=conv_time2double(msg->header.stamp);
    access.set( time ) = conv(*msg);
    if(frame && listener){
      frame->set( time ) = ros_getTransform("/base_link", msg->header.frame_id, *listener);
    }
  }
};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConvNoHeader : SubscriberType{
  Access_typed<var_type> access;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  SubscriberConvNoHeader(const char* topic_name, Access_typed<var_type>& _access)
    : access(NULL, _access) {
    if(mlr::getParameter<bool>("useRos")){
      nh = new ros::NodeHandle;
      registry().newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into access '" <<access.name <<'\'';
      sub = nh->subscribe( topic_name, 1, &SubscriberConvNoHeader::callback, this);
    }
  }
  SubscriberConvNoHeader(const char* topic_name, const char* var_name)
    : access(NULL, var_name) {
    if(mlr::getParameter<bool>("useRos")){
      nh = new ros::NodeHandle;
      registry().newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
      LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> into access '" <<access.name <<'\'';
      sub = nh->subscribe( topic_name, 1, &SubscriberConvNoHeader::callback, this);
    }
  }

  ~SubscriberConvNoHeader(){
    delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) {
    access.set() = conv(*msg);
  }
};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, msg_type conv(const var_type&)>
struct PublisherConv : Thread {
  Access_typed<var_type> access;
  ros::NodeHandle *nh;
  ros::Publisher pub;
  const char* topic_name;

  PublisherConv(const char* _topic_name, Access_typed<var_type>& _access)
      : Thread(STRING("Publisher_"<<_access.name <<"->" <<_topic_name), -1),
        access(this, _access, true),
        nh(NULL),
        topic_name(_topic_name){
    if(mlr::getParameter<bool>("useRos")){
      LOG(0) <<"publishing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> from access '" <<access.name <<'\'';
      nh = new ros::NodeHandle;
      threadOpen();
    }
  }
  PublisherConv(const char* _topic_name, const char* var_name)
      : Thread(STRING("Publisher_"<<var_name <<"->" <<_topic_name), -1),
        access(this, var_name, true),
        nh(NULL),
        topic_name(_topic_name){
    if(mlr::getParameter<bool>("useRos")){
      LOG(0) <<"publishing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> from access '" <<access.name <<'\'';
      nh = new ros::NodeHandle;
      threadOpen();
    }
  }
  ~PublisherConv(){
    threadClose();
    pub.shutdown();
    if(nh) delete nh;
  }
  void open(){
    if(nh) pub = nh->advertise<msg_type>(topic_name, 1);
  }
  void step(){
    if(nh) pub.publish(conv(access.get()));
  }
  void close(){
  }
};


//===========================================================================
//
// variable declarations
//


//-- a basic message type for communication with the soft hand controller
struct SoftHandMsg{
  mlr::String soft_hand_cmd;
  SoftHandMsg(){}
  SoftHandMsg(const mlr::String soft_hand_cmd)
    :soft_hand_cmd(soft_hand_cmd){}
};
//inline void operator<<(ostream& os, const CtrlMsg& m){ os<<"BLA"; }
//inline void operator>>(istream& os, CtrlMsg& m){  }


//===========================================================================
//
// modules
//
//===========================================================================
/// This module only calls ros:spinOnce() in step() and loops full speed -- to sync the process with the ros server



// Helper function so sync ors with the real PR2
/**
 * This starts the initial sync of the world with ctrl_obs from the robot.
 *
 * This is verbose (helps debugging) and retries to connect to the robot multiple times.
 *
 * If useRos==false then nothing happens.
 */
void initialSyncJointStateWithROS(mlr::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

/**
 * Sync the world with ctrl_obs from the robot.
 *
 * If useRos==false then nothing happens.
 */
void syncJointStateWitROS(mlr::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

//===========================================================================

struct PerceptionObjects2Ors : Thread {
  Access_typed<visualization_msgs::MarkerArray> perceptionObjects;
  Access_typed<mlr::KinematicWorld> modelWorld;
  PerceptionObjects2Ors()
    : Thread("PerceptionObjects2Ors"),
    perceptionObjects(this, "perceptionObjects", true),
    modelWorld(this, "modelWorld"){}
  void open(){}
  void step();
  void close(){}
};
