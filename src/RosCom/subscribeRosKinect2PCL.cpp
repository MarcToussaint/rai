#ifdef MLR_ROS

#include "subscribeRosKinect2PCL.h"
#include "roscom.h"
#include <tf/tf.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

struct sSubscribeRosKinect2PCL{
  ros::NodeHandle *nh = NULL;
  ros::Subscriber sub;
  tf::TransformListener *listener = NULL;

  pcl::PCLPointCloud2 pcl_pc2;
  Pcl pcl;
  Pcl pcl_trans;
};

SubscribeRosKinect2PCL::SubscribeRosKinect2PCL(const char* cloud_name, const char* topic_name)
  : cloud(NULL, cloud_name),
    kinect_frame(NULL, "kinect_frame"){
  s = new sSubscribeRosKinect2PCL;
  if(mlr::getParameter<bool>("useRos")){
    s->nh = new ros::NodeHandle;
    s->listener = new tf::TransformListener;
    //      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
    LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(Pcl).name() <<"> into access '" <<cloud.name <<'\'';
    s->sub = s->nh->subscribe(topic_name, 1, &SubscribeRosKinect2PCL::callback, this);
  }
}

SubscribeRosKinect2PCL::~SubscribeRosKinect2PCL(){
  delete s->listener;
  delete s->nh;
  delete s;
}

void SubscribeRosKinect2PCL::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  double time=conv_time2double(msg->header.stamp);
  tf::Transform trans;
  mlr::Transformation t = ros_getTransform("/base_link", msg->header, *s->listener, &trans);

  pcl_conversions::toPCL(*msg, s->pcl_pc2);
  pcl::fromPCLPointCloud2(s->pcl_pc2, s->pcl);
  pcl_ros::transformPointCloud(s->pcl, s->pcl_trans, trans);

  cloud.set( time ) = s->pcl_trans;
  kinect_frame.set( time ) = t;
}
#endif
