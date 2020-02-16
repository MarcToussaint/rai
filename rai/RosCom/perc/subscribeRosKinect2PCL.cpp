#ifdef RAI_ROS

#ifdef RAI_PCL

#include "subscribeRosKinect2PCL.h"
#include "roscom.h"
#include <tf/tf.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

struct sSubscribeRosKinect2PCL{
  ros::NodeHandle *nh = nullptr;
  ros::Subscriber sub;
  tf::TransformListener *listener = nullptr;

  pcl::PCLPointCloud2 pcl_pc2;
  Pcl pcl;
  Pcl pcl_trans;
};

SubscribeRosKinect2PCL::SubscribeRosKinect2PCL(const char* cloud_name, const char* topic_name)
  : cloud(nullptr, cloud_name),
    kinect_frame(nullptr, "kinect_frame"){
  s = new sSubscribeRosKinect2PCL;
  if(rai::getParameter<bool>("useRos")){
    self->nh = new ros::NodeHandle;
    self->listener = new tf::TransformListener;
    //      registry()->newNode<SubscriberType*>({"Subscriber", topic_name}, {access.registryNode}, this);
    LOG(0) <<"subscribing to topic '" <<topic_name <<"' <" <<typeid(Pcl).name() <<"> into access '" <<cloud.name <<'\'';
    self->sub = self->nh->subscribe(topic_name, 1, &SubscribeRosKinect2PCL::callback, this);
  }
}

SubscribeRosKinect2PCL::~SubscribeRosKinect2PCL(){
  delete self->listener;
  delete self->nh;
}

void SubscribeRosKinect2PCL::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  double time=conv_time2double(msg->header.stamp);
  tf::Transform trans;
  rai::Transformation t = ros_getTransform("/base_link", msg->header, *self->listener, &trans);

  pcl_conversions::toPCL(*msg, self->pcl_pc2);
  pcl::fromPCLPointCloud2(self->pcl_pc2, self->pcl);
  pcl_ros::transformPointCloud(self->pcl, self->pcl_trans, trans);

  cloud.set( time ) = self->pcl_trans;
  kinect_frame.set( time ) = t;
}
#endif

#endif
