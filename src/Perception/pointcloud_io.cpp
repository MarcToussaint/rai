/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pointcloud_io.h"

#ifdef HAVE_ROS_IMAGE_TRANSPORT
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#endif

#include <functional>
/*
namespace rai {
    struct sPointCloudSubscriber {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
        ros::NodeHandle n;
        ros::Subscriber rgb_sub, depth_sub;
#endif
        sPointCloudSubscriber(depth_cb depth_cb, video_cb video_cb, const std::string& base_topic)
#ifdef HAVE_ROS_IMAGE_TRANSPORT
            : rgb_sub(STRING(base_topic << "/rgb/image_raw").p, 1, std::bind(&sPointCloudSubscriber::ros_video_cb, this, std::placeholders::_1)),
                      depth_sub(STRING(base_topic << "/depth_registered/image_raw"), 1, std::bind(&sPointCloudSubscriber::ros_video_cb, this, std::placeholders::_1))
#endif
        {

        }

#ifdef HAVE_ROS_IMAGE_TRANSPORT
        void ros_video_cb(const sensor_msgs::Image::ConstPtr& image) {

        }
        void ros_depth_cb(const sensor_msgs::Image::ConstPtr& image) {

        }
#endif
    };

    PointCloudSubscriber::PointCloudSubscriber(depth_cb depth_cb, video_cb video_cb, const std::string& base_topic) {

    }
}*/
