/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/*
 * image_pub.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: ingo
 */

#include "image_pub.h"

#ifdef RAI_ROS

#include <ros/ros.h>

#ifdef HAVE_ROS_IMAGE_TRANSPORT
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
using namespace image_transport;
using namespace camera_info_manager;
#endif

#include "../Core/array.h"
#include "../Core/util.h"
#include <sstream>
#include <time.h>

namespace rai {

struct sImagePublisher {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
  ros::NodeHandle n;
  ImageTransport t;
  Publisher p_img;
  ros::Publisher p_info;
  CameraInfoManager cim;
  sensor_msgs::Image msg;
  sensor_msgs::CameraInfo cinfo;
#endif
  uint32_t seq, bypp;
  std::string link_name, encoding;
  double time_offset;
  PixelFormat pix_fmt;

  sImagePublisher(const std::string& base_topic, const std::string& camera_name, PixelFormat pix_fmt) :
#ifdef HAVE_ROS_IMAGE_TRANSPORT
    n(base_topic), t(n), p_img(t.advertise("image_raw", 1)),
    p_info(n.advertise<sensor_msgs::CameraInfo>("camera_info", 1)),
    cim(n, camera_name),
#endif
    seq(0), pix_fmt(pix_fmt) {
    std::ostringstream str;
    str << camera_name << "_link";
    link_name = ros::names::resolve(str.str());

    // get seconds since epoch for 00:00 today (to offsets marc's %86400 stuff)
    time_offset = time(nullptr);
    time_offset -= (((time_t)time_offset)%86400);

#ifdef HAVE_ROS_IMAGE_TRANSPORT
    switch(pix_fmt) {
      case PIXEL_FORMAT_RAW8:
        // TODO need to define more informative RAW format, to also give bayer info, if available
        encoding = sensor_msgs::image_encodings::MONO8;
        bypp = 1;
        break;
      case PIXEL_FORMAT_RGB8:
        encoding = sensor_msgs::image_encodings::RGB8;
        bypp = 3;
        break;
      case PIXEL_FORMAT_BGR8:
        encoding = sensor_msgs::image_encodings::BGR8;
        bypp = 3;
        break;
      case PIXEL_FORMAT_UYV422:
        encoding = sensor_msgs::image_encodings::YUV422;
        bypp = 2;
        break;
      default:
        throw "Unsupported pixel format";
    }
#endif
  }

  void publish(const byteA& image, double timestamp) {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
    sensor_msgs::fillImage(msg, encoding, image.d0, image.d1, bypp * image.d1, image.p);
    msg.header.seq    = seq++;
    msg.header.stamp  = ros::Time(timestamp + time_offset);
    msg.header.frame_id = link_name;

    cinfo = cim.getCameraInfo();
    cinfo.header = msg.header;

    p_img.publish(msg);
    p_info.publish(cinfo);
#endif
  }
};

ImagePublisher::ImagePublisher(const std::string& topic, const std::string& camera_name, const PixelFormat pix_fmt) :
  s(new sImagePublisher(topic, camera_name, pix_fmt)) {

}

ImagePublisher::~ImagePublisher() {
}

void ImagePublisher::publish(const rai::Array<unsigned char>& image, double timestamp) {
  self->publish(image, timestamp);
}

void init_image_publishers(int argc, char* argv[], const char* name, bool install_sigint_handler) {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
  ros::init(argc, argv, name, install_sigint_handler ?  0 : ros::init_options::NoSigintHandler);
#endif
}
void init_image_publishers(int argc, char* argv[], const char* name) {
  init_image_publishers(argc, argv, name, true);
}

bool process_image_callbacks() {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
  ros::spinOnce();
  return ros::ok();
#endif
  return true;
}
void ros_shutdown() {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
  ros::shutdown();
#endif
}
}

#else //RAI_ROS

#endif
