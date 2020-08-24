/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include <ros/ros.h>

/** MACRO to create a RAI module that integrates data from ROS into the RAI
 *  module system.
 *
 *  This macro creates a class "ROS_#var_name" that subscribes the given
 *  topic_name with the given msg_type.  Each received ros msg is copied into
 *  the RAI module system.
 *
 *  To use it you have to
 *  - add the created module "ROS_#var_name"  as module to your system
 *  - add the ACCESS variable with the ros msg type to your system
 *
 *  See ../../examples/pr2/generic_ros_sync/main.cpp for more.
 */
#ifdef RAI_ROS

#define ROSSUB(topic_name, msg_type, var_name) \
  struct ROSSUB_##var_name : Thread { \
    VAR(msg_type, var_name) \
    ros::NodeHandle* _nh; \
    ros::Subscriber _sub; \
    ROSSUB_##var_name() : Thread(#var_name) {} \
    void open() { \
      this->_nh = new ros::NodeHandle; \
      this->_sub  = this->_nh->subscribe( \
                                          topic_name, 1, &ROSSUB_##var_name::callback, this); \
    } \
    void step() {} \
    void close() { \
      this->_nh->shutdown(); \
      delete _nh; \
    } \
    void callback(const msg_type::ConstPtr& msg) { \
      this->var_name.set() = *msg; \
    } \
  };

#else

struct ROSSUB_##var_name : Thread {
  \
  VAR(msg_type, var_name) \
  ROSSUB_##var_name() : Thread(#var_name) {} \
  void open() {
    \
    LOG(-1) <<"fake subscriber: " <<#var_name <<" -- compiled without RAI_ROS flag"; \
  } \
  void step() {} \
  void close() {} \
  void callback(const msg_type::ConstPtr& msg) {} \
};

#endif
