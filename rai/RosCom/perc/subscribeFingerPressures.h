#pragma once

#include "roscom.h"
#include <pr2_msgs/PressureState.h>

arr conv_pressureState2arr(const pr2_msgs::PressureState& msg);

struct SubscribeLaserScan{
  Var<arr> fingerPressures_left;
  Var<arr> fingerPressures_right;
  SubscriberConv<pr2_msgs::PressureState, arr, &conv_pressureState2arr> subLeft;
  SubscriberConv<pr2_msgs::PressureState, arr, &conv_pressureState2arr> subRight;

  SubscribeLaserScan()
    : subRight("/pressure/l_gripper_motor", fingerPressures_left),
      subRight("/pressure/r_gripper_motor", fingerPressures_right){}
  ~SubscribeLaserScan(){}
};

