#pragma once

#include "roscom.h"

struct SubscribeLaserScan{
  Var<arr> laserScan;
  Var<rai::Transformation> laserScan_frame;
  SubscriberConv<sensor_msgs::LaserScan, byteA, &conv_laserScan2arr> subLaser;

  SubscribeLaserScan()
    : laserScan(nullptr, "laserScan"),
      laserScan_frame(nullptr, "laserScan_frame"),
      subLaser("/tilt_scan", laserScan, &laserScan_frame){}
  ~SubscribeLaserScan(){}

};
