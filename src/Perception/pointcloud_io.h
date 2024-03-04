/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef POINTCLOUDSUBSCRIBER_H
#define POINTCLOUDSUBSCRIBER_H

#include <string>
#include "../Core/array.h"

namespace rai {
/// Typedef for depth image received event callbacks
typedef std::function<void(const uint16A&, double)> depth_cb;
/// Typedef for video image received event callbacks
typedef std::function<void(const byteA&, double)> video_cb;

class PointCloudReader {
 private:
  unique_ptr<struct sPointCloudReader> self;
 public:
  PointCloudReader(depth_cb depth_cb, video_cb video_cb, const std::string& filename);
};

class PointCloudSubscriber {
 private:
  unique_ptr<struct sPointCloudSubscriber> self;
 public:
  PointCloudSubscriber(depth_cb depth_cb, video_cb video_cb, const std::string& base_topic);
};

}

#endif // POINTCLOUDSUBSCRIBER_H
