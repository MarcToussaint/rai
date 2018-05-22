/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef POINTCLOUDSUBSCRIBER_H
#define POINTCLOUDSUBSCRIBER_H

#include <string>
#include <Core/array.h>

namespace rai {
/// Typedef for depth image received event callbacks
typedef std::function<void(const uint16A&, double)> depth_cb;
/// Typedef for video image received event callbacks
typedef std::function<void(const byteA&, double)> video_cb;

class PointCloudReader {
private:
  struct sPointCloudReader *s;
public:
  PointCloudReader(depth_cb depth_cb, video_cb video_cb, const std::string& filename);
};

class PointCloudSubscriber {
private:
  struct sPointCloudSubscriber *s;
public:
  PointCloudSubscriber(depth_cb depth_cb, video_cb video_cb, const std::string& base_topic);
};

}

#endif // POINTCLOUDSUBSCRIBER_H
