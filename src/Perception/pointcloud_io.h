#ifndef POINTCLOUDSUBSCRIBER_H
#define POINTCLOUDSUBSCRIBER_H

#include <string>
#include <Core/array.h>

namespace MLR { namespace ROS {

/// Typedef for depth image received event callbacks
typedef std::function<void(const uint16A&, double)> kinect_depth_cb;
/// Typedef for video image received event callbacks
typedef std::function<void(const byteA&, double)> kinect_video_cb;

class PointCloudSubscriber
{
private:
    struct sPointCloudSubscriber *s;
public:
    PointCloudSubscriber(kinect_depth_cb depth_cb, kinect_video_cb video_cb, const std::string& base_topic);
};

}}

#endif // POINTCLOUDSUBSCRIBER_H
