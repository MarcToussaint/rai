#ifdef RAI_ROS

#include "subscribeOptitrack.h"
#include <tf/tfMessage.h>

void Optitrack::open()
{
  this->listenTo(*tf_messages.data);
  sub = new SubscribeOptitrack();
}
void Optitrack::close(){
  this->stopListenTo(*tf_messages.data);
  delete sub;
}

void Optitrack::step()
{
  tf::tfMessage msg = tf_messages.get();
  if (msg.transforms.size() == 0)
  {
    return;
  }

  std::vector<geometry_msgs::TransformStamped> tfs = msg.transforms;
  std::vector<geometry_msgs::TransformStamped> bodies; bodies.clear();
  std::vector<geometry_msgs::TransformStamped> markers; markers.clear();

  for (uint i = 0; i < tfs.size(); ++i)
  {
      geometry_msgs::TransformStamped tf = tfs.at(i);
      if (!strcmp(tf.header.frame_id.c_str(), "world") )
      {
        if (!strncmp(tf.child_frame_id.c_str(), "body_", 4))
        {
          bodies.push_back(tf);
          continue;
        }
        if (!strncmp(tf.child_frame_id.c_str(), "marker_", 6))
        {
          markers.push_back(tf);
          continue;
        }
      }
  }
  if (bodies.size() > 0)
  {
      opti_bodies.set() = bodies;
  }
  if (markers.size() > 0)
  {
      opti_markers.set() = markers;
  }
}
#endif
