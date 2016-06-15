#include <tf/tfMessage.h>
#include "subscribeOptitrack.h"

void Optitrack::open()
{
  this->listenTo(*tf_messages.var);
}
void Optitrack::close(){
  this->stopListenTo(*tf_messages.var);
}

void Optitrack::step()
{
  tf::tfMessage msg = tf_messages.get();
  if (msg.transforms.size() == 0)
  {
    return;
  }
  geometry_msgs::TransformStamped tf = msg.transforms[0];
//  cout << tf << endl;
  if (!strcmp(tf.header.frame_id.c_str(), "optitrack_frame") )
  {
    if (!strcmp(tf.child_frame_id.c_str(), "drone") )
    {
      opti_drone.set() = tf;
      return;
    }
    if (!strcmp(tf.child_frame_id.c_str(), "body"))
    {
      opti_body.set() = tf;
      return;
    }
  }

}

//tf::TransformListener listener;
//          tf::StampedTransform baseTransform;
//          try{
//            listener.waitForTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), ros::Duration(1.0));
//            listener.lookupTransform("/base", msg.markers[0].header.frame_id, ros::Time(0), baseTransform);
//            tf = conv_transform2transformation(baseTransform);
//            ors::Transformation inv;
//            inv.setInverse(tf);
//            inv.addRelativeTranslation(0,0,-1);
//            inv.setInverse(inv);
//            tf = inv;
//            has_transform = true;
