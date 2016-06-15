#include "roscom.h"

struct SubscribeOptitrack{
  ACCESSname(tf::tfMessage, tf_messages)
  Subscriber<tf::tfMessage> sub;

  SubscribeOptitrack()
    : sub("/tf", tf_messages) {
  }
  ~SubscribeOptitrack(){
  }

};

struct Optitrack : Module{
  ACCESSname(tf::tfMessage, tf_messages)
  ACCESSname(geometry_msgs::TransformStamped, opti_drone)
  ACCESSname(geometry_msgs::TransformStamped, opti_body)

  Optitrack():Module("Optitrack", -1){}
  ~Optitrack(){}

  virtual void open();
  virtual void step();
  virtual void close();
};
