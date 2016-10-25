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

struct Optitrack : Thread{
  ACCESSname(tf::tfMessage, tf_messages)
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_markers)
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_bodies)

  SubscribeOptitrack* sub;

  Optitrack():Thread("Optitrack", -1){}
  ~Optitrack(){}

  virtual void open();
  virtual void step();
  virtual void close();
};
