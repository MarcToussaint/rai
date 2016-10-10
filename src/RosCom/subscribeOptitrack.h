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
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_markers)
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_bodies)

  SubscribeOptitrack* sub;

  Optitrack():Module("Optitrack", -1){}
  ~Optitrack(){}

  virtual void open();
  virtual void step();
  virtual void close();
};
