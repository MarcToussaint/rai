#include "roscom.h"

struct SubscribeOptitrack{
  Var<tf::tfMessage> tf_messages;
  Subscriber<tf::tfMessage> sub;

  SubscribeOptitrack()
    : tf_messages(nullptr, "tf_messages"),
      sub("/tf", tf_messages) {
  }
  ~SubscribeOptitrack(){
  }

};

struct Optitrack : Thread{
  VAR(tf::tfMessage, tf_messages)
  VAR(std::vector<geometry_msgs::TransformStamped>, opti_markers)
  VAR(std::vector<geometry_msgs::TransformStamped>, opti_bodies)

  SubscribeOptitrack* sub;

  Optitrack() : Thread("Optitrack", -1){}
  ~Optitrack(){}

  virtual void open();
  virtual void step();
  virtual void close();
};
