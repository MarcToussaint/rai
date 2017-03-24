#include "roscom.h"

struct SubscribeOptitrack{
  Access<tf::tfMessage> tf_messages;
  Subscriber<tf::tfMessage> sub;

  SubscribeOptitrack()
    : tf_messages(NULL, "tf_messages"),
      sub("/tf", tf_messages) {
  }
  ~SubscribeOptitrack(){
  }

};

struct Optitrack : Thread{
  ACCESS(tf::tfMessage, tf_messages)
  ACCESS(std::vector<geometry_msgs::TransformStamped>, opti_markers)
  ACCESS(std::vector<geometry_msgs::TransformStamped>, opti_bodies)

  SubscribeOptitrack* sub;

  Optitrack() : Thread("Optitrack", -1){}
  ~Optitrack(){}

  virtual void open();
  virtual void step();
  virtual void close();
};
