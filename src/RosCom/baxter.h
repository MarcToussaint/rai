#include "roscom.h"
//#include <baxter_core_msgs/JointCommand.h>

//baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const ors::KinematicWorld& baxterModel, const char* prefix);

bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel);


struct SendPositionCommandsToBaxter:Module{
  Access_typed<arr> q_ref;
  ros::NodeHandle *nh;
  ros::Publisher pubL, pubR, pubHead, pubGripper;

  ors::KinematicWorld baxterModel;
  SendPositionCommandsToBaxter();
  ~SendPositionCommandsToBaxter(){}

  void open();

  void step();

  void close();
};
