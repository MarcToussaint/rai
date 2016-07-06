#include <Core/array.h>
#include <Core/module.h>
#include <Control/ctrlMsg.h>

#ifdef MLR_ROS
#include <sensor_msgs/JointState.h>
#include <Ors/ors.h>
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel);
arr baxter_getEfforts(const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel);
#endif

struct SendPositionCommandsToBaxter:Module{
  Access_typed<CtrlMsg> ctrl_ref;
  struct sSendPositionCommandsToBaxter *s;
  ors::KinematicWorld baxterModel;

  SendPositionCommandsToBaxter(const ors::KinematicWorld& baxterWorld);
  ~SendPositionCommandsToBaxter(){}

  void open();
  void step();
  void close();

  bool enablePositionControlL = true;
  bool enablePositionControlR = true;
  bool totalTorqueModeL = false;
  bool totalTorqueModeR = false;
};

