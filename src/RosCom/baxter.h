#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>

#ifdef MLR_ROS
#include <sensor_msgs/JointState.h>
#include <Ors/ors.h>
bool baxter_get_q_qdot_u(arr& q, arr& q_dot, arr& u, const sensor_msgs::JointState& msg, const mlr::KinematicWorld& baxterModel);
//TODO: redundant -> remove
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const mlr::KinematicWorld& baxterModel);
arr baxter_getEfforts(const sensor_msgs::JointState& msg, const mlr::KinematicWorld& baxterModel);
#endif

struct SendPositionCommandsToBaxter : Thread {
  Access_typed<CtrlMsg> ctrl_ref;
  struct sSendPositionCommandsToBaxter *s;
  mlr::KinematicWorld baxterModel;

  SendPositionCommandsToBaxter(const mlr::KinematicWorld& baxterWorld);
  ~SendPositionCommandsToBaxter(){}

  void open();
  void step();
  void close();

  bool enablePositionControlL = true;
  bool enablePositionControlR = true;
  bool totalTorqueModeL = false;
  bool totalTorqueModeR = false;
};

