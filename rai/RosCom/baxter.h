/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Kin/kin.h>

#ifdef RAI_ROS
#include <sensor_msgs/JointState.h>
bool baxter_get_q_qdot_u(arr& q, arr& q_dot, arr& u, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel);
//TODO: redundant -> remove
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel);
arr baxter_getEfforts(const sensor_msgs::JointState& msg, const rai::KinematicWorld& baxterModel);
#endif

struct SendPositionCommandsToBaxter : Thread {
  Var<CtrlMsg> ctrl_ref;
  struct sSendPositionCommandsToBaxter *s;
  rai::KinematicWorld baxterModel;
  
  SendPositionCommandsToBaxter(const rai::KinematicWorld& baxterWorld);
  ~SendPositionCommandsToBaxter() {}
  
  void open();
  void step();
  void close();
  
  bool enablePositionControlL = true;
  bool enablePositionControlR = true;
  bool totalTorqueModeL = false;
  bool totalTorqueModeR = false;
};

