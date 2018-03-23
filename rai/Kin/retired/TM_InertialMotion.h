#include "taskMap.h"

struct TM_InertialMotion : TaskMap {
  int i;
  double g,c;

  TM_InertialMotion(const rai::KinematicWorld &K, const char* i_name, double g=-9.81, double c=0.)
    : i(initIdArg(K, i_name)), g(g), c(c) {}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const rai::KinematicWorld& K){ HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  virtual uint dim_phi(const WorldL& Ktuple, int t);

  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("InertialMotion"); }
};
