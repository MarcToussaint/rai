#include "taskMap.h"

struct TM_InertialMotion : TaskMap {
  int i;
  double g,c;

  TM_InertialMotion(const mlr::KinematicWorld &K, const char* i_name, double g=-9.81, double c=0.)
    : i(initIdArg(K, i_name)), g(g), c(c) {}

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const mlr::KinematicWorld& K){ HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  virtual uint dim_phi(const WorldL& Ktuple, int t);

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("InertialMotion"); }
};
