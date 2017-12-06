#include "taskMap.h"

struct TM_Gravity : TaskMap {

  TM_Gravity(){}

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const mlr::KinematicWorld& K){ HALT("can only be of higher order"); }

  virtual void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  virtual uint dim_phi(const WorldL& Ktuple, int t);

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("Gravity"); }
};
