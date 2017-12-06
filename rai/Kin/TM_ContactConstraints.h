#include "taskMap.h"


/// this stacks all constacts to one feature vector OF VARIABLE SIZE! (the optimizer has to deal with variable size features; transfering lambdas across steps)
struct TM_ContactConstraints : TaskMap {
  TM_ContactConstraints() {}

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& K);
  virtual mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ContactConstraints"); }
};
