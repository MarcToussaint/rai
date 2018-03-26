#include "taskMap.h"

struct TM_Time : TaskMap {
  TM_Time(){}

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& K);
  virtual uint dim_phi(const rai::KinematicWorld& K){ return 1; }

  virtual rai::String shortTag(const rai::KinematicWorld& G){ return STRING("Time"); }
};
