#include "taskMap.h"

struct TM_ImpulsExchange : TaskMapInit2Frames {

  TM_ImpulsExchange(const mlr::KinematicWorld &K, const char* i_name, const char* j_name)
    : TaskMapInit2Frames(K, i_name, j_name){}

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1){ HALT(""); }
  uint dim_phi(const mlr::KinematicWorld& K){ HALT(""); }

  void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);

  uint dim_phi(const WorldL& Ktuple, int t){ return 6; }

  mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ImpulseExchange"); }
};
