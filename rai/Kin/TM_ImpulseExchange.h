#include "taskMap.h"

struct TM_ImpulsExchange : TaskMap {
  int i,j;

  TM_ImpulsExchange(const mlr::KinematicWorld &K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)){}

  void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  uint dim_phi(const mlr::KinematicWorld& K){ return 6; }

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1){ HALT(""); }

  mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ImpulseExchange"); }
};


struct TM_ImpulsExchange_weak : TaskMap {
  int i,j;

  TM_ImpulsExchange_weak(const mlr::KinematicWorld &K, const char* i_name, const char* j_name)
    : i(initIdArg(K, i_name)), j(initIdArg(K, j_name)){}

  void phi(arr& y, arr& J, const WorldL& Ktuple, double tau, int t=-1);
  uint dim_phi(const mlr::KinematicWorld& K){ return 3; }

  void phi(arr& y, arr& J, const mlr::KinematicWorld& K, int t=-1){ HALT(""); }

  mlr::String shortTag(const mlr::KinematicWorld& K){ return STRING("ImpulseExchange"); }
};
