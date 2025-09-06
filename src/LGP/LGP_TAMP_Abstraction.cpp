#include "LGP_TAMP_Abstraction.h"

#include "../KOMO/manipTools.h"

namespace rai {

//===========================================================================

std::shared_ptr<KOMO> LGP_TAMP_Abstraction::get_waypointsProblem(Configuration& C, StringAA& actionSequence){
  std::shared_ptr<KOMO> ways = setup_sequence(C, actionSequence.N);

  // cout <<ways->report(true, true, false) <<endl;

  for(uint t=0;t<actionSequence.N;t++){
    add_action_constraints(ways, double(t)+1., actionSequence(t));
    // cout <<ways->report(true, true, false) <<endl;
  }

  for(uint i=0; i<explicitCollisions.N; i+=2) {
    ways->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {1e1});
  }

  return ways;
}

std::shared_ptr<KOMO> LGP_TAMP_Abstraction::get_fullMotionProblem(rai::Configuration& C, StringAA& actionSequence, shared_ptr<KOMO> initWithWaypoints){
  std::shared_ptr<KOMO> path = setup_motion(C, actionSequence.N);

  for(uint t=0;t<actionSequence.N;t++){
    add_action_constraints(path, double(t)+1., actionSequence(t));
  }

  for(uint i=0; i<explicitCollisions.N; i+=2) {
    path->addObjective({}, FS_distance, {explicitCollisions.elem(i), explicitCollisions.elem(i+1)}, OT_ineq, {1e1});
  }

  if(initWithWaypoints){
    auto ways = initWithWaypoints;
    arr stable_q = ways->getConfiguration_qAll(-1);
    path->setConfiguration_qAll(-2, stable_q);
    arrA waypointsAll = ways->getPath_qAll();
    path->initWithWaypoints(waypointsAll, 1, true, 0.1);
  }

  for(uint t=0;t<actionSequence.N;t++){
    add_action_constraints_motion(path, double(t)+1., (t>0?actionSequence(t-1):StringA()), actionSequence(t), t);
  }

  //  for(uint k=0;k<waypointsAll.d0;k++){
  //    path->addObjective({double(k)}, FS_qItself, {}, OT_sos, {1e1}, waypointsAll(k));
  //  }

  return path;
}

}//namespace
