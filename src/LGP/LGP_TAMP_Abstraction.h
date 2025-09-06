#pragma once

#include <KOMO/komo.h>

namespace rai {

//===========================================================================

struct LGP_TAMP_Abstraction{
  virtual ~LGP_TAMP_Abstraction() {}

  //parameters
  virtual Configuration& getConfig() = 0;
  bool useBroadCollisions=false;
  StringA explicitCollisions;

  //abstraction of the logic parts: get feasible action sequences
  virtual Array<StringA> getNewActionSequence() = 0;

  //abstraction of the motion parts: given an action sequence, what are the constraints on waypoints, and potential additional running constraints on motions between waypoints
  virtual std::shared_ptr<KOMO> setup_sequence(Configuration& C, uint K) = 0;
  virtual std::shared_ptr<KOMO> setup_motion(Configuration& C, uint K) = 0;
  virtual void add_action_constraints(std::shared_ptr<KOMO>& komo, double time, const StringA& action) = 0;
  virtual void add_action_constraints_motion(std::shared_ptr<KOMO>& komo, double time, const StringA& prev_action, const StringA& action, uint actionPhase) = 0;

  //predefined helper functions for solvers: generically using the above to setup a full waypoints and path problem
  std::shared_ptr<KOMO> get_waypointsProblem(Configuration& C, StringAA& actionSequence);
  std::shared_ptr<KOMO> get_fullMotionProblem(Configuration& C, StringAA& actionSequence, shared_ptr<KOMO> initWithWaypoints={});
};

PTR<LGP_TAMP_Abstraction> default_LGP_TAMP_Abstraction(rai::Configuration &C, const char* lgp_configfile);

}
