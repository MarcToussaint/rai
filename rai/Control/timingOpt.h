#pragma once

#include <Optim/MathematicalProgram.h>
#include <Algo/spline.h>

//===========================================================================

struct TimingProblem : MathematicalProgram {
  //problem specs
  arr waypoints; //way points
  arr tangents;  //optional tangents at the way points
  arr x0, v0;    //start state
  double timeCost;  //time-opt weight
  double ctrlCost;
  bool optTau=true; //option: if false, only velocities are fitted to the given points and timing
  bool optLastVel=false;

  const double maxVel;
  const double maxAcc;
  const double maxJer;

  //decision variables (optimization output)
  arr v;   //velocities at way points
  arr tau; //timing

  TimingProblem(const arr& _waypoints, const arr& _tangents, const arr& _x0, const arr& _v0, double _timeCost,
                const arr& v_init={}, const arr& tau_init={},
                bool _optTau=true,
                double _maxVel=-1., double _maxAcc=-1., double _maxJer=-1., double _ctrlCost=1e0, bool _optLastVel=false);
  ~TimingProblem(){}

  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual arr  getInitializationSample(const arr& previousOptima= {});

  void getVels(arr& vel);
};
