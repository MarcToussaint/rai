#pragma once

#include <Optim/NLP.h>
#include <Algo/spline.h>

//===========================================================================

struct TimingProblem : NLP {
  //problem specs
  arr waypoints; //way points
  arr x0, v0;    //start state
  double timeCost;  //time-opt weight
  double timeCost2;  //time-opt weight
  double ctrlCost;
  bool optTau=true; //option: if false, only velocities are fitted to the given points and timing
  bool optLastVel=false;
  bool tauBarrier=false;
  bool accCont=false;
  uintA wayFree;

  arr maxVel;
  arr maxAcc;
  arr maxJer;

  //decision variables (optimization output)
  arr v;   //velocities at way points
  arr tau; //timing

  TimingProblem(const arr& _waypoints, const arr& _tangents, const arr& _x0, const arr& _v0,
                double _timeCost, double _ctrlCost,
                bool _optTau=true,  bool _optLastVel=false,
                const arr& v_init={}, const arr& tau_init={},
                double _maxVel=-1., double _maxAcc=-1., double _maxJer=-1.,
                const uintA& _wayFree={}, bool _accCont=false, double _timeCost2=-1.);
  ~TimingProblem(){}

  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual arr  getInitializationSample(const arr& previousOptima= {});
  virtual void report(ostream& os, int verbose, const char* msg=0);

  void smartInitVels();
  void getVels(arr& vel);

private:
  arr xJ(int k);
  arr vJ(int k);
  arr Jtau(int k);
};
