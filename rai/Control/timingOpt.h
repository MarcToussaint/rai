#pragma once

#include <Optim/MathematicalProgram.h>
#include <Algo/spline.h>

//===========================================================================

struct TimingProblem : MathematicalProgram {
  //problem specs
  arr flags;    //way points
  arr tangents; //optional tangents at the way points
  arr x0, v0;   //start state
  double alpha; //time-opt weight
  bool optTau=true; //option: if false, only velocities are fitted to the given points and timing

  //decision variables (optimization output)
  arr v;   //velocities at way points
  arr tau; //timing

  TimingProblem(const arr& _flags, const arr& _tangents, const arr& _x0, const arr& _v0, double _alpha,
                const arr& v_init={}, const arr& tau_init={},
                bool _optTau=true);
  ~TimingProblem(){}

  void getVels(arr& vel){
    if(tangents.N) vel = v%tangents;
    else vel = v;
    vel.append(zeros(flags.d1));
    vel.reshape(flags.d0, flags.d1);
  }
  virtual void evaluate(arr& phi, arr& J, const arr& x);
  virtual arr  getInitializationSample(const arr& previousOptima= {});
};
