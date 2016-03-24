#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/module.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>

struct RTControllerSimulation : Module {
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<ors::KinematicWorld> modelWorld;

  ors::KinematicWorld* world;
  double tau;
  bool gravity;

  uint stepCount;
  double systematicErrorSdv;
  arr systematicError;

  //controller internals
  arr Kp_base, Kd_base, limits;
  arr I_term;

  RTControllerSimulation(double tau=0.01, bool gravity=false, double _systematicErrorSdv=0.);
  virtual ~RTControllerSimulation() {}

  void open();
  void step();
  void close(){}
};

#endif // PR2DYNAMICSIMULATION_H
