#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/thread.h>
#include <Kin/kin.h>
#include <Control/ctrlMsg.h>

struct RTControllerSimulation : Thread {
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_obs;
  //Var<mlr::KinematicWorld> modelWorld;

  mlr::KinematicWorld* world;
  mlr::Joint *j_baseTranslationRotation;
  double tau;
  bool gravity;

  uint stepCount;
  double systematicErrorSdv;
  arr systematicError;

  //controller internals
  arr Kp_base, Kd_base, limits;
  arr I_term;

  RTControllerSimulation(mlr::KinematicWorld realWorld, double tau=0.01, bool gravity=false, double _systematicErrorSdv=0.);
  virtual ~RTControllerSimulation() {}

  void open();
  void step();
  void close(){}
};

#endif // PR2DYNAMICSIMULATION_H
