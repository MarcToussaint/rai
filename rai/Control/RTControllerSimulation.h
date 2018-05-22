/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/thread.h>
#include <Kin/kin.h>
#include <Control/ctrlMsg.h>

struct RTControllerSimulation : Thread {
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_obs;
  //Var<rai::KinematicWorld> modelWorld;
  
  rai::KinematicWorld* world;
  rai::Joint *j_baseTranslationRotation;
  double tau;
  bool gravity;
  
  uint stepCount;
  double systematicErrorSdv;
  arr systematicError;
  
  //controller internals
  arr Kp_base, Kd_base, limits;
  arr I_term;
  
  RTControllerSimulation(rai::KinematicWorld realWorld, double tau=0.01, bool gravity=false, double _systematicErrorSdv=0.);
  virtual ~RTControllerSimulation() {}
  
  void open();
  void step();
  void close() {}
};

#endif // PR2DYNAMICSIMULATION_H
