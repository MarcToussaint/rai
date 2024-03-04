/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include "../Core/array.h"
#include "../Core/thread.h"
#include "../Kin/kin.h"
#include "ctrlMsg.h"

struct RTControllerSimulation : Thread {
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_obs;
  //Var<rai::Configuration> modelWorld;

  rai::Configuration* world;
  rai::Joint* j_baseTranslationRotation;
  double tau;
  bool gravity;

  uint stepCount;
  double systematicErrorSdv;
  arr systematicError;

  //controller internals
  arr Kp_base, Kd_base, limits;
  arr I_term;

  RTControllerSimulation(const rai::Configuration& realWorld,
                         const Var<CtrlMsg>& _ctrl_ref,
                         const Var<CtrlMsg>& _ctrl_obs,
                         double tau=0.01, bool gravity=false, double _systematicErrorSdv=0.);
  virtual ~RTControllerSimulation() {}

  void open();
  void step();
  void close() {}
};

#endif // PR2DYNAMICSIMULATION_H
