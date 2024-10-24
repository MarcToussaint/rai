/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "opt-benchmarks.h"

#include "../Kin/frame.h"
#include "../KOMO/komo.h"

OptBench_InvKin_Simple::OptBench_InvKin_Simple() {
  rai::Configuration C(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  rai::Frame* f = C.addFrame("target", "table");
  f->setRelativePosition({.3, .2, .2});
  f->setShape(rai::ST_sphere, {.02}) .setColor({1., 1., 0.});

  komo = make_shared<KOMO>();
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 0);

  komo->addControlObjective({}, 0, 1e-1);
//  komo->add_jointLimits();
  komo->addObjective({}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1});

  nlp = komo->nlp();
}

OptBench_InvKin_Endeff::OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained) {
  rai::Configuration C(modelFile);
  komo = make_unique<KOMO>();
  komo->opt.sparse = false;
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 1);
  komo->addControlObjective({}, 1, 1.);
  //    komo->addQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  ObjectiveType ot = OT_eq;
  double prec = 1e0;
  if(unconstrained) { ot = OT_sos;  prec = 1e2;  }

  komo->addObjective({}, FS_positionDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_vectorZDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_scalarProductXX, {"gripper", "box"}, ot, {prec});

  nlp = komo->nlp();
}

void OptBench_Skeleton::create(const char* modelFile, const rai::Skeleton& S, rai::ArgWord sequenceOrPath) {
  rai::Configuration C(modelFile);

  komo = make_unique<KOMO>();
  komo->setConfig(C, false);

  double maxPhase = S.getMaxPhase();
  if(sequenceOrPath==rai::_sequence) {
    komo->setTiming(maxPhase, 1, 2., 1);
    komo->addControlObjective({}, 1, 1e-1);
  } else {
    komo->setTiming(maxPhase, 30, 5., 2);
    komo->addControlObjective({}, 2, 1e0);
  }
  komo->addQuaternionNorms();

  S.addObjectives(*komo);

  nlp = komo->nlp();

  komo->run_prepare(0.);
  cout <<"** OptBench_Skeleton: created path ";
  komo->pathConfig.report();
}

//===========================================================================

OptBench_Skeleton_Pick::OptBench_Skeleton_Pick(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //grasp
    { 1., 1., rai::SY_touch, {"R_endeff", "box3"} },
    { 1., 1., rai::SY_stable, {"R_endeff", "box3"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_Handover::OptBench_Skeleton_Handover(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //grasp
    { 1., 1., rai::SY_touch, {"R_endeff", "stick"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "stick"} },

    //handover
    { 2., 2., rai::SY_touch, {"L_endeff", "stick"} },
    { 2., -1., rai::SY_stable, {"L_endeff", "stick"} },

    //touch something
    { 3., -1., rai::SY_touch, {"stick", "ball"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_StackAndBalance::OptBench_Skeleton_StackAndBalance(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //pick
    { 1., 1., rai::SY_touch, {"R_endeff", "box0"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "box0"} },
    { .9, 1.1, rai::SY_downUp, {"R_endeff"} },

    //place
    { 2., 2., rai::SY_touch, {"table", "box0"} },
    { 2., -1., rai::SY_stable, {"table", "box0"} },
    { 1.9, 2.1, rai::SY_downUp, {"R_endeff"} },

    //pick
    { 1.5, 1.5, rai::SY_touch, {"L_endeff", "box1"} },
    { 1.5, 3., rai::SY_stable, {"L_endeff", "box1"} },
    { 1.4, 1.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 3., 3., rai::SY_touch, {"box0", "box1"} },
    { 3., -1., rai::SY_stable, {"box0", "box1"} },
    { 2.9, 3.1, rai::SY_downUp, {"L_endeff"} },

    { 3., 4., rai::SY_forceBalance, {"box1"} },
    { 3., 4., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"R_endeff", "box2"} },
    { 4., 5., rai::SY_stable, {"R_endeff", "box2"} },
    { 3.9, 4.5, rai::SY_downUp, {"R_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box2"} },
    { 5., -1., rai::SY_stable, {"box1", "box2"} },
    { 4.9, 5.1, rai::SY_downUp, {"R_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box2"} },
    { 5., 5., rai::SY_contact, {"box1", "box2"} },

    { 5., -1., rai::SY_forceBalance, {"box1"} },
    { 5., -1., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"L_endeff", "box3"} },
    { 4., 5., rai::SY_stable, {"L_endeff", "box3"} },
    { 3.9, 4.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box3"} },
    { 5., 5., rai::SY_touch, {"box2", "box3"} },
    { 5., 5.5, rai::SY_stable, {"box1", "box3"} },
    { 4.9, 5.1, rai::SY_downUp, {"L_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box3"} },
    { 5., 5., rai::SY_contact, {"box1", "box3"} },

  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

