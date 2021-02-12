#include "opt-benchmarks.h"

#include "../KOMO/komo.h"

OptBench_InvKin_Endeff::OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained){
  rai::Configuration C(modelFile);
  komo = make_unique<KOMO>();
  komo->solver = rai::KS_dense;
  komo->setModel(C, false);
  komo->setTiming(1., 1, 1., 1);
  komo->add_qControlObjective({}, 1, 1.);
  //    komo->addSquaredQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  ObjectiveType ot = OT_eq;
  double prec = 1e0;
  if(unconstrained){ ot = OT_sos;  prec = 1e2;  }

  komo->addObjective({}, FS_positionDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_vectorZDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_scalarProductXX, {"gripper", "box"}, ot, {prec});

  nlp = make_shared<KOMO::Conv_KOMO_SparseNonfactored>(*komo, false);
}

void OptBench_Skeleton::create(const char* modelFile, const Skeleton& S, rai::ArgWord sequenceOrPath) {
  rai::Configuration C(modelFile);

  komo = make_unique<KOMO>();
  if(sequenceOrPath==rai::_sequence){
    komo->solver = rai::KS_dense;
  }else{
    komo->solver = rai::KS_sparse;
  }
  komo->setModel(C, false);

  double maxPhase = getMaxPhaseFromSkeleton(S);
  if(sequenceOrPath==rai::_sequence){
    komo->setTiming(maxPhase, 1, 2., 1);
    komo->add_qControlObjective({}, 1, 1e-1);
  }else{
    komo->setTiming(maxPhase, 30, 5., 2);
    komo->add_qControlObjective({}, 2, 1e0);
  }
  komo->addSquaredQuaternionNorms();

  komo->setSkeleton(S);

  nlp = make_shared<KOMO::Conv_KOMO_SparseNonfactored>(*komo, sequenceOrPath==rai::_path);

  komo->run_prepare(0.);
  cout <<"** OptBench_Skeleton: created path ";
  komo->pathConfig.report();
}

//===========================================================================

OptBench_Skeleton_Pick::OptBench_Skeleton_Pick(rai::ArgWord sequenceOrPath){
  Skeleton S = {
    //grasp
    { 1., 1., SY_touch, {"R_endeff", "box3"} },
    { 1., 1.2, SY_stable, {"R_endeff", "box3"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_Handover::OptBench_Skeleton_Handover(rai::ArgWord sequenceOrPath){
  Skeleton S = {
    //grasp
    { 1., 1., SY_touch, {"R_endeff", "stick"} },
    { 1., 2., SY_stable, {"R_endeff", "stick"} },

    //handover
    { 2., 2., SY_touch, {"L_endeff", "stick"} },
    { 2., -1., SY_stable, {"L_endeff", "stick"} },

    //touch something
    { 3., -1., SY_touch, {"stick", "ball"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_StackAndBalance::OptBench_Skeleton_StackAndBalance(rai::ArgWord sequenceOrPath){
  Skeleton S = {
    //pick
    { 1., 1., SY_touch, {"R_endeff", "box0"} },
    { 1., 2., SY_stable, {"R_endeff", "box0"} },
    { .9, 1.1, SY_downUp, {"R_endeff"} },

    //place
    { 2., 2., SY_touch, {"table", "box0"} },
    { 2., -1., SY_stable, {"table", "box0"} },
    { 1.9, 2.1, SY_downUp, {"R_endeff"} },

    //pick
    { 1.5, 1.5, SY_touch, {"L_endeff", "box1"} },
    { 1.5, 3., SY_stable, {"L_endeff", "box1"} },
    { 1.4, 1.5, SY_downUp, {"L_endeff"} },

    //place
    { 3., 3., SY_touch, {"box0", "box1"} },
    { 3., -1., SY_stable, {"box0", "box1"} },
    { 2.9, 3.1, SY_downUp, {"L_endeff"} },

    { 3., 4., SY_forceBalance, {"box1"} },
    { 3., 4., SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., SY_touch, {"R_endeff", "box2"} },
    { 4., 5., SY_stable, {"R_endeff", "box2"} },
    { 3.9, 4.5, SY_downUp, {"R_endeff"} },

    //place
    { 5., 5., SY_touch, {"box1", "box2"} },
    { 5., -1., SY_stable, {"box1", "box2"} },
    { 4.9, 5.1, SY_downUp, {"R_endeff"} },

    { 5., 5., SY_forceBalance, {"box2"} },
    { 5., 5., SY_contact, {"box1", "box2"} },

    { 5., -1., SY_forceBalance, {"box1"} },
    { 5., -1., SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., SY_touch, {"L_endeff", "box3"} },
    { 4., 5., SY_stable, {"L_endeff", "box3"} },
    { 3.9, 4.5, SY_downUp, {"L_endeff"} },

    //place
    { 5., 5., SY_touch, {"box1", "box3"} },
    { 5., 5., SY_touch, {"box2", "box3"} },
    { 5., 5.5, SY_stable, {"box1", "box3"} },
    { 4.9, 5.1, SY_downUp, {"L_endeff"} },

    { 5., 5., SY_forceBalance, {"box3"} },
    { 5., 5., SY_contact, {"box1", "box3"} },

  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}
