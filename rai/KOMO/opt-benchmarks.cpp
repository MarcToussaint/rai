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

  nlp = make_shared<KOMO::Conv_KOMO_SparseNonfactored>(*komo);
}
