#include "komo.h"

#include "../Optim/MathematicalProgram.h"

struct OptBench_InvKin_Endeff {
  OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained);
  shared_ptr<MathematicalProgram> get(){  return nlp;  }

private:
  unique_ptr<KOMO> komo;
  shared_ptr<KOMO::Conv_KOMO_SparseNonfactored> nlp;
};

struct OptBench_Skeleton {
  void create(const char* modelFile, const Skeleton& S, rai::ArgWord sequenceOrPath);
  shared_ptr<MathematicalProgram> get(){  CHECK(nlp, "need to create first"); return nlp;  }

//private:
  unique_ptr<KOMO> komo;
  shared_ptr<KOMO::Conv_KOMO_SparseNonfactored> nlp;
};

struct OptBench_Skeleton_Pick : OptBench_Skeleton {
  Skeleton S;
  OptBench_Skeleton_Pick(rai::ArgWord sequenceOrPath);
};

struct OptBench_Skeleton_Handover : OptBench_Skeleton {
  Skeleton S;
  OptBench_Skeleton_Handover(rai::ArgWord sequenceOrPath);
};

struct OptBench_Skeleton_StackAndBalance : OptBench_Skeleton {
  Skeleton S;
  OptBench_Skeleton_StackAndBalance(rai::ArgWord sequenceOrPath);
};
