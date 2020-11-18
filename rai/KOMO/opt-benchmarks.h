#include "komo.h"

#include "../Optim/MathematicalProgram.h"

struct OptBench_InvKin_Endeff {
  OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained);
  shared_ptr<MathematicalProgram> get(){  return nlp;  }

private:
  unique_ptr<KOMO> komo;
  shared_ptr<KOMO::Conv_KOMO_SparseNonfactored> nlp;
};
