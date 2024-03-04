/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"
#include "skeleton.h"

#include "../Optim/NLP.h"

struct OptBench_InvKin_Simple {
  OptBench_InvKin_Simple();
  shared_ptr<KOMO> komo;
  shared_ptr<NLP> nlp;
};

struct OptBench_InvKin_Endeff {
  OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained);
  shared_ptr<NLP> get() {  return nlp;  }

 private:
  unique_ptr<KOMO> komo;
  shared_ptr<NLP> nlp;
};

struct OptBench_Skeleton {
  void create(const char* modelFile, const rai::Skeleton& S, rai::ArgWord sequenceOrPath);
  shared_ptr<NLP> get() {  CHECK(nlp, "need to create first"); return nlp;  }

//private:
  unique_ptr<KOMO> komo;
  shared_ptr<NLP> nlp;
};

struct OptBench_Skeleton_Pick : OptBench_Skeleton {
  rai::Skeleton S;
  OptBench_Skeleton_Pick(rai::ArgWord sequenceOrPath);
};

struct OptBench_Skeleton_Handover : OptBench_Skeleton {
  rai::Skeleton S;
  OptBench_Skeleton_Handover(rai::ArgWord sequenceOrPath);
};

struct OptBench_Skeleton_StackAndBalance : OptBench_Skeleton {
  rai::Skeleton S;
  OptBench_Skeleton_StackAndBalance(rai::ArgWord sequenceOrPath);
};

