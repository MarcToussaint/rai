/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "komo.h"
#include "skeleton.h"
#include "manipTools.h"

#include "../Optim/NLP.h"

namespace rai{
  std::shared_ptr<NLP> make_NLP_Problem(str problem={});
  StringA get_NLP_Problem_names();
}

struct KOMO_wrap : std::shared_ptr<NLP> {
  std::shared_ptr<KOMO> komo;
  KOMO_wrap(const std::shared_ptr<KOMO>& _komo) : komo(_komo){
    std::shared_ptr<NLP>::operator=( komo->nlp() );
  }
};

shared_ptr<KOMO> problem_IK();
shared_ptr<KOMO> problem_IKobstacle();
shared_ptr<KOMO> problem_IKtorus();
shared_ptr<KOMO> problem_PushToReach();
shared_ptr<KOMO> problem_StableSphere();

//===========================================================================

//a set of spheres, confined in a box, and no collision, minimizing their height..
struct SpherePacking : NLP{
  arr x; //position of spheres
  uint n;
  double rad;
  bool ineqAccum;

  rai::Configuration disp; //for reporting/display only

  SpherePacking(uint _n=50, double _rad=.21, bool _ineqAccum=false);

  void ineqAccumulation(uint phiIdx, arr& phi, arr& J, arr& g, const arr& Jg);
  void evaluate(arr& phi, arr& J, const arr &_x);
  void report(ostream &os, int verbose, const char *msg=0);
};

//===========================================================================

struct MinimalConvexCore : NLP {
  double radius;
  arr x, cen;

  rai::Mesh M;
  rai::Configuration disp; //for reporting/display only

  MinimalConvexCore(const arr& X={}, double radius=.1);
  arr getInitializationSample();
  void evaluate(arr& phi, arr& J, const arr& _x);
  void report(ostream &os, int verbose, const char *msg=0);
};

//===========================================================================

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

