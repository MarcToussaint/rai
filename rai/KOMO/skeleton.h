#pragma once

#include "skeletonSymbol.h"

#include "../Core/defines.h"
#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Optim/NLP.h"
#include "../Optim/NLP_Solver.h"

struct KOMO;

namespace rai {

//===========================================================================

struct SkeletonEntry { //-> rename to Mode or something
  double phase0=-1.;
  double phase1=-1.;
  Enum<SkeletonSymbol> symbol;
  StringA frames; //strings referring to things
  SkeletonEntry() {}
  SkeletonEntry(double phase0, double phase1, SkeletonSymbol symbol, StringA frames) : phase0(phase0), phase1(phase1), symbol(symbol), frames(frames) {}
  void write(ostream& os) const;
};
stdOutPipe(SkeletonEntry)

//===========================================================================

struct SkeletonTranscription {
  std::shared_ptr<KOMO> komo;
  std::shared_ptr<NLP> nlp;
  std::shared_ptr<NLP_Factored> fmp;
  std::shared_ptr<SolverReturn> ret;
};

//===========================================================================

struct Skeleton {
  Array<SkeletonEntry> S;
  const Configuration *C=0;
  std::shared_ptr<KOMO> komo;
  bool collisions=false;
  int verbose=1;

  Skeleton() {}
  Skeleton(std::initializer_list<SkeletonEntry> entries) : S(entries) {}

  //-- set the skeleton
  void setConfiguration(const Configuration& _C){ C=&_C; } //brittle..
  void setFromStateSequence(Array<Graph*>& states, const arr& times);
  void fillInEndPhaseOfModes();

  //-- skeleton info
  double getMaxPhase() const;
  intA getSwitches(const Configuration& C) const;

  //-- get NLP transcriptions
  //keyframes
  SkeletonTranscription nlp(uint stepsPerPhase=1);
  SkeletonTranscription nlp_finalSlice(); //"pose bound"
  shared_ptr<NLP> nlp_timeConditional(const uintA& vars, const uintA& cond);
  shared_ptr<NLP_Factored> nlp_timeFactored();
  shared_ptr<NLP_Factored> nlp_fineFactored();
  //path
  SkeletonTranscription nlp_path(const arrA& waypoints={});

  //-- to be removed (call generic NLPsolver)
  arr solve(rai::ArgWord sequenceOrPath, int verbose=2);
  SkeletonTranscription solve2(int verbose=4);
  shared_ptr<SolverReturn> solve3(bool useKeyframes, int verbose=4);

  //-- drivers
  void getKeyframeConfiguration(rai::Configuration& C, int step, int verbose=0); //get the Configuration (esp. correct switches/dofs) for given step
  void solve_RRTconnectKeyframes(const arr& keyFrames_X);

  //not sure
  //void setKOMOBackground(const Animation& _A, const arr& times);
  void setKOMO(KOMO& komo) const;
  void setKOMO(KOMO& komo, ArgWord sequenceOrPath, uint stepsPerPhase=30, double accScale=1e0, double lenScale=1e-2, double homingScale=1e-2) const;

  //-- I/O
  void read(istream& is);
  void read_old(istream& is);
  void write(ostream& is, const intA& switches= {}) const;

private:
  void ensure_komo();
};
stdPipes(Skeleton)

} //namespace
