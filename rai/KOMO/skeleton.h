#pragma once

#include "skeletonSymbol.h"

#include "../Core/util.h"
#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Optim/MathematicalProgram.h"
#include "../Optim/MP_Solver.h"

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
  shared_ptr<KOMO> komo;
  shared_ptr<MathematicalProgram> mp;
  shared_ptr<MathematicalProgram_Factored> fmp;
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

  //-- get MP transcriptions
  //keyframes
  SkeletonTranscription mp(uint stepsPerPhase=1);
  SkeletonTranscription mp_finalSlice(); //"pose bound"
  shared_ptr<MathematicalProgram> mp_timeConditional(const uintA& vars, const uintA& cond);
  shared_ptr<MathematicalProgram_Factored> mp_timeFactored();
  shared_ptr<MathematicalProgram_Factored> mp_fineFactored();
  //path
  SkeletonTranscription mp_path(const arrA& waypoints={});

  //-- to be removed (call generic NLPsolver)
  arr solve(rai::ArgWord sequenceOrPath, int verbose=2);
  shared_ptr<SolverReturn> solve2();
  shared_ptr<SolverReturn> solve3(bool useKeyframes);

  //-- drivers
  void getKeyframeConfiguration(rai::Configuration& C, int step, int verbose=0); //get the Configuration (esp. correct switches/dofs) for given step
  void solve_RRTconnectKeyframes(const arr& keyFrames_X);

  //not sure
  //void setKOMOBackground(const Animation& _A, const arr& times);
  void setKOMO(KOMO& komo) const;
  void setKOMO(KOMO& komo, ArgWord sequenceOrPath) const;

  //-- I/O
  void read(istream& is);
  void read_old(istream& is);
  void write(ostream& is, const intA& switches= {}) const;

private:
  void ensure_komo();
};
stdPipes(Skeleton)

} //namespace
