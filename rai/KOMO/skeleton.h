#pragma once

#include "skeletonSymbol.h"

#include "../Core/util.h"
#include "../Core/array.h"
#include "../Kin/kin.h"
#include "../Optim/MathematicalProgram.h"
#include "../Optim/solver.h"

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
  shared_ptr<KOMO> komo;
  bool collisions=false;
  int verbose=1;

  Skeleton() {}
  Skeleton(std::initializer_list<SkeletonEntry> entries) : S(entries) {}

  //-- set the skeleton
  void setConfiguration(const Configuration& _C){ C=&_C; } //brittle..
  void setFromStateSequence(Array<Graph*>& states, const arr& times);

  //-- skeleton info
  double getMaxPhase() const;
  intA getSwitches(const Configuration& C) const;

  //-- get MP transcriptions
  //keyframes
  SkeletonTranscription mp();
  SkeletonTranscription mp_finalSlice(); //"pose bound"
  shared_ptr<MathematicalProgram> mp_timeConditional(const uintA& vars, const uintA& cond);
  shared_ptr<MathematicalProgram_Factored> mp_timeFactored();
  shared_ptr<MathematicalProgram_Factored> mp_fineFactored();
  //path
  SkeletonTranscription mp_path(const arrA& waypoints={});

  //-- to be removed (call generic NLPsolver)
  void solve();
  shared_ptr<SolverReturn> solve2();

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
