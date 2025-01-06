/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

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
  std::shared_ptr<SolverReturn> ret;
};

//===========================================================================

struct Skeleton {
  Array<SkeletonEntry> S;
  std::shared_ptr<KOMO> komoWaypoints;
  std::shared_ptr<KOMO> komoPath;
  std::shared_ptr<KOMO> komoFinal;
  StringA explicitCollisions;
  StringA explicitLiftPriors;
  bool useBroadCollisions=false;
  int verbose=1;

  Skeleton() {}
  Skeleton(std::initializer_list<SkeletonEntry> entries) : S(entries) {}

  //-- set the skeleton
  void setFromStateSequence(const Array<Graph*>& states, const arr& times);
  void fillInEndPhaseOfModes();

  //-- addEntry
  void addEntry(const arr& timeInterval, SkeletonSymbol symbol, StringA frames) {
    S.append(SkeletonEntry(timeInterval(0), timeInterval(-1), symbol, frames));
  }
  void appendSkeleton(const Skeleton& other) {
    double maxPhase = getMaxPhase();
    for(SkeletonEntry s:other.S) { //copy!
      if(s.phase0>=0) s.phase0+=maxPhase;
      if(s.phase1>=0) s.phase1+=maxPhase;
      S.append(s);
    }
  }

  //-- add objectives
  void addExplicitCollisions(const StringA& collisions);
  void addLiftPriors(const StringA& lift);

  //-- skeleton info
  double getMaxPhase() const;
  intA getSwitches(const Configuration& C) const;

  //-- get KOMO transcriptions (grab the NLP directly from KOMO)
  shared_ptr<KOMO> getKomo_path(const rai::Configuration& C, uint stepsPerPhase=30, double accScale=1e0, double lenScale=1e-2, double homingScale=1e-2, double collScale=1e1);
  shared_ptr<KOMO> getKomo_waypoints(const rai::Configuration& C, double lenScale=1e-2, double homingScale=1e-2, double collScale=1e1);
  shared_ptr<KOMO> getKomo_finalSlice(const rai::Configuration& C, double lenScale=1e-2, double homingScale=1e-2, double collScale=1e1);

  //-- get same as above, with "Transcription"
  //keyframes
  SkeletonTranscription nlp_waypoints(const rai::Configuration& C);
  SkeletonTranscription nlp_path(const Configuration& C, const arrA& initWaypoints= {});
  SkeletonTranscription nlp_finalSlice(const rai::Configuration& C); //"pose bound"

  //-- get path finding problem between 2 waypoints
  static void getTwoWaypointProblem(int t2, Configuration& C, arr& q1, arr& q2, KOMO& komoWays);

  //-- I/O
  void read(istream& is);
  void read_old(istream& is);
  void write(ostream& is, const intA& switches= {}) const;

  //lower level (used within getKomo_*): add skeleton's objective to KOMO
  void addObjectives(KOMO& komoPath) const;

  //-------- deprecated
  void getKeyframeConfiguration(rai::Configuration& C, int step, int verbose=0); //get the Configuration (esp. correct switches/dofs) for given step
  shared_ptr<NLP> nlp_timeConditional(const rai::Configuration& C, const uintA& vars, const uintA& cond);
  shared_ptr<NLP_Factored> nlp_timeFactored(const rai::Configuration& C);
  shared_ptr<NLP_Factored> nlp_fineFactored(const rai::Configuration& C);
  //-- to be removed (call generic NLPsolver)
  arr solve(const rai::Configuration& C, rai::ArgWord sequenceOrPath, int verbose=2);
  shared_ptr<SolverReturn> solve2(const rai::Configuration& C, int verbose=4);
  shared_ptr<SolverReturn> solve3(const rai::Configuration& C, bool useKeyframes, int verbose=4);

};
stdPipes(Skeleton)

} //namespace
