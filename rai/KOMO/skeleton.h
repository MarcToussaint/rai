#pragma once

#include "skeletonSymbol.h"

#include "../Core/util.h"
#include "../Core/array.h"
#include "../Kin/kin.h"

struct KOMO;

namespace rai {

//===========================================================================

struct SkeletonEntry {
  double phase0=-1.;
  double phase1=-1.;
  rai::Enum<SkeletonSymbol> symbol;
  StringA frames; //strings referring to things
  SkeletonEntry() {}
  SkeletonEntry(double phase0, double phase1, SkeletonSymbol symbol, StringA frames) : phase0(phase0), phase1(phase1), symbol(symbol), frames(frames) {}
  void write(ostream& os) const { os <<symbol <<' '; frames.write(os, " ", nullptr, "()"); os <<" from " <<phase0 <<" to " <<phase1; }
};
stdOutPipe(SkeletonEntry)

struct Skeleton {
  rai::Array<SkeletonEntry> S;
  shared_ptr<KOMO> komo;

  Skeleton() {}
  Skeleton(std::initializer_list<SkeletonEntry> entries) : S(entries) {}

  void solve(const Configuration& C);

  SkeletonEntry& operator()(int i) const { return S.elem(i); }
  double getMaxPhase() const;
  intA getSwitches(const Configuration& C) const;

  void setKOMO(KOMO& komo) const;

  void setKOMO(KOMO& komo, rai::ArgWord sequenceOrPath) const;


  void read(istream& is);
  void read_old(istream& is);
  void write(ostream& is, const intA& switches= {}) const;

 private:
  void ensure_komo();
};
stdPipes(Skeleton)

} //namespace
