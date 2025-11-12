#pragma once

#include "../Kin/kin.h"

struct RndStableConfigs_Options {
  RAI_PARAM("RndStableConfigs/", int, verbose, 1)
  RAI_PARAM("RndStableConfigs/", double, frictionCone_mu, .8)
};

struct RndStableConfigs {
  RndStableConfigs_Options opt;
  uint totalEvals=0, totalSucc=0;
  bool savePngs=false;
  strA supp;
  intA pairs;
  arr forces;

  bool getSample(rai::Configuration& C, const StringA& supports);
  void report();
};
