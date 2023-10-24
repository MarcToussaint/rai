/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "percept.h"
#include "../Core/thread.h"

/// syncs percepts with modelWorld
struct SyncFiltered : Thread {
  Var<PerceptL> percepts;
  Var<rai::Configuration> kin;

  SyncFiltered(Var<PerceptL>& _percepts, Var<rai::Configuration>& _kin);
  ~SyncFiltered();

  virtual void step();
};
