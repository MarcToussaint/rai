/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Core/array.h"

struct ComputeReport {
  double time;
  bool feasible;
  double cost_sos;
  double constraints_ineq, constraints_eq;
};

struct ComputeObject {
  ptr<ComputeReport> report;

  virtual ~ComputeObject() {}

  virtual ptr<ComputeReport> run(double timeBudget=-1.) = 0;
  virtual void restart() { NIY }
};
