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
