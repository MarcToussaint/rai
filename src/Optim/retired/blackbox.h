/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================

struct LocalModelBasedOptim {
  struct Datum {
    arr x;
    double f;
    double distToBest;
  };

  arr x_init;
  ScalarFunction f;
  rai::Array<Datum*> D; ///< data collected so far
  Datum* best;
  double alpha;
  OptOptions o;
  uint it, evals, numTinySteps;
//  Gradient::StopCriterion stopCriterion;
  ofstream fil;

  LocalModelBasedOptim(arr& x, const ScalarFunction& f, OptOptions o=NOOPT);
  ~LocalModelBasedOptim();
  void step();
  void run(uint maxIt = 1000);

 private:
  void evaluate(const arr& x, bool sort=true);

};
