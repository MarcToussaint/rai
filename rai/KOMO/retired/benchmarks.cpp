/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "benchmarks.h"
#include "komo.h"
//#include "../Optim/kOrderMarkov.h"
#include "../Optim/convert.h"

void setTasks(KOMO& MP,
              rai::Shape& endeff,
              rai::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

struct sPR2EndPoseProblem {
  rai::Configuration world;
  KOMO MP;
  Convert* CP;
  sPR2EndPoseProblem()
    :world("model.kvg"), MP(world), CP(nullptr) {}
};

PR2EndPoseProblem::PR2EndPoseProblem()
  : s(*(new sPR2EndPoseProblem())) {

  for(rai::Shape* sh:s.world.shapes) sh->cont=true;

  setTasks(s.MP, *s.world.getShapeByName("endeff"), *s.world.getShapeByName("target"), 0, 1, 0, 5.);

  s.CP = new Convert(s.MP.komo_problem);
  ConstrainedProblem::operator=(*s.CP);  //conv_KOrderMarkovFunction2ConstrainedProblem(s.MP.komo_problem) );
}

arr PR2EndPoseProblem::getInitialization() {
  arr x = s.MP.getInitialization();
  rndGauss(x, .01, true); //don't initialize at a singular config
  return x;
}

void PR2EndPoseProblem::report() {
  cout <<s.MP.getReport();
  s.world.watch(true);
}

void PR2EndPoseProblem::setState(const arr& x) {
  s.world.setJointState(x);
}

