#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct LocalGreedy {
  RAI_PARAM("LocalGreedy/", double, sigma, .1)

  shared_ptr<NLP> P;


  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  double f_x=0.;

  //-- counters
  uint steps=0, tinySteps=0, maxSteps=300;

  LocalGreedy(shared_ptr<NLP> P, const arr& x_init={});

  shared_ptr<SolverReturn> solve(){
    while(!step()){}
    shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
    ret->x = x;
    ret->f = f_x;
    ret->feasible=true;
    return ret;
  }

  bool step();
};
