/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "m_RestartNewton.h"

namespace rai {

bool useNewton=true;

RestartNewton::RestartNewton(ScalarFunction f, const arr& bounds, std::shared_ptr<OptOptions> opt)
    : f(f),
    opt(opt),
    bounds(bounds),
    best(nullptr) {
}

RestartNewton::~RestartNewton() {
}

void RestartNewton::addRunFrom(arr& x) {
  //-- run Newton
  OptNewton newton(x, f, make_shared<OptOptions>(*opt));
  newton.setBounds(bounds);
  newton.opt->verbose = 10;
  checkGradient(newton.f, x, 1e-6, true);
  checkHessian(newton.f, x, 1e-6, true);
  newton.run();

  //-- check if we already have this local optimum
  RestartNewton::LocalMinimum* found=nullptr;
  for(RestartNewton::LocalMinimum& m:localMinima) {
    double d = euclideanDistance(x, m.x);
    if(euclideanDistance(x, m.x)<3.*newton.opt->stopTolerance) {
      if(!found) found = &m;
      else if(d<euclideanDistance(x, found->x)) found = &m;
    }
  }

  //-- if yes, increase hits and slightly update
  if(found) {
    found->hits++;
    if(newton.fx<found->fx) {
      found->x = x;
      found->fx = newton.fx;
    }
  } else {
    localMinima.append({x, newton.fx, 1});
    found = &localMinima.last();
    best = nullptr;
  }

  //-- update best
  if(!best) {
    best = &localMinima.first();
    for(RestartNewton::LocalMinimum& m:localMinima) if(m.fx < best->fx) best = &m;
  }
  if(found->fx<best->fx) best=found;

  if(opt->verbose>2) cout <<"--restartNewton-- local minimum: " <<found->hits <<' ' <<found->fx <<' ' <<found->x <<endl;
}

void RestartNewton::step() {
  arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(bounds.d1);
  // if(opt->verbose>1) cout <<"***** optGlobalIterativeNewton: new iteration from x=" <<x <<endl;
  addRunFrom(x);
}

void RestartNewton::run(uint maxIt) {
  for(uint i=0; i<maxIt; i++) {
    step();
  }
}

void RestartNewton::report() {
  cout <<"# local minima = " <<localMinima.N <<endl;
  uint i=0;
  for(LocalMinimum& m:localMinima) {
    cout <<i++ <<' ' <<m.hits <<' ' <<m.fx <<" \t" <<m.x <<endl;
  }
}

void RestartNewton::reOptimizeLocalMinima() {
  if(!localMinima.N) return;
  arr X;
  for(LocalMinimum& m:localMinima) X.append(m.x);
  X.reshape(localMinima.N, X.N/localMinima.N);
  rndGauss(X, 1e-6, true);

  localMinima.clear();
  for(uint i=0; i<X.d0; i++){
    boundClip(X[i].noconst(), bounds);
    addRunFrom(X[i].noconst());
  }
}

} //namespace
