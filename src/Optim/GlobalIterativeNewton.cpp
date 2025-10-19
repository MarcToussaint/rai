/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "GlobalIterativeNewton.h"

namespace rai {

bool useNewton=true;

GlobalIterativeNewton::GlobalIterativeNewton(ScalarFunction f, const arr& bounds, std::shared_ptr<OptOptions> opt)
  : x(.5*(bounds[0]+bounds[1])),
    newton(x, f, opt),
    grad(x, f, opt),
    bounds(bounds),
    best(nullptr) {
  newton.setBounds(bounds);
  newton.opt->verbose = 0;
}

GlobalIterativeNewton::~GlobalIterativeNewton() {
}

void addRun(GlobalIterativeNewton& gin, const arr& x, double fx, double tol) {
  GlobalIterativeNewton::LocalMinimum* found=nullptr;
  for(GlobalIterativeNewton::LocalMinimum& m:gin.localMinima) {
    double d = euclideanDistance(x, m.x);
    if(euclideanDistance(x, m.x)<tol) {
      if(!found) found = &m;
      else if(d<euclideanDistance(x, found->x)) found = &m;
    }
  }

  if(found) {
    found->hits++;
    if(fx<found->fx) {
      found->x = x;
      found->fx = fx;
    }
  } else {
    gin.localMinima.append({x, fx, 1});
    found = &gin.localMinima.last();
    gin.best = nullptr;
  }

  if(!gin.best) {
    gin.best = &gin.localMinima.first();
    for(GlobalIterativeNewton::LocalMinimum& m:gin.localMinima) if(m.fx < gin.best->fx) gin.best = &m;
  }
  if(found->fx<gin.best->fx) gin.best=found;
  gin.newton.x = gin.best->x;
  gin.newton.fx = gin.best->fx;
  if(gin.newton.opt->verbose>1) cout <<"***** optGlobalIterativeNewton: local minimum: " <<found->hits <<' ' <<found->fx <<' ' <<found->x <<endl;
}

void addRunFrom(GlobalIterativeNewton& gin, const arr& x) {
  if(useNewton) {
    gin.newton.reinit(x);
    gin.newton.run();
    addRun(gin, gin.newton.x, gin.newton.fx, 3.*gin.newton.opt->stopTolerance);
  } else {
    gin.grad.reinit(x);
    gin.grad.run();
    addRun(gin, gin.grad.x, gin.grad.f_x, 3.*gin.grad.opt->stopTolerance);
  }
}

void GlobalIterativeNewton::step() {
  arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(bounds.d1);
  if(newton.opt->verbose>1) cout <<"***** optGlobalIterativeNewton: new iteration from x=" <<x <<endl;
  addRunFrom(*this, x);
}

void GlobalIterativeNewton::run(uint maxIt) {
  for(uint i=0; i<maxIt; i++) {
    step();
  }
}

void GlobalIterativeNewton::report() {
  cout <<"# local minima = " <<localMinima.N <<endl;
  uint i=0;
  for(LocalMinimum& m:localMinima) {
    cout <<i++ <<' ' <<m.hits <<' ' <<m.fx <<" \t" <<m.x <<endl;
  }
}

void GlobalIterativeNewton::reOptimizeAllPoints() {
  if(!localMinima.N) return;
  arr X;
  for(LocalMinimum& m:localMinima) X.append(m.x);
  X.reshape(localMinima.N, X.N/localMinima.N);
  rndGauss(X, .01, true);
  localMinima.clear();
  for(uint i=0; i<X.d0; i++) addRunFrom(*this, X[i]);
}

} //namespace
