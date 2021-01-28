/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "BayesOpt.h"
#include "RidgeRegression.h"
//#include "../Plot/plot.h"
//#include "../Algo/MLcourse.h"

BayesOpt::BayesOpt(const ScalarFunction& _f, const arr& bounds_lo, const arr& bounds_hi, double init_lengthScale, double prior_var, OptOptions o)
  : f(_f),
    bounds_lo(bounds_lo), bounds_hi(bounds_hi),
    f_now(nullptr), f_smaller(nullptr),
    alphaMinima_now(ScalarFunction(), bounds_lo, bounds_hi),
    alphaMinima_smaller(ScalarFunction(), bounds_lo, bounds_hi) {

  init_lengthScale *= sum(bounds_hi - bounds_lo)/bounds_lo.N;

  kernel_now = new DefaultKernelFunction();
  kernel_smaller = new DefaultKernelFunction();

  kernel_now->type = kernel_smaller->type = DefaultKernelFunction::Gauss; //TODO: ugly!!

  kernel_now->hyperParam1 = ARR(init_lengthScale);
  kernel_now->hyperParam2 = ARR(prior_var);
  kernel_smaller->hyperParam1 = kernel_now->hyperParam1;
  kernel_smaller->hyperParam1 /= 2.;
  kernel_smaller->hyperParam2 = kernel_now->hyperParam2;
}

BayesOpt::~BayesOpt() {
  delete kernel_now;
  delete kernel_smaller;
  delete f_now;
  delete f_smaller;

}

void BayesOpt::step() {
  arr x;
  if(!data_X.N) {
    x = bounds_lo + (bounds_hi-bounds_lo) % rand(bounds_lo.N);
  } else {
    x = pickNextPoint();
  }

  double fx = f(NoArr, NoArr, x);
//  report();

  addDataPoint(x, fx);

  reOptimizeAlphaMinima();
}

void BayesOpt::run(uint maxIt) {
  for(uint i=0; i<maxIt; i++) step();
}

void BayesOpt::report(bool display, const ScalarFunction& f) {
  if(!f_now) return;
  cout <<"mean=" <<f_now->mu <<" var=" <<kernel_now->hyperParam2.scalar() <<endl;

  arr X_grid, s_grid;
  X_grid.setGrid(data_X.d1, 0., 1., (data_X.d1==1?500:30));
  X_grid = X_grid % (bounds_hi-bounds_lo);
  X_grid += repmat(bounds_lo, X_grid.d0, 1);
  arr y_grid = f_now->evaluate(X_grid, s_grid);
  s_grid = sqrt(s_grid);

  arr f_grid(X_grid.d0);
  if(f) for(uint i=0; i<X_grid.d0; i++) f_grid(i) = f(NoArr, NoArr, X_grid[i]);

  arr s2_grid;
  arr y2_grid = f_smaller->evaluate(X_grid, s2_grid);
  s2_grid = sqrt(s2_grid);

  arr locmin_X(0u, data_X.d1), locmin_y;
  for(auto& l:alphaMinima_now.localMinima) {
    locmin_X.append(l.x);
    locmin_y.append(l.fx);
  }
  arr locmin2_X(0u, data_X.d1), locmin2_y;
  for(auto& l:alphaMinima_smaller.localMinima) {
    locmin2_X.append(l.x);
    locmin2_y.append(l.fx);
  }

#if 1
  HALT("dependence on plot deprecated")
#else
  plot()->Gnuplot();
  plot()->Clear();
  plot()->FunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
//  plot()->Function(X_grid, y2_grid);
  plot()->Function(X_grid, y_grid-2.*s_grid);
  plot()->Function(X_grid, y2_grid-2.*s2_grid);
  if(f) plot()->Function(X_grid, f_grid);
  plot()->Points(data_X, data_y);
//  plot()->Points(locmin_X, locmin_y);
//  plot()->Points(locmin2_X, locmin2_y);
  plot()->update(false);
#endif
}

void BayesOpt::addDataPoint(const arr& x, double y) {
  if(f_now) delete f_now;
  if(f_smaller) delete f_smaller;

  data_X.append(x);  data_X.reshape(data_X.N/x.N, x.N);
  data_y.append(y);

  double fmean = sum(data_y)/data_y.N;
  if(data_y.N>2) {
    kernel_now->hyperParam2 = 2.*var(data_y);
    kernel_smaller->hyperParam2 = kernel_now->hyperParam2;
  }

  f_now = new KernelRidgeRegression(data_X, data_y, *kernel_now, -1., fmean);
  f_smaller = new KernelRidgeRegression(data_X, data_y, *kernel_smaller, -1., fmean);
}

void BayesOpt::reOptimizeAlphaMinima() {
  alphaMinima_now.newton.f = f_now->getF(-2.);
  alphaMinima_smaller.newton.f = f_smaller->getF(-2.);

  alphaMinima_now.reOptimizeAllPoints();
  alphaMinima_now.run(20);
  alphaMinima_smaller.reOptimizeAllPoints();
  alphaMinima_smaller.run(20);
}

arr BayesOpt::pickNextPoint() {
  arr x_now = alphaMinima_now.best->x;
  arr x_sma = alphaMinima_smaller.best->x;

  double fx_0 = f_now->evaluate(x_now, NoArr, NoArr, -2., false);
  double fx_1 = f_smaller->evaluate(x_sma, NoArr, NoArr, -1., false);

  if(fx_1 < fx_0) {
    reduceLengthScale();
    return x_sma;
  }

  return x_now;
}

void BayesOpt::reduceLengthScale() {
  cout <<"REDUCING LENGTH SCALE!!" <<endl;
  kernel_now->hyperParam1 = kernel_smaller->hyperParam1;
  kernel_smaller->hyperParam1 /= 2.;
}
