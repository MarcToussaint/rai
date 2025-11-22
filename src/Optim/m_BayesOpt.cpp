/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "m_BayesOpt.h"
#include "../Algo/RidgeRegression.h"
#include "../Core/plot.h"
//#include "../Algo/MLcourse.h"

namespace rai {

BayesOpt::BayesOpt(shared_ptr<NLP> P, shared_ptr<OptOptions> opt, double init_lengthScale, double prior_var)
    : GenericBO("BayesOpt", P, P->bounds[0] + (P->bounds[1]-P->bounds[0]) % rand(P->bounds.d1), opt),
    alphaMinima(P->f_scalar(), P->bounds, opt) {

  lengthScale = init_lengthScale * length(P->bounds[1] - P->bounds[0]);

  kernel = make_shared<DefaultKernelFunction>();

  kernel->type = DefaultKernelFunction::Gauss; //TODO: ugly!!

  kernel->lengthScaleSqr = rai::sqr(lengthScale);
  kernel->priorVar = prior_var;

  lambda = 1e-9;
}

arr BayesOpt::generateSamples(){
  arr x;
  if(!data_X.N) {
    x = P->getInitializationSample();
  } else {
    x = alphaMinima.best->x;
  }
  return ~x;
}

void BayesOpt::update(arr& X, arr& Phi, arr& F){
  //-- add data and recompute GP
  kernel->lengthScaleSqr = rai::sqr(lengthScale);
  addDataPoint(X[0], F.elem());

  //-- reoptimize local minima
  alphaMinima.f = gp_model->getF(-2.);
  alphaMinima.reOptimizeLocalMinima();

  //-- add more local minima
  alphaMinima.run(10);

  // alphaMinima.report();
  double potentialImprovement = best_f - alphaMinima.best->fx;
  if(opt->verbose>1) cout <<" best: " <<best_f <<" alpha improvement: " <<potentialImprovement <<" #local optima: " <<alphaMinima.localMinima.N;
  if(potentialImprovement<1e-3){
    lengthScale *= .5;
    cout <<"[new length scale: " <<lengthScale <<"]";
  }
}

void BayesOpt::run(uint maxIt) {
  for(uint i=0; i<maxIt; i++) step();
}

void BayesOpt::report() {
  if(!gp_model) return;
  // cout <<"mean=" <<gp_model->mu <<" var=" <<kernel->priorVar.scalar() <<endl;

  arr X_grid, s_grid;
  X_grid = grid(data_X.d1, 0., 1., (data_X.d1==1?500:30));
  X_grid = X_grid % (P->bounds[1]-P->bounds[0]);
  X_grid += repmat(P->bounds[0], X_grid.d0, 1);
  arr y_grid = gp_model->evaluate(X_grid, s_grid);
  for(double &s:s_grid) if(s>1e-10) s=2.*sqrt(s);

  arr f_grid(X_grid.d0);
  for(uint i=0; i<X_grid.d0; i++) f_grid(i) = P->eval_scalar(NoArr, NoArr, X_grid[i]);

  arr a_grid(X_grid.d0);
  for(uint i=0; i<X_grid.d0; i++) a_grid(i) = alphaMinima.f(NoArr, NoArr, X_grid[i]);

  arr locmin_X(0u, data_X.d1), locmin_y;
  for(auto& l:alphaMinima.localMinima) {
    locmin_X.append(l.x);
    locmin_y.append(l.fx);
  }

  plot()->Gnuplot();
  plot()->Clear();
  plot()->FunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
  // plot()->Function(X_grid, y_grid-2.*s_grid);
  plot()->Function(X_grid, a_grid);
  plot()->Function(X_grid, f_grid);
  plot()->Points(data_X, data_y);
  plot()->Points(locmin_X, locmin_y);
  plot()->update(false);
}

void BayesOpt::addDataPoint(const arr& x, double y) {

  data_X.append(x);  data_X.reshape(data_X.N/x.N, x.N);
  data_y.append(y);

  double fmean = ::sum(data_y)/data_y.N;
  if(data_y.N>4) {
    kernel->priorVar = 2.*var(data_y);
  }

  gp_model = make_shared<KernelRidgeRegression>(data_X, data_y, *kernel, lambda, fmean);
}

} //namespace
