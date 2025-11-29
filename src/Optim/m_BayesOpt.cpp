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

  kernel = make_shared<DefaultKernelFunction>(DefaultKernelFunction::Gauss);

  kernel->lengthScaleSqr = rai::sqr(lengthScale);
  kernel->priorVar = prior_var;
}

arr BayesOpt::generateSamples(){
  arr x;
  if(!data_X.N) {
    x = P->getInitializationSample();
  } else if(!thompsonMethod) {
    x = alphaMinima.best->x;
  } else {
    x = thompsonMinima.x;
  }
  return ~x;
}

void BayesOpt::updateAlphaMinima(){
  if(!leastSquaresCase){
    alphaMinima.f = gp_model->getF(-2.);
  }else{
    alphaMinima.f = gp_model->getFSquare(-2.);
  }
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

void BayesOpt::updateThompsonMinima(){
  uint N=100;
  thompsonSampleX = rand(uintA{N, P->dimension})%(P->bounds[1]-P->bounds[0]) + P->bounds[0];
  // thompsonSampleX = grid(P->dimension, 0., 1., N)% (P->bounds[1]-P->bounds[0]) + P->bounds[0];
  arr Sigma;
  arr mu = gp_model->evaluate(thompsonSampleX, Sigma, true);
#if 0
  arr V, sig2;
  lapack_EigenDecomp(Sigma, sig2, V);
  // cout <<"eigen err: " <<maxDiff(Sigma, ~V * (sig2%V)) <<endl;
  // cout <<"eigen vals: " <<s <<endl;
  // // cout <<"Sigma smallest eigen values: " <<lapack_kSmallestEigenValues_sym(Sigma, 5) <<endl;
  for(double &s: sig2){ if(s>1e-10) s=sqrt(s); }
  arr C = sig2%V;
#else
  for(uint i=0;i<Sigma.d0;i++) Sigma(i,i) += 1e-6;
  // // cout <<Sigma <<endl;
  arr C = lapack_cholesky(Sigma);
#endif
  thompsonSampleF = ~C * randn(thompsonSampleX.d0) + mu;
  if(leastSquaresCase){
    thompsonSampleF = ::sum(::sqr(thompsonSampleF), 1);
  }

  uint i = argmin(thompsonSampleF);
  thompsonMinima.x = thompsonSampleX[i];
  thompsonMinima.fx = thompsonSampleF.elem(i);
}

void BayesOpt::update(arr& X, arr& Phi, arr& F){
  //-- add data and recompute GP
  kernel->lengthScaleSqr = rai::sqr(lengthScale);
  CHECK_EQ(X.d0, 1, "");
  if(!leastSquaresCase){
    addDataPoint(X[0], F);
  }else{
    addDataPoint(X[0], Phi[0]);
  }

  if(!thompsonMethod){
    updateAlphaMinima();
  }else{
    updateThompsonMinima();
  }
}

void BayesOpt::run(uint maxIt) {
  for(uint i=0; i<maxIt; i++) step();
}

void BayesOpt::report() {
  if(!gp_model) return;
  // cout <<"mean=" <<gp_model->mu <<" var=" <<kernel->priorVar.scalar() <<endl;

  if(P->dimension>2) return;

  arr X_grid, s_grid;
  uint n_grid = (data_X.d1==1?500:30);
  X_grid = grid(data_X.d1, 0., 1., n_grid);
  X_grid = X_grid % (P->bounds[1]-P->bounds[0]);
  X_grid += repmat(P->bounds[0], X_grid.d0, 1);
  arr y_grid = gp_model->evaluate(X_grid, s_grid);
  // for(uint i=0; i<X_grid.d0; i++) y_grid(i) = gp_model->evaluateSquare(X_grid[i], NoArr, NoArr,+2., false);
  for(double &s:s_grid) if(s>1e-10) s=2.*sqrt(s);

  if(leastSquaresCase){
    y_grid = ::sum(::sqr(y_grid),1);
  }

  arr f_grid(X_grid.d0);
  for(uint i=0; i<X_grid.d0; i++) f_grid(i) = P->eval_scalar(NoArr, NoArr, X_grid[i]);

  arr a_grid(X_grid.d0);
  if(!thompsonMethod){
    for(uint i=0; i<X_grid.d0; i++) a_grid(i) = alphaMinima.f(NoArr, NoArr, X_grid[i]);
  }

  arr locmin_X(0u, data_X.d1), locmin_y;
  for(auto& l:alphaMinima.localMinima) {
    locmin_X.append(l.x);
    locmin_y.append(l.fx);
  }
  if(thompsonMethod){
    for(uint i=0;i<thompsonSampleX.d0;i++) {
      locmin_X.append(thompsonSampleX[i]);
      locmin_y.append(thompsonSampleF.elem(i));
    }
  }

  if(P->dimension==2){
    f_grid.reshape(n_grid+1,n_grid+1);
    y_grid.reshape(n_grid+1,n_grid+1);


    FILE("z.fct") <<f_grid.modRaw() <<endl;
    FILE("z.fcty") <<y_grid.modRaw() <<endl;
    FILE("z.pts") <<catCol({data_X, data_y.reshape(-1,1)}).modRaw() <<endl;
    FILE("z.pts2") <<catCol({locmin_X, locmin_y.reshape(-1,1)}).modRaw() <<endl;
    rai::String cmd;
    // cmd <<"reset; set contour; set size square; set cntrparam linear; set cntrparam levels incremental 0,.1,10; set xlabel 'x'; set ylabel 'y'; ";
    cmd <<"reset; set size square; set xlabel 'x'; set ylabel 'y'; ";
    rai::String splot;
    arr B = P->bounds;
    splot <<"splot [" <<B(0,0) <<':' <<B(1,0) <<"][" <<B(0,1) <<':' <<B(1,1) <<"] "
          <<"'z.fct' matrix us (" <<B(0,0) <<"+(" <<B(1,0)-B(0,0) <<")*$2/"<<n_grid<<"):(" <<B(0,1) <<"+(" <<B(1,1)-B(0,1) <<")*$1/"<<n_grid<<"):3 w l, ";
    splot <<"'z.fcty' matrix us (" <<B(0,0) <<"+(" <<B(1,0)-B(0,0) <<")*$2/"<<n_grid<<"):(" <<B(0,1) <<"+(" <<B(1,1)-B(0,1) <<")*$1/"<<n_grid<<"):3 w l,";
    splot <<"'z.pts' w p ps 1 lt -1 pt 7,";
    splot <<"'z.pts2' w p ps 1 lt 1 pt 6";
    cmd <<splot <<";";
    gnuplot(cmd);

    return;
  }

  plot()->Gnuplot();
  plot()->Clear();
  plot()->Function(X_grid, f_grid);
  // plot()->Function(X_grid, y_grid-2.*s_grid);
  if(!thompsonMethod){
    plot()->FunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
    plot()->Function(X_grid, a_grid);
  }else{
    plot()->Function(X_grid, y_grid);
  }
  plot()->Points(locmin_X, locmin_y);
  if(!leastSquaresCase){
    plot()->Points(data_X, data_y);
  }else{
    arr Y = ::sum(::sqr(data_y),1);
    plot()->Points(data_X,Y);
  }
  plot()->update(false);
}

void BayesOpt::addDataPoint(const arr& x, const arr& y) {

  data_X.append(x);  data_X.reshape(-1, x.N);
  data_y.append(y);
  if(leastSquaresCase) data_y.reshape(-1, y.N);

  double fmean = 0.;

  if(!leastSquaresCase){
    fmean = ::sum(data_y)/data_y.N;
    if(data_y.N>4) {
      kernel->priorVar = 2.*var(data_y);
    }
  }

  gp_model = make_shared<KernelRidgeRegression>(data_X, data_y, *kernel, lambda, true);
}

} //namespace
