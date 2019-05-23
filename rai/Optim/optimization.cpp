/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "optimization.h"

uint eval_count=0;
Singleton<OptOptions> globalOptOptions;
ObjectiveTypeA& NoTermTypeA = *((ObjectiveTypeA*)NULL);

template<> const char* rai::Enum<ObjectiveType>::names []= {
  "none", "f", "sos", "ineq", "eq", NULL
};

//===========================================================================
//
// checks and converters
//

bool checkJacobianCP(ConstrainedProblem &P, const arr& x, double tolerance) {
  VectorFunction F = [&P](arr& phi, arr& J, const arr& x) {
    return P.phi(phi, J, NoArr, NoTermTypeA, x, NoArr);
  };
  return checkJacobian(F, x, tolerance);
}

bool checkHessianCP(ConstrainedProblem &P, const arr& x, double tolerance) {
  uint i;
  arr phi, J;
  ObjectiveTypeA tt;
  P.phi(phi, NoArr, NoArr, tt, x, NoArr); //TODO: only call getStructure
  for(i=0; i<tt.N; i++) if(tt(i)==OT_f) break;
  if(i==tt.N) {
    RAI_MSG("no f-term in this KOM problem");
    return true;
  }
  ScalarFunction F = [&P,&phi,&J,i](arr& g, arr& H, const arr& x) -> double{
    P.phi(phi, J, H, NoTermTypeA, x, NoArr);
    g = J[i];
    return phi(i);
  };
  return checkHessian(F, x, tolerance);
}

//===========================================================================
//
// optimization options
//

OptOptions::OptOptions() {
  verbose    = rai::getParameter<int> ("opt/verbose", 1);
  fmin_return=NULL;
  stopTolerance= rai::getParameter<double>("opt/stopTolerance", 1e-2);
  stopFTolerance= rai::getParameter<double>("opt/stopFTolerance", 1e-1);
  stopGTolerance= rai::getParameter<double>("opt/stopGTolerance", -1.);
  stopEvals = rai::getParameter<uint> ("opt/stopEvals", 1000);
  stopIters = rai::getParameter<uint> ("opt/stopIters", 1000);
  stopOuters = rai::getParameter<uint> ("opt/stopOuters", 1000);
  stopLineSteps = rai::getParameter<uint> ("opt/stopLineSteps", 10);
  stopTinySteps = rai::getParameter<uint> ("opt/stopTinySteps", 10);
  initStep  = rai::getParameter<double>("opt/initStep", 1.);
  minStep   = rai::getParameter<double>("opt/minStep", -1.);
  maxStep   = rai::getParameter<double>("opt/maxStep", .2);
  damping   = rai::getParameter<double>("opt/damping", 1.);
  stepInc   = rai::getParameter<double>("opt/stepInc", 1.5);
  stepDec   = rai::getParameter<double>("opt/stepDec", .5);
  dampingInc= rai::getParameter<double>("opt/dampingInc", 1.);
  dampingDec= rai::getParameter<double>("opt/dampingDec", 1.);
  wolfe     = rai::getParameter<double>("opt/wolfe", .01);
  nonStrictSteps= rai::getParameter<uint> ("opt/nonStrictSteps", 0);
  allowOverstep= rai::getParameter<bool> ("opt/allowOverstep", false);
  constrainedMethod = (ConstrainedMethodType)rai::getParameter<int>("opt/constrainedMethod", augmentedLag);
  muInit = rai::getParameter<double>("opt/muInit", 1.);
  muLBInit = rai::getParameter<double>("opt/muLBInit", 1.);
  aulaMuInc = rai::getParameter<double>("opt/aulaMuInc", 2.);
}

void OptOptions::write(std::ostream& os) const {
#define WRT(x) os <<#x <<" = " <<x <<endl;
  WRT(verbose);
//  double *fmin_return);
  WRT(stopTolerance);
  WRT(stopEvals);
  WRT(stopIters);
  WRT(initStep);
  WRT(minStep);
  WRT(maxStep);
  WRT(damping);
  WRT(stepInc);
  WRT(stepDec);
  WRT(dampingInc);
  WRT(dampingDec);
  WRT(nonStrictSteps);
  WRT(allowOverstep);
  WRT(constrainedMethod);
  WRT(aulaMuInc);
#undef WRT
}

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction &f, bool wait, double lo, double hi) {
  arr X, Y;
  X.setGrid(2,lo,hi,100);
  Y.resize(X.d0);
  for(uint i=0; i<X.d0; i++) {
    double fx=f(NoArr, NoArr, X[i]);
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101,101);
//  plotGnuplot();  plotSurface(Y);  plot(true);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", wait, true);
}

/// minimizes \f$f(x)\f$ using its gradient only
uint optGradDescent(arr& x, const ScalarFunction& f, OptOptions o) {
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=o.initStep;
  
  fx = f(grad_x, NoArr, x);  evals++;
  if(o.verbose>1) cout <<"*** optGradDescent: starting point x=" <<(x.N<20?x:arr()) <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_count <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
  
  grad_x /= length(grad_x);
  
  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f(grad_y, NoArr, y);  evals++;
    CHECK_EQ(fy,fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_count <<" \tprobing y=" <<(y.N<20?y:arr()) <<" \tf(y)=" <<fy <<" \t|grad|=" <<length(grad_y) <<" \ta=" <<a;
    
    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=length(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/length(grad_y);
      a *= 1.2;
      if(o.maxStep>0. && a>o.maxStep) a = o.maxStep;
      if(o.verbose>0) fil <<evals <<' ' <<eval_count <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
      if(step<o.stopTolerance) break;
    } else {
      if(o.verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
    if(k>o.stopIters) break;
  }
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l",NULL,true);
  return evals;
}

RUN_ON_INIT_BEGIN(optimization)
ObjectiveTypeA::memMove=true;
RUN_ON_INIT_END(optimization)
