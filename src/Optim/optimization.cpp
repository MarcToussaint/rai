/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "optimization.h"

uint eval_cost=0;
//SqrPotential& NoPot = *((SqrPotential*)NULL);
//PairSqrPotential& NoPairPot = *((PairSqrPotential*)NULL);
OptOptions globalOptOptions;

//===========================================================================
//
// misc (internal)
//

//void init(SqrPotential &V, uint n) { V.A.resize(n,n); V.a.resize(n); V.A.setZero(); V.a.setZero(); V.c=0.; }


//documentations... TODO: move! but not in header!

/// A scalar function $y = f(x)$, if @grad@ is not NoArr, gradient is returned
struct ScalarFunction;

/// A vector function $y = f(x)$, if @J@ is not NoArr, Jacobian is returned
/// This also implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of (iterated)
/// Gauss-Newton type where the Hessian is approximated by J^T J
struct VectorFunction;

/// A scalar function $y = f(x)$, if @S@ is non-NULL, local quadratic approximation is returned
/// This also implies an optimization problem of (iterated) Newton
/// type with the given Hessian
struct QuadraticFunction;


//===========================================================================
//
// checks, evaluation and converters
//

double evaluateSF(ScalarFunction& f, const arr& x) {
  return f.fs(NoArr, NoArr, x);
}

double evaluateVF(VectorFunction& f, const arr& x) {
  arr y;
  f.fv(y, NoArr, x);
  return sumOfSqr(y);
}


bool checkAllGradients(ConstrainedProblem &P, const arr& x, double tolerance){
  struct F:ScalarFunction{
    ConstrainedProblem &P;
    F(ConstrainedProblem &_P):P(_P){}
    double fs(arr& g, arr& H, const arr& x){  return P.fc(g, H, NoArr, NoArr, x); }
  } f(P);
  struct G:VectorFunction{
    ConstrainedProblem &P;
    G(ConstrainedProblem &_P):P(_P){}
    void fv(arr& y, arr& J, const arr& x){ P.fc(NoArr, NoArr, y, J, x); }
  } g(P);

  bool good=true;
  good &= checkGradient(f, x, tolerance);
  good &= checkHessian (f, x, tolerance);
  good &= checkJacobian(g, x, tolerance);
  return good;
}


//===========================================================================
//
// optimization methods
//

OptOptions::OptOptions() {
  verbose=0;
  fmin_return=NULL;
  stopTolerance=1e-2;
  stopEvals=1000;
  stopIters=1000;
  initStep=1.;
  minStep=-1.;
  maxStep=-1.;
  damping=1.;
  stepInc=.01; stepDec=.1;
  dampingInc=1.; dampingDec=.9;
  nonStrict=false;
  useAdaptiveDamping=false;
  clampInitialState=false;
  constrainedMethod=augmentedLag;
}

OptOptions global_optOptions;



/// minimizes \f$f(x)\f$ using its gradient only
uint optGradDescent(arr& x, ScalarFunction& f, OptOptions o) {
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=o.initStep;
  
  fx = f.fs(grad_x, NoArr, x);  evals++;
  if(o.verbose>1) cout <<"*** optGradDescent: starting point x=" <<x <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
  
  grad_x /= length(grad_x);
  
  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f.fs(grad_y, NoArr, y);  evals++;
    CHECK(fy==fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_cost <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|grad|=" <<length(grad_y) <<" \ta=" <<a;
    
    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=length(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/length(grad_y);
      a *= 1.2;
      if(o.maxStep>0. && a>o.maxStep) a = o.maxStep;
      if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
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


