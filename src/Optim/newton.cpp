/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <iomanip>

#include "newton.h"

bool sanityCheck=false; //true;

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
int optNewton(arr& x, const ScalarFunction& f,  OptOptions o) {
  return OptNewton(x, f, o).run();
}

//===========================================================================

OptNewton::OptNewton(arr& _x, const ScalarFunction& _f,  OptOptions _o):
  x(_x), f(_f), o(_o), it(0), evals(0), numTinySteps(0){
  alpha = o.initStep;
  beta = o.damping;
  additionalRegularizer=NULL;
  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x){
  if(&x!=&_x) x = _x;
  fx = f(gx, Hx, x);  evals++;
  if(additionalRegularizer)  fx += scalarProduct(x,(*additionalRegularizer)*vectorShaped(x));

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" beta=" <<beta <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;
}

//===========================================================================

OptNewton::StopCriterion OptNewton::step(){
  double fy;
  arr y, gy, Hy, Delta;
  bool betaChanged=false;

  it++;
  if(o.verbose>1) cout <<"optNewton it=" <<std::setw(4) <<it << " \tbeta=" <<std::setw(8) <<beta <<flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  //compute Delta
  arr R=Hx;
  if(beta) { //Levenberg Marquardt damping
    if(isRowShifted(R)) for(uint i=0; i<R.d0; i++) R(i,0) += beta; //(R(i,0) is the diagonal in the packed matrix!!)
    else for(uint i=0; i<R.d0; i++) R(i,i) += beta;
  }
  if(additionalRegularizer) { //obsolete -> retire
    if(isRowShifted(R)) R = unpack(R);
    Delta = lapack_Ainv_b_sym(R + (*additionalRegularizer), -(gx+(*additionalRegularizer)*vectorShaped(x)));
  } else {
    bool inversionFailed=false;
    try {
      Delta = lapack_Ainv_b_sym(R, -gx);
    }catch(...){
      inversionFailed=true;
    }
    if(inversionFailed){
      if(false){ //increase beta
        arr sig = lapack_kSmallestEigenValues_sym(R, 3);
        if(o.verbose>0){
          cout <<"** hessian inversion failed ... increasing damping **\neigenvalues=" <<sig <<endl;
        }
        double sigmin = sig.min();
        if(sigmin>0.) THROW("Hessian inversion failed, but eigenvalues are positive???");
        beta = 2.*beta - sigmin;
        betaChanged=true;
        return stopCriterion=stopNone;
      }else{ //use gradient
        if(o.verbose>0){
          cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
        }
        Delta = -gx / length(gx) * o.maxStep;
//        alpha = 1.;
      }
    }
  }

  //restrict stepsize
  double maxDelta = absMax(Delta);
  if(o.maxStep>0. && maxDelta>o.maxStep){  Delta *= o.maxStep/maxDelta; maxDelta = o.maxStep; }
  double alphaLimit = o.maxStep/maxDelta;

  //...due to bounds
  if(bound_lo.N && bound_hi.N){
    double a=1.;
    for(uint i=0;i<x.N;i++){
      if(x(i)+a*Delta(i)>bound_hi(i)) a = (bound_hi(i)-x(i))/Delta(i);
      if(x(i)+a*Delta(i)<bound_lo(i)) a = (bound_lo(i)-x(i))/Delta(i);
    }
    Delta *= a;
  }
  if(o.verbose>1) cout <<" \t|Delta|=" <<std::setw(11) <<maxDelta <<flush;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*o.stopTolerance){
    if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
    return stopCriterion=stopCrit1;
  }

  for(;!betaChanged;) { //line search
    if(!o.allowOverstep) if(alpha>1.) alpha=1.;
    if(alphaLimit>0. && alpha>alphaLimit) alpha=alphaLimit;
    y = x + alpha*Delta;
    fy = f(gy, Hy, y);  evals++;
    if(additionalRegularizer) fy += scalarProduct(y,(*additionalRegularizer)*vectorShaped(y));
    if(o.verbose>2) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<flush;
    bool wolfe = (fy <= fx + o.wolfe*alpha*scalarProduct(Delta,gx) );
    if(fy==fy && (wolfe || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)it)) { //fy==fy is for NAN?
      //accept new point
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      if(fx-fy<o.stopFTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      Hx = Hy;
      if(wolfe){
        if(alpha>.9 && beta>o.damping){
          beta *= o.dampingDec;
          if(alpha>1.) alpha=1.;
          betaChanged=true;
        }
        alpha *= o.stepInc;
      }else{
        //this is the nonStrict case... weird, but well
        if(alpha<.01){
          beta*=o.dampingInc;
          alpha*=o.dampingInc*o.dampingInc;
          betaChanged=true;
          if(o.verbose>1) cout <<"(line search stopped)" <<endl;
        }
        alpha *= o.stepDec;
      }
      break;
    } else {
      //reject new point
      if(o.verbose>1) cout <<" - reject" <<flush;
      if(evals>o.stopEvals){
        if(o.verbose>1) cout <<" (evals>stopEvals)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(alpha<.01){
        beta*=o.dampingInc;
        alpha*=o.dampingInc*o.dampingInc;
        betaChanged=true;
        if(o.verbose>1) cout <<", stop & betaInc" <<endl;
      }else{
        if(o.verbose>1) cout <<"\n\t\t\t\t\t(line search)\t" <<flush;
      }
      alpha *= o.stepDec;
    }
  }

  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  //stopping criteria
#define STOPIF(expr, code, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }
  STOPIF(absMax(Delta)<o.stopTolerance, , stopCrit1);
  STOPIF(numTinySteps>10, numTinySteps=0, stopCrit2);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=o.stopEvals, , stopCritEvals);
  STOPIF(it>=o.stopIters, , stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}


OptNewton::~OptNewton(){
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MLR_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
#endif
  if(o.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
}


OptNewton::StopCriterion OptNewton::run(uint maxIt){
  numTinySteps=0;
  for(uint i=0;i<maxIt;i++){
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopCrit1) break;
  }
  return stopCriterion;
}
