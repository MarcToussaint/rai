/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "newton.h"
#include <iomanip>

bool sanityCheck=false; //true;

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
int optNewton(arr& x, const ScalarFunction& f,  OptOptions o) {
  OptNewton opt(x, f, o);
  ofstream fil("z.opt");
  opt.simpleLog = &fil;
  return opt.run();
}

//===========================================================================

OptNewton::OptNewton(arr& _x, const ScalarFunction& _f,  OptOptions _o, ostream* _logFile):
  x(_x), f(_f), o(_o), logFile(_logFile) {
  alpha = o.initStep;
  beta = o.damping;
  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x) {
  if(&x!=&_x) x = _x;
  fx = f(gx, Hx, x);  evals++;

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" beta=" <<beta <<endl;
  if(o.verbose>3){ if(x.N<5) cout <<"x=" <<x <<endl; }
  if(logFile) {
    (*logFile) <<"{ newton: " <<its <<", evaluations: " <<evals <<", f_x: " <<fx <<", alpha: " <<alpha;
    if(o.verbose>3)(*logFile) <<", x: " <<x;
    (*logFile) <<" }," <<endl;
  }
  if(simpleLog) {
    (*simpleLog) <<its <<' ' <<evals <<' ' <<fx <<' ' <<alpha;
    if(x.N<=5) x.writeRaw(*simpleLog);
    (*simpleLog) <<endl;
  }
}

//===========================================================================

void boundClip(arr& y, const arr& bound_lo, const arr& bound_up) {
  if(bound_lo.N && bound_up.N) {
    for(uint i=0; i<y.N; i++) if(bound_up(i)>bound_lo(i)) {
        if(y(i)>bound_up(i)) y(i) = bound_up(i);
        if(y(i)<bound_lo(i)) y(i) = bound_lo(i);
      }
  }
}

OptNewton::StopCriterion OptNewton::step() {
  double fy;
  arr y, gy, Hy, Delta;

  its++;
  if(o.verbose>1) cout <<"optNewton it=" <<std::setw(4) <<its << " \tbeta=" <<std::setw(8) <<beta <<flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  rai::timerRead(true);

  //-- compute Delta
  arr R=Hx;
  if(beta) { //Levenberg Marquardt damping
    if(!isSpecial(R)) {
      for(uint i=0; i<R.d0; i++) R(i, i) += beta;
    } else if(isRowShifted(R)) {
      for(uint i=0; i<R.d0; i++) R(i, 0) += beta; //(R(i,0) is the diagonal in the packed matrix!!)
    } else if(isSparseMatrix(R)) {
      for(uint i=0; i<R.d0; i++) R.sparse().addEntry(i, i) = beta;
    } else NIY;
  }
  {
    bool inversionFailed=false;
    try {
      if(!rootFinding) {
        Delta = lapack_Ainv_b_sym(R, -gx);
      } else {
        lapack_mldivide(Delta, R, -gx);
      }
    } catch(...) {
      inversionFailed=true;
    }
    if(inversionFailed) {
      if(false) { //increase beta
        arr sig = lapack_kSmallestEigenValues_sym(R, 3);
        if(o.verbose>0) {
          cout <<"** hessian inversion failed ... increasing damping **\neigenvalues=" <<sig <<endl;
        }
        double sigmin = sig.min();
        if(sigmin>0.) THROW("Hessian inversion failed, but eigenvalues are positive???");
        beta = 2.*beta - sigmin;
        return stopCriterion=stopNone;
      } else { //use gradient
        if(o.verbose>0) {
          cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
        }
        Delta = gx * (-o.maxStep/length(gx));
      }
    }
  }

  //restrict stepsize
  double maxDelta = absMax(Delta);
  if(o.maxStep>0. && maxDelta>o.maxStep) {  Delta *= o.maxStep/maxDelta; maxDelta = o.maxStep; }
  double alphaHiLimit = o.maxStep/maxDelta;
  double alphaLoLimit = 1e-1*o.stopTolerance/maxDelta;

  if(o.verbose>1) cout <<" \t|Delta|=" <<std::setw(11) <<maxDelta <<flush;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*o.stopTolerance) {
    if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
    return stopCriterion=stopCrit1;
  }
  timeNewton += rai::timerRead(true);

  //-- line search along Delta
  uint lineSearchSteps=0;
  for(bool endLineSearch=false; !endLineSearch; lineSearchSteps++) {
    if(!o.allowOverstep) if(alpha>1.) alpha=1.;
    if(alphaHiLimit>0. && alpha>alphaHiLimit) alpha=alphaHiLimit;
    y = x + alpha*Delta;
    boundClip(y, bound_lo, bound_up);
    double timeBefore = rai::timerStart();
    fy = f(gy, Hy, y);  evals++;
    timeEval += rai::timerRead(true, timeBefore);
    if(o.verbose>5) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<flush;
    if(simpleLog) {
      (*simpleLog) <<its <<' ' <<evals <<' ' <<fy <<' ' <<alpha;
      if(y.N<=5) y.writeRaw(*simpleLog);
      (*simpleLog) <<endl;
    }

    bool wolfe = (fy <= fx + o.wolfe*scalarProduct(y-x, gx));
    if(rootFinding) wolfe=true;
    if(fy==fy && (wolfe || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)its)) { //fy==fy is for !NAN
      //accept new point
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: True }," <<endl;
      }
      if(fx-fy<o.stopFTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      Hx = Hy;
      if(wolfe) {
        if(alpha>.9 && beta>o.damping) {
          beta *= o.dampingDec;
          if(alpha>1.) alpha=1.;
          endLineSearch=true;
        }
        alpha *= o.stepInc;
      } else {
        //this is the nonStrict case... weird, but well
        if(alpha<.01 && o.dampingInc!=1.) {
          beta*=o.dampingInc;
          alpha*=o.dampingInc*o.dampingInc;
          endLineSearch=true;
          if(o.verbose>1) cout <<"(line search stopped)" <<endl;
        }
        alpha *= o.stepDec;
      }
      break;
    } else {
      //reject new point
      if(o.verbose>1) cout <<" - reject (lineSearch:" <<lineSearchSteps <<")" <<flush;
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: False }," <<endl;
      }
      if(evals>o.stopEvals) {
        if(o.verbose>1) cout <<" (evals>stopEvals)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(lineSearchSteps>10) {
        if(o.verbose>1) cout <<" (lineSearchSteps>10)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(alpha<.01 && o.dampingInc!=1.) {
        beta*=o.dampingInc;
        alpha*=o.dampingInc*o.dampingInc;
        endLineSearch=true;
        if(o.verbose>1) cout <<", stop & betaInc"<<endl;
      } else {
        if(o.verbose>1) cout <<"\n\t\t\t\t\t(line search)\t" <<flush;
      }
      alpha *= o.stepDec;
      if(alpha<alphaLoLimit) endLineSearch=true;
    }
  }

  if(logFile) {
    (*logFile) <<"{ newton: " <<its <<", evaluations: " <<evals <<", f_x: " <<fx <<", alpha: " <<alpha;
    if(o.verbose>2)(*logFile) <<", Delta: " <<Delta;
    (*logFile) <<" }," <<endl;
  }

  //stopping criteria

#define STOPIF(expr, code, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }

  STOPIF(absMax(Delta)<o.stopTolerance,, stopCrit1);
  STOPIF(numTinySteps>4, numTinySteps=0, stopTinySteps);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=o.stopEvals,, stopCritEvals);
  STOPIF(its>=o.stopIters,, stopCritEvals);

#undef STOPIF

  return stopCriterion=stopNone;
}

OptNewton::~OptNewton() {
  if(o.fmin_return) *o.fmin_return=fx;
#ifndef RAI_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(o.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
}

OptNewton::StopCriterion OptNewton::run(uint maxIt) {
  numTinySteps=0;
  for(uint i=0; i<maxIt; i++) {
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopCrit1) break;
  }
  return stopCriterion;
}
