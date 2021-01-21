/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "newton.h"
#include <iomanip>

bool sanityCheck=false; //true;
void updateBoundActive(intA& boundActive, const arr& x, const arr& bound_lo, const arr& bound_up);
void boundClip(arr& y, const arr& bound_lo, const arr& bound_up);

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
  x(_x), f(_f), options(_o), logFile(_logFile) {
  alpha = options.initStep;
  beta = options.damping;
  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x) {
  if(&x!=&_x) x = _x;

  boundClip(x, bounds_lo, bounds_up);
  fx = f(gx, Hx, x);  evals++;

  //startup verbose
  if(options.verbose>1) cout <<"*** optNewton: initial point f(x)=" <<fx <<" alpha=" <<alpha <<" beta=" <<beta <<endl;
  if(options.verbose>3){ if(x.N<5) cout <<"x=" <<x <<endl; }
  if(logFile) {
    (*logFile) <<"{ newton: " <<its <<", evaluations: " <<evals <<", f_x: " <<fx <<", alpha: " <<alpha;
    if(options.verbose>3)(*logFile) <<", x: " <<x;
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

//===========================================================================

OptNewton::StopCriterion OptNewton::step() {
  double fy;
  arr y, gy, Hy, Delta;

  its++;
  if(options.verbose>1) cout <<"optNewton it:" <<std::setw(4) <<its << "  beta:" <<std::setw(4) <<beta <<flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  rai::timerRead(true);

  //-- check active bounds, and decorrelate Hessian
  arr R=Hx;
#if 1
  {
    intA boundActive; //analogy to dual parameters for bounds: -1: lower active; +1: upper active
    uint nActiveBounds=0;
    if(!boundActive.N) boundActive.resize(x.N).setZero();
#define BOUND_EPS 1e-10
    if(bounds_lo.N && bounds_up.N) {
      for(uint i=0; i<x.N; i++) if(bounds_up(i)>bounds_lo(i)) {
        if(x(i)>=bounds_up(i)-BOUND_EPS){ boundActive(i) = +1; nActiveBounds++; }
        else if(x(i)<=bounds_lo(i)+BOUND_EPS){ boundActive(i) = -1; nActiveBounds++; }
        else boundActive(i) = 0;
      }
    }
#undef BOUND_EPS
    if(nActiveBounds){
      //zero correlations to bound-active variables
      if(!isSpecial(R)) {
        for(uint i=0;i<x.N;i++) if(boundActive.elem(i)){
          for(uint j=0;j<x.N;j++) if(i!=j){ R(i,j)=0; R(j,i)=0; }
        }
      } else if(R.isSparse()) {
        rai::SparseMatrix& s = R.sparse();
        for(uint k=0; k<s.elems.d0; k++) {
          uint i = s.elems(k, 0);
          uint j = s.elems(k, 1);
          if(i!=j && (boundActive.elem(i) || boundActive.elem(j))){
            s.Z.elem(k) = 0.;
          }
        }
      } else NIY;
      if(options.verbose>5) cout <<"  boundActive:" <<boundActive;
    }
  }
#endif

  //-- compute Delta
#if 0
  arr sig = lapack_kSmallestEigenValues_sym(R, 3);
  double sigmin = sig.min();
  double diag = 0.;
  if(sigmin<beta) diag = beta-sigmin;
#endif
  if(beta) { //Levenberg Marquardt damping
    if(!isSpecial(R)) {
      for(uint i=0; i<R.d0; i++) R(i, i) += beta;
    } else if(isRowShifted(R)) {
      for(uint i=0; i<R.d0; i++) R.rowShifted().entry(i, 0) += beta; //(R(i,0) is the diagonal in the packed matrix!!)
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
    if(!inversionFailed && scalarProduct(Delta,gx)>0.){
      inversionFailed = true;
    }
    if(inversionFailed) {
#if 0 //increase beta to min eig value and repeat
      arr sig = lapack_kSmallestEigenValues_sym(R, 3);
      if(o.verbose>0) {
        cout <<"** hessian inversion failed ... increasing damping **\neigenvalues=" <<sig <<endl;
      }
      double sigmin = sig.min();
      if(sigmin>0.) THROW("Hessian inversion failed, but eigenvalues are positive???");
      beta = 2.*beta - sigmin;
      return stopCriterion=stopNone;
#endif
#if 1 //increase beta by betaInc
      if(options.dampingInc!=1.) beta*=options.dampingInc;
#endif
      //use gradient
      if(options.verbose>0) {
        cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
      }
      Delta = gx * (-options.maxStep/length(gx));
    } else { //inversion successful
      //decrease beta by betaDec
      if(options.dampingDec!=1.) beta *= options.dampingDec;
      if(beta<options.damping) beta=options.damping;
    }
  }

  //restrict stepsize
  double maxDelta = absMax(Delta);
  if(options.maxStep>0. && maxDelta>options.maxStep) {  Delta *= options.maxStep/maxDelta; maxDelta = options.maxStep; }
  double alphaHiLimit = options.maxStep/maxDelta;
  double alphaLoLimit = 1e-1*options.stopTolerance/maxDelta;

  if(options.verbose>1) cout <<"  |Delta|:" <<std::setw(11) <<maxDelta <<flush;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*options.stopTolerance) {
    if(options.verbose>1) cout <<" \t -- absMax(Delta)<1e-1*o.stopTolerance -- NO UPDATE" <<endl;
    return stopCriterion=stopDeltaConverge;
  }
  timeNewton += rai::timerRead(true);

  //-- line search along Delta
  uint lineSearchSteps=0;
  for(bool endLineSearch=false; !endLineSearch; lineSearchSteps++) {
    if(!options.allowOverstep) if(alpha>1.) alpha=1.;
    if(alphaHiLimit>0. && alpha>alphaHiLimit) alpha=alphaHiLimit;
    y = x + alpha*Delta;
    boundClip(y, bounds_lo, bounds_up);
    double timeBefore = rai::timerStart();
    fy = f(gy, Hy, y);  evals++;
    timeEval += rai::timerRead(true, timeBefore);
    if(options.verbose>5) cout <<"  probing y:" <<y;
    if(options.verbose>1) cout <<"  evals:" <<std::setw(4) <<evals <<"  alpha:" <<std::setw(11) <<alpha <<"  f(y):" <<fy <<flush;
    if(simpleLog) {
      (*simpleLog) <<its <<' ' <<evals <<' ' <<fy <<' ' <<alpha;
      if(y.N<=5) y.writeRaw(*simpleLog);
      (*simpleLog) <<endl;
    }

    bool wolfe = (fy <= fx + options.wolfe*scalarProduct(y-x, gx));
    if(rootFinding) wolfe=true;
    if(fy==fy && (wolfe || options.nonStrictSteps==-1 || options.nonStrictSteps>(int)its)) { //fy==fy is for !NAN
      //accept new point
      if(options.verbose>1) cout <<" - ACCEPT" <<endl;
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: True }," <<endl;
      }
      if(options.stopFTolerance<0. && fx-fy<options.stopFTolerance) numTinyFSteps++; else numTinyFSteps=0;
      if(absMax(y-x)<1e-1*options.stopTolerance) numTinyXSteps++; else numTinyXSteps=0;
      x = y;
      fx = fy;
      gx = gy;
      Hx = Hy;
      if(wolfe) {
        if(alpha>.9 && beta>options.damping) {
          beta *= options.dampingDec;
          if(alpha>1.) alpha=1.;
          endLineSearch=true;
        }
        alpha *= options.stepInc;
      } else {
        //this is the nonStrict case... weird, but well
        if(alpha<.01 && options.dampingInc!=1.) {
          beta*=options.dampingInc;
          alpha*=options.dampingInc*options.dampingInc;
          endLineSearch=true;
          if(options.verbose>1) cout <<"(line search stopped)" <<endl;
        }
        alpha *= options.stepDec;
      }
      break;
    } else {
      //reject new point
      if(options.verbose>1) cout <<" - reject (lineSearch:" <<lineSearchSteps <<")" <<flush;
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: False }," <<endl;
      }
      if(evals>options.stopEvals) {
        if(options.verbose>1) cout <<" (evals>stopEvals)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(lineSearchSteps>10) {
        if(options.verbose>1) cout <<" (lineSearchSteps>10)" <<endl;
        break; //WARNING: this may lead to non-monotonicity -> make evals high!
      }
      if(alpha<.01 && options.dampingInc!=1.) {
        beta*=options.dampingInc;
        alpha*=options.dampingInc*options.dampingInc;
        endLineSearch=true;
        if(options.verbose>1) cout <<", stop & betaInc"<<endl;
      } else {
        if(options.verbose>1) cout <<"\n                                  (line search)  " <<flush;
      }
      alpha *= options.stepDec;
//      if(alpha<alphaLoLimit) endLineSearch=true;
    }
  }

  if(logFile) {
    (*logFile) <<"{ newton: " <<its <<", evaluations: " <<evals <<", f_x: " <<fx <<", alpha: " <<alpha;
    if(options.verbose>2)(*logFile) <<", Delta: " <<Delta;
    (*logFile) <<" }," <<endl;
  }

  //stopping criteria

#define STOPIF(expr, code, ret) if(expr){ if(options.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }

  STOPIF(absMax(Delta)<options.stopTolerance,, stopDeltaConverge);
  STOPIF(numTinyFSteps>4, numTinyFSteps=0, stopTinyFSteps);
  STOPIF(numTinyXSteps>4, numTinyXSteps=0, stopTinyXSteps);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=options.stopEvals,, stopCritEvals);
  STOPIF(its>=options.stopIters,, stopCritEvals);

#undef STOPIF

  return stopCriterion=stopNone;
}

OptNewton::~OptNewton() {
  if(options.fmin_return) *options.fmin_return=fx;
#ifndef RAI_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(options.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
}

OptNewton::StopCriterion OptNewton::run(uint maxIt) {
  numTinyFSteps=numTinyXSteps=0;
  for(uint i=0; i<maxIt; i++) {
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopDeltaConverge) break;
  }
  return stopCriterion;
}
