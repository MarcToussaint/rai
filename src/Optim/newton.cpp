/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "newton.h"

#include <iomanip>

//#define NewtonLazyLineSearchMode

template<> const char* rai::Enum<OptNewton::StopCriterion>::names []= {
  "None", "DeltaConverge", "TinyFSteps", "TinyXSteps", "CritEvals", "StepFailed", "LineSearchSteps", 0
};

bool sanityCheck=false; //true;
void updateBoundActive(intA& boundActive, const arr& x, const arr& bound_lo, const arr& bound_up);

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
int optNewton(arr& x, const ScalarFunction& f, rai::OptOptions o) {
  OptNewton opt(x, f, o);
  ofstream fil("z.opt");
  opt.simpleLog = &fil;
  return opt.run();
}

//===========================================================================

OptNewton::OptNewton(arr& _x, const ScalarFunction& _f, rai::OptOptions _o, ostream* _logFile):
  x(_x), f(_f), options(_o), logFile(_logFile) {
  alpha = options.initStep;
  beta = options.damping;
//  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x) {
  if(&x!=&_x) x = _x;

//  boundClip(x, bounds_lo, bounds_up);
  boundCheck(x, bounds);
  timeEval -= rai::cpuTime();
#ifdef NewtonLazyLineSearchMode
  fx = f(NoArr, NoArr, x);  evals++;
#else
  fx = f(gx, Hx, x);  evals++;
#endif
  timeEval += rai::cpuTime();

  //startup verbose
  if(options.verbose>1) cout <<"----newton---- initial point f(x):" <<fx <<" alpha:" <<alpha <<" beta:" <<beta <<endl;
  if(options.verbose>3) { if(x.N<5) cout <<"x:" <<x <<endl; }
  if(logFile) {
    (*logFile) <<"{ newton: " <<its <<", evaluations: " <<evals <<", f_x: " <<fx <<", alpha: " <<alpha;
    if(options.verbose>3)(*logFile) <<", x: " <<x;
    (*logFile) <<" }," <<endl;
  }
  if(simpleLog) {
    (*simpleLog) <<its <<' ' <<evals <<' ' <<fx <<' ' <<alpha;
    if(x.N<=5)(*simpleLog) <<x.modRaw();
    (*simpleLog) <<endl;
  }
}

//===========================================================================

OptNewton::StopCriterion OptNewton::step() {
  if(!evals) reinit(x);

  double fy;
  arr y, gy, Hy, Delta;

#ifdef NewtonLazyLineSearchMode
  timeEval -= rai::cpuTime();
  fx = f(gx, Hx, x);  //evals++;
  timeEval += rai::cpuTime();
#endif

  its++;
  if(options.verbose>1) cout <<"--newton-- it:" <<std::setw(4) <<its <<std::flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  timeNewton -= rai::cpuTime();

  //-- check active bounds, and decorrelate Hessian
  arr R=Hx;
#if 1
  {
    intA boundActive; //analogy to dual parameters for bounds: -1: lower active; +1: upper active
    uint nActiveBounds=0;
    if(!boundActive.N) boundActive.resize(x.N).setZero();
#define BOUND_EPS 1e-10
    if(bounds.N) {
      for(uint i=0; i<x.N; i++) if(bounds(1,i)>bounds(0,i)) {
          if(x(i)>=bounds(1,i)-BOUND_EPS) { boundActive(i) = +1; nActiveBounds++; }
          else if(x(i)<=bounds(0,i)+BOUND_EPS) { boundActive(i) = -1; nActiveBounds++; }
          else boundActive(i) = 0;
        }
    }
#undef BOUND_EPS
    if(nActiveBounds) {
      //zero correlations to bound-active variables
      if(!isSpecial(R)) {
        for(uint i=0; i<x.N; i++) if(boundActive.elem(i)) {
            for(uint j=0; j<x.N; j++) if(i!=j) { R(i, j)=0; R(j, i)=0; }
          }
      } else if(isSparse(R)) {
        rai::SparseMatrix& s = R.sparse();
        for(uint k=0; k<s.elems.d0; k++) {
          uint i = s.elems(k, 0);
          uint j = s.elems(k, 1);
          if(i!=j && (boundActive.elem(i) || boundActive.elem(j))) {
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
  double sigmin = min(sig);
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
    if(!inversionFailed && scalarProduct(Delta, gx)>0.) {
      inversionFailed = true;
    }
    if(inversionFailed) {
#if 0 //increase beta to min eig value and repeat
      arr sig = lapack_kSmallestEigenValues_sym(R, 3);
      if(o.verbose>0) {
        cout <<"** hessian inversion failed ... increasing damping **\neigenvalues:" <<sig <<endl;
      }
      double sigmin = min(sig);
      if(sigmin>0.) THROW("Hessian inversion failed, but eigenvalues are positive???");
      beta = 2.*beta - sigmin;
      return stopCriterion=stopNone;
#endif
      //use gradient
      if(options.verbose>0) {
        cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
      }
      Delta = gx * (-options.maxStep/length(gx));
    }
  }

  //restrict stepsize
  double maxDelta = absMax(Delta);
  if(options.maxStep>0. && maxDelta>options.maxStep) {
    Delta *= options.maxStep/maxDelta;
    maxDelta = options.maxStep;
  }
  double alphaHiLimit = options.maxStep/maxDelta;
  //double alphaLoLimit = 1e-1*options.stopTolerance/maxDelta;

  if(options.verbose>1) cout <<"  |Delta|:" <<std::setw(11) <<maxDelta;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*options.stopTolerance) {
    if(options.verbose>1) cout <<" \t -- absMax(Delta)<1e-1*o.stopTolerance -- NO UPDATE" <<endl;
    return stopCriterion=stopDeltaConverge;
  }

  timeNewton += rai::cpuTime();

  //-- line search along Delta
  uint lineSearchSteps=0;
  for(;; lineSearchSteps++) {
    if(alpha>1.) alpha=1.;
    if(alphaHiLimit>0. && alpha>alphaHiLimit) alpha=alphaHiLimit;
    if(options.verbose>1) cout <<"  alpha:" <<std::setw(11) <<alpha <<std::flush;
    y = x + alpha*Delta;
    if(options.verbose>5) cout <<"  y:" <<y;
    boundClip(y, bounds);
    timeEval -= rai::cpuTime();
#ifdef NewtonLazyLineSearchMode
    fy = f(NoArr, NoArr, y);  evals++;
#else
    fy = f(gy, Hy, y);  evals++;
#endif
    timeEval += rai::cpuTime();
    if(options.verbose>1) cout <<"  evals:" <<std::setw(4) <<evals <<"  f(y):" <<std::setw(11) <<fy <<std::flush;
    if(simpleLog) {
      (*simpleLog) <<its <<' ' <<evals <<' ' <<fy <<' ' <<alpha;
      if(y.N<=5)(*simpleLog) <<y.modRaw();
      (*simpleLog) <<endl;
    }

    bool wolfe = (fy <= fx + options.wolfe*scalarProduct(y-x, gx));
    if(rootFinding) wolfe=true;
    if(fy==fy && wolfe) { //fy==fy is for !NAN
      //== accept new point
      if(options.verbose>1) cout <<"  ACCEPT" <<endl;
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: True }," <<endl;
      }
      if(options.stopFTolerance<0. && fx-fy<options.stopFTolerance) numTinyFSteps++; else numTinyFSteps=0;
      if(absMax(y-x)<1e-2*options.stopTolerance) numTinyXSteps++; else numTinyXSteps=0;
      x = y;
      fx = fy;
#ifdef NewtonLazyLineSearchMode
#else
      gx = gy;
      Hx = Hy;
#endif
      alpha *= options.stepInc;
      break; //end line search
    } else {
      //== reject new point
      if(options.verbose>1) cout <<"  reject (lineSearch:" <<lineSearchSteps <<")";
      if(logFile) {
        (*logFile) <<"{ lineSearch: " <<lineSearchSteps <<", alpha: " <<alpha <<", beta: " <<beta <<", f_x: " <<fx <<", f_y: " <<fy <<", wolfe: " <<wolfe <<", accept: False }," <<endl;
      }
      if(evals>options.stopEvals) {
        if(options.verbose>1) cout <<" (evals>stopEvals)" <<endl;
        numTinyXSteps++;
        break; //end line search
      }
      if(lineSearchSteps>10) {
        if(options.verbose>1) cout <<" (lineSearchSteps>10)" <<endl;
        numTinyXSteps++;
        break; //end line search
      }
      if(options.verbose>1) cout <<"\n                    (line search)      ";
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

#define STOPIF(expr, code, ret) if(expr){ if(options.verbose>1) cout <<"--newton-- stopping: '" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }

  STOPIF(absMax(Delta)<options.stopTolerance,, stopDeltaConverge);
  STOPIF(numTinyFSteps>4, numTinyFSteps=0, stopTinyFSteps);
  STOPIF(numTinyXSteps>4, numTinyXSteps=0, stopTinyXSteps);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=options.stopEvals,, stopCritEvals);
  STOPIF(its>=options.stopInners,, stopCritEvals);
  STOPIF(lineSearchSteps>10,, stopLineSearchSteps);

#undef STOPIF

  return stopCriterion=stopNone;
}

OptNewton::~OptNewton() {
#ifndef RAI_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(options.verbose>1) cout <<"----newton---- final f(x):" <<fx <<endl;
}

OptNewton& OptNewton::setBounds(const arr& _bounds) {
  bounds = _bounds;
  if(x.N) {
    CHECK_EQ(bounds.nd, 2, "");
    CHECK_EQ(bounds.d1, x.N, "");
    bool good = boundCheck(x, bounds);
    if(!good) HALT("seed x is not within bounds")
//    boundClip(x, bounds_lo, bounds_up);
//    reinit(x);
    }
  return *this;
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
