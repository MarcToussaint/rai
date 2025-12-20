/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "m_Newton.h"

#include <iomanip>

//#define NewtonLazyLineSearchMode

namespace rai {

template<> const char* Enum<OptNewton::StopCriterion>::names []= {
  "None", "DeltaConverge", "TinyFSteps", "TinyXSteps", "CritEvals", "StepFailed", "LineSearchSteps", 0
};

bool sanityCheck=false; //true;
void updateBoundActive(intA& boundActive, const arr& x, const arr& bound_lo, const arr& bound_up);

//===========================================================================

OptNewton::OptNewton(arr& _x, ScalarFunction _f, std::shared_ptr<OptOptions> _opt):
  f(_f), opt(_opt), x(_x) {
  alpha = opt->stepInit;
  beta = opt->damping;
//  if(f) reinit(_x);
}

void OptNewton::reinit(const arr& _x) {
  if(&x!=&_x) x = _x;

//  boundClip(x, bounds_lo, bounds_up);
  boundCheck(x, bounds);
  timeEval -= cpuTime();
#ifdef NewtonLazyLineSearchMode
  fx = f(NoArr, NoArr, x);  evals++;
#else
  fx = f(gx, Hx, x);  evals++;
#endif
  if(!(fx==fx)) HALT("NAN!")
  timeEval += cpuTime();

  //startup verbose
  if(opt->verbose>0) cout <<"--newton-- initial point f(x):" <<fx <<" alpha:" <<alpha <<" beta:" <<beta <<endl;
  if(opt->verbose>3) { if(x.N<5) cout <<"x:" <<x <<endl; }
}

//===========================================================================

OptNewton::StopCriterion OptNewton::step() {
  if(!evals) reinit(x);

  double fy;
  arr y, gy, Hy, Delta;

#ifdef NewtonLazyLineSearchMode
  timeEval -= cpuTime();
  fx = f(gx, Hx, x);  //evals++;
  timeEval += cpuTime();
#endif

  inner_iters++;
  if(opt->verbose>1) cout <<"--newton-- it:" <<std::setw(4) <<inner_iters <<std::flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  timeNewton -= cpuTime();

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
        SparseMatrix& s = R.sparse();
        for(uint k=0; k<s.elems.d0; k++) {
          uint i = s.elems(k, 0);
          uint j = s.elems(k, 1);
          if(i!=j && (boundActive.elem(i) || boundActive.elem(j))) {
            s.Z.elem(k) = 0.;
          }
        }
      } else NIY;
      if(opt->verbose>5) cout <<"  boundActive:" <<boundActive;
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
      if(opt->verbose>0) {
        cout <<"** hessian inversion failed ... using gradient descent direction" <<endl;
      }
      Delta = gx * (-opt->stepMax/length(gx));
    }
  }

  //-- restrict stepsize
  double maxDelta = absMax(Delta);
  if(opt->stepMax>0. && maxDelta>opt->stepMax) {
    Delta *= opt->stepMax/maxDelta;
    maxDelta = opt->stepMax;
  }
  double alphaHiLimit = opt->stepMax/maxDelta;
  //double alphaLoLimit = 1e-1*options.stopTolerance/maxDelta;

  if(opt->verbose>1) cout <<"  |Delta|:" <<std::setw(11) <<maxDelta;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*opt->stopTolerance) {
    if(opt->verbose>1) cout <<" \t -- absMax(Delta)<1e-1*o.stopTolerance -- NO UPDATE" <<endl;
    return stopCriterion=stopDeltaConverge;
  }

  timeNewton += cpuTime();

  //-- line search along Delta
  uint lineSearchSteps=0;
  for(;; lineSearchSteps++) {
    if(alpha>1.) alpha=1.;
    // if(alphaHiLimit>0. && alpha>alphaHiLimit) alpha=alphaHiLimit; //TODO: is this really right? seems to cap it twice?
    if(opt->verbose>1) cout <<"  alpha:" <<std::setw(11) <<alpha <<std::flush;
    y = x + alpha*Delta;
    if(opt->verbose>5) cout <<"  y:" <<y;
    boundClip(y, bounds);
    timeEval -= cpuTime();
#ifdef NewtonLazyLineSearchMode
    fy = f(NoArr, NoArr, y);  evals++;
#else
    fy = f(gy, Hy, y);  evals++;
#endif
    timeEval += cpuTime();
    if(opt->verbose>1) cout <<"  evals:" <<std::setw(4) <<evals <<"  f(y):" <<std::setw(11) <<fy <<std::flush;

    bool wolfe = (fy <= fx + opt->wolfe*scalarProduct(y-x, gx));
    if(rootFinding) wolfe=true;
    if(fy==fy && wolfe) { //fy==fy is for !NAN
      //== accept new point
      if(opt->verbose>1) cout <<"  ACCEPT" <<endl;
      double df = fx - fy;
      if(opt->stopFTolerance>0. && fx-fy<opt->stopFTolerance) numTinyFSteps++; else numTinyFSteps=0;
      if(absMax(y-x)<1e-2*opt->stopTolerance) numTinyXSteps++; else numTinyXSteps=0;
      x = y;
      fx = fy;
#ifdef NewtonLazyLineSearchMode
#else
      gx = gy;
      Hx = Hy;
#endif
      alpha *= opt->stepInc;
      break; //end line search
    } else {
      //== reject new point
      if(opt->verbose>1) cout <<"  reject (lineSearch:" <<lineSearchSteps <<")";
      if(evals>opt->stopEvals) {
        if(opt->verbose>1) cout <<" (evals>stopEvals)" <<endl;
        numTinyXSteps++;
        break; //end line search
      }
      if(lineSearchSteps>10) {
        if(opt->verbose>1) cout <<" (lineSearchSteps>10)" <<endl;
        numTinyXSteps++;
        break; //end line search
      }
      if(opt->verbose>1) cout <<"\n                    (line search)      ";
      alpha *= opt->stepDec;
//      if(alpha<alphaLoLimit) endLineSearch=true;
    }
  }

  //stopping criteria

#define STOPIF(expr, code, ret) if(expr){ if(opt->verbose>1) cout <<"--newton-- stopping: '" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }

  STOPIF(absMax(Delta)<opt->stopTolerance,, stopDeltaConverge);
  STOPIF(numTinyFSteps>4, numTinyFSteps=0, stopTinyFSteps);
  STOPIF(numTinyXSteps>4, numTinyXSteps=0, stopTinyXSteps);
//  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=opt->stopEvals,, stopCritEvals);
  STOPIF(inner_iters>=opt->stopInners,, stopCritEvals);
  STOPIF(lineSearchSteps>10,, stopLineSearchSteps);

#undef STOPIF

  return stopCriterion=stopNone;
}

OptNewton::~OptNewton() {
#ifndef RAI_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(opt->verbose>0) cout <<"--newton done-- final f(x):" <<fx <<endl;
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

shared_ptr<SolverReturn> OptNewton::run(uint maxIt) {
  numTinyFSteps=numTinyXSteps=0;
  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  for(uint i=0; i<maxIt; i++) {
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopDeltaConverge) break;
  }
  ret->evals = evals;
  ret->f = fx;
  ret->feasible = true;
  return ret;
}

} //namespace
