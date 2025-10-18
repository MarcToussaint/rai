/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "gradient.h"
#include <iomanip>
#include <math.h>

namespace rai {

// function evaluation counter (used only for performance meassurements, global for simplicity)
uint eval_count=0;

//===========================================================================

OptGrad::OptGrad(arr& _x, ScalarFunction& _f, shared_ptr<OptOptions> _opt):
  x(_x), f(_f), opt(_opt), it(0), evals(0), numTinySteps(0) {
  alpha = opt->stepInit;
//  if(f) reinit();
}

void OptGrad::reinit(const arr& _x) {
  if(!!_x && &_x!=&x) x=_x;
  fx = f.f(gx, NoArr, x);  evals++;

  //startup verbose
  if(opt->verbose>1) cout <<"*** optGrad: starting point f(x)=" <<fx <<" alpha=" <<alpha <<endl;
  if(opt->verbose>2) cout <<"             x=" <<x <<endl;
  if(opt->verbose>0) fil.open("z.opt");
  if(opt->verbose>0) { fil <<0 <<' ' <<eval_count <<' ' <<fx <<' ' <<alpha;  if(x.N<=5) fil <<x.modRaw();  fil <<endl; }
}

OptGrad::StopCriterion OptGrad::step() {
  double fy;
  arr y, gy, Delta;

  if(!evals) reinit();

  it++;
  if(opt->verbose>1) cout <<"--grad-- it=" <<std::setw(4) <<it <<std::flush;

  if(!(fx==fx)) HALT("you're calling a gradient step with initial function value = NAN");

  //compute Delta
  Delta = gx / (-length(gx));

  //line search
  uint lineSteps=0;
  for(;; lineSteps++) {
    y = x + alpha*Delta;
    fy = f.f(gy, NoArr, y);  evals++;
    if(opt->verbose>2) cout <<" \tprobing y=" <<y;
    if(opt->verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<std::flush;
    bool wolfe = (opt->wolfe<=0. || fy <= fx + opt->wolfe*alpha*scalarProduct(Delta, gx));
    if(fy==fy && wolfe) { //fy==fy is for NAN?
      //accept new point
      if(opt->verbose>1) cout <<" - ACCEPT" <<endl;
      if(fx-fy<opt->stopFTolerance || alpha<opt->stopTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      alpha *= opt->stepInc;
      break;
    } else {
      //reject new point
      if(opt->verbose>1) cout <<" - reject" <<std::flush;
      if(opt->stopLineSteps>0 && lineSteps>(uint)opt->stopLineSteps) break;
      if(opt->stopEvals>0 && evals>(uint)opt->stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      if(opt->verbose>1) cout <<"\n  (line search)" <<std::flush;
      alpha *= opt->stepDec;
    }
  }

  if(opt->verbose>0) { fil <<evals <<' ' <<eval_count <<' ' <<fx <<' ' <<alpha;  if(x.N<=5) fil <<x.modRaw();  fil <<endl; }

  //stopping criteria
#define STOPIF(expr, code, ret) if(expr){ if(opt->verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }
  //  STOPIF(absMax(Delta)<o.stopTolerance, , stopCrit1);
  STOPIF(numTinySteps>(uint)opt->stopTinySteps, numTinySteps=0, stopCrit2);
  //  STOPIF(alpha<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(lineSteps>=(uint)opt->stopLineSteps,, stopCritLineSteps);
  STOPIF(evals>=(uint)opt->stopEvals,, stopCritEvals);
  STOPIF(it>=(uint)opt->stopInners,, stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}

OptGrad::~OptGrad() {
  if(opt->verbose>0) fil.close();
#ifndef RAI_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(opt->verbose>1) cout <<"--- OptGradStop: f(x)=" <<fx <<endl;
}

OptGrad::StopCriterion OptGrad::run(uint maxIt) {
  numTinySteps=0;
  for(uint i=0; i<maxIt; i++) {
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion==stopCritLineSteps) { reinit();   continue; }
    if(stopCriterion>=stopCrit1) break;
  }
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, false);
  return stopCriterion;
}

//===========================================================================
//
// Rprop
//

int _sgn(double x) { if(x > 0) return 1; if(x < 0) return -1; return 0; }
double _mymin(double x, double y) { return x < y ? x : y; }
double _mymax(double x, double y) { return x > y ? x : y; }

struct sRprop {
  double incr;
  double decr;
  double dMax;
  double dMin;
  double rMax;
  double delta0;
  arr lastGrad; // last error gradient
  arr stepSize; // last update
  bool step(arr& w, const arr& grad, uint* singleI);
};

Rprop::Rprop() {
  self = std::make_unique<sRprop>();
  self->incr   = 1.2;
  self->decr   = .33;
  self->dMax = 50.;
  self->dMin = 1e-6;
  self->rMax = 0.;
  self->delta0 = 1.;
}

Rprop::~Rprop() {
}

void Rprop::init(double initialStepSize, double minStepSize, double stepMaxSize) {
  self->stepSize.resize(0);
  self->lastGrad.resize(0);
  self->delta0 = initialStepSize;
  self->dMin = minStepSize;
  self->dMax = stepMaxSize;
}

bool sRprop::step(arr& w, const arr& grad, uint* singleI) {
  if(!stepSize.N) { //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK_EQ(grad.N, stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK_EQ(w.N, stepSize.N, "Rprop: parameter dimensionality changed!");

  uint i=0, I=w.N;
  if(singleI) { i=*(singleI); I=i+1; }
  for(; i<I; i++) {
    if(grad.elem(i) * lastGrad(i) > 0) { //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    } else if(grad.elem(i) * lastGrad(i) < 0) { //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction (undo half the step)
      lastGrad(i) = 0;                               //memorize to continue below next time
    } else {                              //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }

  return max(stepSize) < incr*dMin;
}

bool Rprop::step(arr& x, ScalarFunction& f) {
  arr grad;
  f.f(grad, NoArr, x);
  return self->step(x, grad, nullptr);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 ScalarFunction& f,
                 double stoppingTolerance,
                 double initialStepSize,
                 uint maxEvals,
                 int verbose) {

  if(!self->stepSize.N) init(initialStepSize);
  arr x, J(_x.N), x_min, J_min;
  double fx_min=0;
  uint rejects=0, small_steps=0;
  x=_x;

  if(verbose>1) cout <<"*** optRprop: starting point x=" <<x <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.opt");

  evals=0;
  double diff=0.;
  for(;;) {
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    fx = f.f(J, NoArr, x);  evals++;

    if(verbose>0) { fil <<evals <<' ' <<eval_count <<' ' << fx <<' ' <<diff <<' ' <<x.modRaw() <<endl; }
    if(verbose>1) cout <<"--rprop-- " <<evals <<' ' <<eval_count <<" \tf(x)=" <<fx <<" \tdiff=" <<diff <<" \tx=" <<(x.N<20?x:arr()) <<endl;

    //infeasible point! undo the previous step
    if(fx!=fx) { //is NAN
      if(!evals) HALT("can't start Rprop with unfeasible point");
      self->stepSize*=(double).1;
      self->lastGrad=(double)0.;
      x=x_min;
      fx=fx_min;
      J=J_min;
      rejects=0;
    }

    //update best-so-far
    if(evals<=1) { fx_min= fx; x_min=x; }
    if(fx<=fx_min) {
      x_min=x;
      fx_min=fx;
      J_min=J;
      rejects=0;
    } else {
      rejects++;
      if(rejects>10) {
        self->stepSize*=(double).1;
        self->lastGrad=(double)0.;
        x=x_min;
        fx=fx_min;
        J=J_min;
        rejects=0;
      }
    }

    //update x
    self->step(x, J, nullptr);

    //check stopping criterion based on step-length in x
    diff=maxDiff(x, x_min);

    if(diff<stoppingTolerance) { small_steps++; } else { small_steps=0; }
    if(small_steps>3)  break;
    if(evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
//  if(verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
  _x=x_min;
  fx = fx_min;
  return evals;
}

} //namespace
