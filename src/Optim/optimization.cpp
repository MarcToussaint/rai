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

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif

bool sanityCheck=false; //true;
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

/// numeric (finite difference) check of the gradient of f at x
bool checkGradient(ScalarFunction &f,
                   const arr& x, double tolerance) {
  arr J, dx, JJ;
  double y, dy;
  y=f.fs(J, NoArr, x);
  
  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f.fs(NoArr, NoArr, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance) {
    MT_MSG("checkGradient -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkHessian(ScalarFunction &f, const arr& x, double tolerance) {
  arr g, H, dx, dy, Jg;
  f.fs(g, H, x);
  if(H.special==arr::RowShiftedPackedMatrixST) H = unpack(H);
  
  Jg.resize(g.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f.fs(dy, NoArr, dx);
    dy = (dy-g)/eps;
    for(k=0; k<g.N; k++) Jg(k, i)=dy.elem(k);
  }
  Jg.reshapeAs(H);
  double md=maxDiff(H, Jg, &i);
  //   MT::save(J, "z.J");
  //   MT::save(JJ, "z.JJ");
  if(md>tolerance) {
    MT_MSG("checkHessian -- FAILURE -- max diff=" <<md <<" |"<<H.elem(i)<<'-'<<Jg.elem(i)<<"| (stored in files z.J_*)");
    MT::save(H, "z.J_analytical");
    MT::save(Jg, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkHessian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkJacobian(VectorFunction &f,
                   const arr& x, double tolerance) {
  arr y, J, dx, dy, JJ;
  f.fv(y, J, x);
  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);
  
  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f.fv(dy, NoArr, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance) {
    MT_MSG("checkJacobian -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkJacobian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkAll(ConstrainedProblem &P, const arr& x, double tolerance){
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
  useAdaptiveDamping=false;
  clampInitialState=false;
}

OptOptions global_optOptions;

/// minimizes \f$f(x)\f$ using its gradient only
uint optRprop(arr& x, ScalarFunction& f, OptOptions o) {
  return Rprop().loop(x, f, o.fmin_return, o.stopTolerance, o.initStep, o.stopEvals, o.verbose);
}

/** @brief Minimizes \f$f(x) = \phi(x)^T \phi(x)$ using the Jacobian. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
uint optGaussNewton(arr& x, VectorFunction& f, OptOptions o, arr *addRegularizer, arr *fx_user, arr *Jx_user) {
  double alpha = 1.;
  double lambda = o.damping;
  double fx, fy;
  arr Delta, y;
  uint evals=0;
  
  if(fx_user) NIY;
  
  //compute initial costs
  arr phi;
  arr J;
  f.fv(phi, J, x);  evals++;
  fx = sumOfSqr(phi);
  if(addRegularizer)  fx += scalarProduct(x,(*addRegularizer)*vectorShaped(x));

  //startup verbose
  if(o.verbose>1) cout <<"*** optGaussNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" lambda=" <<lambda <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha <<' ' <<x <<endl;

  for(uint it=1;; it++) { //iterations and lambda adaptation loop
    if(o.verbose>1) cout <<"optGaussNewton it=" <<it << " lambda=" <<lambda <<flush;

    //compute Delta
#if 1
    arr R=comp_At_A(J);
    if(lambda) { //Levenberg Marquardt damping
      if(R.special==arr::RowShiftedPackedMatrixST) for(uint i=0; i<R.d0; i++) R(i,0) += .5*lambda; //(R(i,0) is the diagonal in the packed matrix!!)
      else for(uint i=0; i<R.d0; i++) R(i,i) += .5*lambda;
    }
    if(addRegularizer) {
      if(R.special==arr::RowShiftedPackedMatrixST) R = unpack(R);
//      cout <<*addRegularizer <<R <<endl;
      lapack_Ainv_b_sym(Delta, R + (*addRegularizer), -(comp_At_x(J, phi)+(*addRegularizer)*vectorShaped(x)));
    } else {
      lapack_Ainv_b_sym(Delta, R, -comp_At_x(J, phi));
    }
#else //this uses lapack's LLS minimizer - but is really slow!!
    x.reshape(x.N);
    if(lambda) {
      arr D; D.setDiag(sqrt(.5*lambda),x.N);
      J.append(D);
      phi.append(zeros(x.N,1));
    }
    lapack_min_Ax_b(Delta, J, J*x - phi);
    Delta -= x;
#endif
    if(o.maxStep>0. && absMax(Delta)>o.maxStep)  Delta *= o.maxStep/absMax(Delta);
    if(o.verbose>1) cout <<" \t|Delta|=" <<absMax(Delta) <<flush;

    //lazy stopping criterion: stop without any update
    if(lambda<2. && absMax(Delta)<1e-3*o.stopTolerance) break;

    for(;;) { //stepsize adaptation loop -- doesn't iterate for useDamping option
      y = x + alpha*Delta;
      f.fv(phi, J, y);  evals++;
      fy = sumOfSqr(phi);
      if(addRegularizer) fy += scalarProduct(y,(*addRegularizer)*vectorShaped(y));
      if(o.verbose>2) cout <<" \tprobing y=" <<y;
      if(o.verbose>1) cout <<" \talpha=" <<alpha <<" \tevals=" <<evals <<" \tf(y)=" <<fy <<flush;
      CHECK(fy==fy, "cost seems to be NAN: ly=" <<fy);
      if(fy!=NAN && fy <= fx) {
        if(o.verbose>1) cout <<" - ACCEPT" <<endl;
        //adopt new point and adapt stepsize|damping
        x = y;
        fx = fy;
        if(o.useAdaptiveDamping) { //Levenberg-Marquardt type damping
          lambda = .2*lambda;
        } else {
          alpha = pow(alpha, 0.5);
        }
        break;
      } else {
        if(o.verbose>1) cout <<" - reject" <<endl;
        //reject new points and adapte stepsize|damping
        if(o.useAdaptiveDamping) { //Levenberg-Marquardt type damping
          lambda = 10.*lambda;
          break;
        } else {
          if(alpha*absMax(Delta)<1e-3*o.stopTolerance || evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
          alpha = .1*alpha;
        }
      }
    }

    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha <<' ' <<x <<endl;

    //stopping criterion
    if( (lambda<2. && absMax(Delta)<o.stopTolerance) ||
        (lambda<2. && alpha*absMax(Delta)<1e-3*o.stopTolerance) ||
        (evals>=o.stopEvals) ||
        (it>=o.stopIters) ) break;
  }
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MT_MSVC
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l",NULL,true);
#endif
  return evals;
}

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
uint optNewton(arr& x, ScalarFunction& f,  OptOptions o, arr *addRegularizer, double *fx_user, arr *gx_user, arr *Hx_user) {
  double alpha = 1.;
  double lambda = o.damping;
  double fx, fy;
  arr Delta, y;
  uint evals=0;

  //compute initial costs
  arr gx,Hx,gy,Hy;
  if(fx_user && gx_user && Hx_user) { //pre-condition!: assumes S is correctly evaluated at x!!
    if(sanityCheck) {
      fx = f.fs(gx, Hx, x);
      CHECK(fabs(fx-*fx_user) <1e-6,"");
      CHECK((maxDiff(gx,*gx_user) + maxDiff(Hx,*Hx_user) + fabs(fx-*fx_user))<1e-6,"");
    }
    fx = *fx_user;
    gx = *gx_user;
    Hx = *Hx_user;
  } else {
    fx = f.fs(gx, Hx, x);  evals++;
    if(addRegularizer)  fx += scalarProduct(x,(*addRegularizer)*vectorShaped(x));
  }

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" lambda=" <<lambda <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha <<' ' <<x <<endl;

  for(uint it=1;; it++) { //iterations and lambda adaptation loop
    if(o.verbose>1) cout <<"optNewton it=" <<it << " lambda=" <<lambda <<flush;

    //compute Delta
#if 1
    arr R=Hx;
    if(lambda) { //Levenberg Marquardt damping
      if(R.special==arr::RowShiftedPackedMatrixST) for(uint i=0; i<R.d0; i++) R(i,0) += lambda; //(R(i,0) is the diagonal in the packed matrix!!)
      else for(uint i=0; i<R.d0; i++) R(i,i) += lambda;
    }
    if(addRegularizer) {
      if(R.special==arr::RowShiftedPackedMatrixST) R = unpack(R);
//      cout <<*addRegularizer <<R <<endl;
      lapack_Ainv_b_sym(Delta, R + (*addRegularizer), -(gx+(*addRegularizer)*vectorShaped(x)));
    } else {
      lapack_Ainv_b_sym(Delta, R, -gx);
    }
#else //this uses lapack's LLS minimizer - but is really slow!!
    x.reshape(x.N);
    if(lambda) {
      arr D; D.setDiag(sqrt(.5*lambda),x.N);
      J.append(D);
      phi.append(zeros(x.N,1));
    }
    lapack_min_Ax_b(Delta, J, J*x - phi);
    Delta -= x;
#endif
    if(o.maxStep>0. && absMax(Delta)>o.maxStep)  Delta *= o.maxStep/absMax(Delta);
    if(o.verbose>1) cout <<" \t|Delta|=" <<absMax(Delta) <<flush;

    //lazy stopping criterion: stop without any update
    if(lambda<2. && absMax(Delta)<1e-1*o.stopTolerance){
      if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
      break;
    }

    for(;;) { //stepsize adaptation loop -- doesn't iterate for useDamping option
      y = x + alpha*Delta;
      fy = f.fs(gy, Hy, y);  evals++;
      if(addRegularizer) fy += scalarProduct(y,(*addRegularizer)*vectorShaped(y));
      if(o.verbose>2) cout <<" \tprobing y=" <<y;
      if(o.verbose>1) cout <<" \talpha=" <<alpha <<" \tevals=" <<evals <<" \tf(y)=" <<fy <<flush;
      //CHECK(fy==fy, "cost seems to be NAN: ly=" <<fy);
      if(fy!=NAN && fy <= fx) {
        if(o.verbose>1) cout <<" - ACCEPT" <<endl;
        //adopt new point and adapt stepsize|damping
        x = y;
        fx = fy;
        gx = gy;
        Hx = Hy;
        if(o.useAdaptiveDamping) { //Levenberg-Marquardt type damping
          lambda = .2*lambda;
        } else {
          if(alpha>.9) lambda = .5*lambda;
          alpha = pow(alpha, 0.5);
        }
        break;
      } else {
        if(o.verbose>1) cout <<" - reject" <<endl;
        //reject new points and adapte stepsize|damping
        if(o.useAdaptiveDamping) { //Levenberg-Marquardt type damping
          lambda = 10.*lambda;
          break;
        } else {
          if(alpha*absMax(Delta)<1e-3*o.stopTolerance || evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
//          if(alpha<1e-2) lambda = pow(lambda, 0.5);
          alpha = .1*alpha;
//          break;
        }
      }
    }

    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha <<' ' <<x <<endl;

    //stopping criterion
    if( (lambda<2. && absMax(Delta)<o.stopTolerance) ||
        (lambda<2. && alpha*absMax(Delta)<1e-3*o.stopTolerance) ||
        (evals>=o.stopEvals) ||
        (it>=o.stopIters) ) break;
  }
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MT_MSVC
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l",NULL,true);
#endif
  if(fx_user) *fx_user = fx;
  if(gx_user) *gx_user = gx;
  if(Hx_user) *Hx_user = Hx;
  return evals;
}

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
  
  grad_x /= norm(grad_x);
  
  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f.fs(grad_y, NoArr, y);  evals++;
    CHECK(fy==fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_cost <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|grad|=" <<norm(grad_y) <<" \ta=" <<a;
    
    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=norm(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/norm(grad_y);
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
  bool step(arr& w, const arr& grad, uint *singleI);
};

Rprop::Rprop() {
  s = new sRprop;
  s->incr   = 1.2;
  s->decr   = .33;
  s->dMax = 50.;
  s->dMin = 1e-6;
  s->rMax = 0.;
  s->delta0 = 1.;
}

void Rprop::init(double initialStepSize, double minStepSize, double maxStepSize) {
  s->stepSize.resize(0);
  s->lastGrad.resize(0);
  s->delta0 = initialStepSize;
  s->dMin = minStepSize;
  s->dMax = maxStepSize;
}

bool sRprop::step(arr& w, const arr& grad, uint *singleI) {
  if(!stepSize.N) { //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK(grad.N==stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK(w.N==stepSize.N   , "Rprop: parameter dimensionality changed!");
  
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
  
  return stepSize.max() < incr*dMin;
}

bool Rprop::step(arr& x, ScalarFunction& f) {
  arr grad;
  f.fs(grad, NoArr, x);
  return s->step(x, grad, NULL);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 ScalarFunction& f,
                 double *fmin_return,
                 double stoppingTolerance,
                 double initialStepSize,
                 uint maxEvals,
                 uint verbose) {
                 
  if(!s->stepSize.N) init(initialStepSize);
  arr x, J(_x.N), x_min, J_min;
  double fx, fx_min=0;
  uint rejects=0, small_steps=0;
  x=_x;
  
  if(verbose>1) cout <<"*** optRprop: starting point x=" <<x <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.opt");
  
  uint evals=0;
  double diff=0.;
  for(;;) {
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    fx = f.fs(J, NoArr, x);  evals++;
    
    if(verbose>0) fil <<evals <<' ' <<eval_cost <<' ' << fx <<' ' <<diff <<' ' <<x <<endl;
    if(verbose>1) cout <<"optRprop " <<evals <<' ' <<eval_cost <<" \tf(x)=" <<fx <<" \tdiff=" <<diff <<" \tx=" <<x <<endl;
    
    //infeasible point! undo the previous step
    if(fx==NAN) {
      if(!evals) HALT("can't start Rprop with unfeasible point");
      s->stepSize*=(double).1;
      s->lastGrad=(double)0.;
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
        s->stepSize*=(double).1;
        s->lastGrad=(double)0.;
        x=x_min;
        fx=fx_min;
        J=J_min;
        rejects=0;
      }
    }
    
    //update x
    s->step(x, J, NULL);
    
    //check stopping criterion based on step-length in x
    diff=maxDiff(x, x_min);
    
    if(diff<stoppingTolerance) { small_steps++; } else { small_steps=0; }
    if(small_steps>3)  break;
    if(evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
  if(fmin_return) *fmin_return= fx_min;
  _x=x_min;
  return evals;
}
