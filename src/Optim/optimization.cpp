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

///// return type for a function that returns a square potential $f(x) = x^T A x - 2 a^T x + c
//struct     SqrPotential;
///// return type for a function that returns a square potential $f(x,y) = [x,y]^T [A,C; C^T,B] [x,y] - 2 [a,b]^T [x,y] + c$
//struct PairSqrPotential;

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

/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i)^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)^T f_{ij}(x_i,x_j)$
/// and we can access local Jacobians of f_i and f_{ij}
struct VectorChainFunction;

///// Given a chain $x_{0:T}$ of variables, implies a cost function
///// $f(x) = \sum_{i=0}^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)$
///// and we can access local SqrPotential approximations of f_i and f_{ij}
//struct QuadraticChainFunction;

//===========================================================================
//
// checks, evaluation and converters
//

//double evaluateSP(const SqrPotential& S, const arr& x) {
//  return scalarProduct(x,S.A*x) - 2.*scalarProduct(S.a,x) + S.c;
//}

//double evaluatePSP(const PairSqrPotential& S, const arr& x, const arr& y) {
//  double f=0.;
//  f += scalarProduct(x,S.A*x);
//  f += scalarProduct(y,S.B*y);
//  f += 2.*scalarProduct(x,S.C*y);
//  f -= 2.*scalarProduct(S.a,x);
//  f -= 2.*scalarProduct(S.b,y);
//  f += S.c;
//  return f;
//}

//double evaluateCSP(const MT::Array<SqrPotential>& fi, const MT::Array<PairSqrPotential>& fij, const arr& x) {
//  double f=0.;
//  uint T=fi.N-1;
//  for(uint t=0; t<=T; t++) {
//    f += evaluateSP(fi(t), x[t]);
//    if(t<T) f += evaluatePSP(fij(t), x[t], x[t+1]);
//  }
//  return f;
//}

//void recomputeChainSquarePotentials(MT::Array<SqrPotential>& fi, MT::Array<PairSqrPotential>& fij, QuadraticChainFunction& f, const arr& x, uint& evals) {
//  uint T=fi.N-1;
//  for(uint t=0; t<=T; t++) {
//    f.fq_i(fi(t) , t, x[t]);  evals++;
//    if(t<T) f.fq_ij(fij(t), t, t+1, x[t], x[t+1]);
//  }
//}

//void sanityCheckUptodatePotentials(const MT::Array<SqrPotential>& R, QuadraticChainFunction& f, const arr& x) {
//  if(!sanityCheck) return;
//  SqrPotential R_tmp;
//  for(uint t=0; t<R.N; t++) {
//    f.fq_i(R_tmp, t, x[t]);
//    CHECK((maxDiff(R(t).A,R_tmp.A) + maxDiff(R(t).a,R_tmp.a) + fabs(R(t).c-R_tmp.c))<1e-6,"potentials not up-to-date");
//  }
//}

double evaluateSF(ScalarFunction& f, const arr& x) {
  return f.fs(NoArr, NoArr, x);
}

double evaluateVF(VectorFunction& f, const arr& x) {
  arr y;
  f.fv(y, NoArr, x);
  return sumOfSqr(y);
}

//double evaluateQCF(QuadraticChainFunction& f, const arr& x) {
//  double cost=0.;
//  uint T=x.d0-1;
//  cost += f.fq_i(NoPot, 0, x[0]);
//  for(uint t=1; t<=T; t++) {
//    cost += f.fq_i(NoPot, t, x[t]);
//    cost += f.fq_ij(NoPairPot, t-1, t, x[t-1], x[t]);
//  }
//  return cost;
//}

double evaluateVCF(VectorChainFunction& f, const arr& x) {
  double ncost=0.,pcost=0.;
  uint T=x.d0-1;
  arr y;
  f.fv_i(y, NoArr, 0, x[0]);  ncost += sumOfSqr(y);
  for(uint t=1; t<=T; t++) {
    f.fv_i(y, NoArr, t, x[t]);  ncost += sumOfSqr(y);
    f.fv_ij(y, NoArr, NoArr, t-1, t, x[t-1], x[t]);  pcost += sumOfSqr(y);
    //cout <<t <<' ' <<sumOfSqr(y) <<endl;
  }
  //cout <<"node costs=" <<ncost <<" pair costs=" <<pcost <<endl;
  return ncost+pcost;
}



/*double ScalarGraphFunction::f_total(const arr& X){
  uint n=X.d0;
  uintA E=edges();
  double f=0.;
  for(uint i=0;i<n;i++)    f += fi(NULL, i, X[i]);
  for(uint i=0;i<E.d0;i++) f += fij(NULL, NULL, E(i,0), E(i,1), X[E(i,0)], X[E(i,1)]);
  return f;
}

struct Tmp:public ScalarFunction{
  ScalarGraphFunction *sgf;

  double fs(arr* grad, const arr& X){
    uint n=X.d0,i,j,k;
    uintA E=sgf->edges();
    double f=0.;
    arr gi, gj;
    if(grad) (*grad).resizeAs(X);
    for(i=0;i<n;i++){
      f += sgf->fi((grad?&gi:NULL), i, X[i]);
      if(grad) (*grad)[i]() += gi;
    }
    for(k=0;k<E.d0;k++){
      i=E(k,0);
      j=E(i,1);
      f += sgf->fij((grad?&gi:NULL), (grad?&gj:NULL), i, j, X[i], X[j]);
      if(grad) (*grad)[i]() += gi;
      if(grad) (*grad)[j]() += gj;
    }
    return f;
  }
};

Tmp convert_ScalarFunction(ScalarGraphFunction& f){
  Tmp tmp;
  tmp.sgf=&f;
  return tmp;
}*/

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
    MT_MSG("checkGradient (scalar) -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient (scalar) -- SUCCESS (max diff error=" <<md <<")" <<endl;
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
    MT_MSG("checkGradient (vector) -- FAILURE -- max diff=" <<md <<" |"<<H.elem(i)<<'-'<<Jg.elem(i)<<"| (stored in files z.J_*)");
    MT::save(H, "z.J_analytical");
    MT::save(Jg, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient (vector) -- SUCCESS (max diff error=" <<md <<")" <<endl;
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
    MT_MSG("checkGradient (vector) -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    MT::save(J, "z.J_analytical");
    MT::save(JJ, "z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient (vector) -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
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

/// preliminary
uint optNodewise(arr& x, VectorChainFunction& f, OptOptions o) {

  struct MyVectorFunction:VectorFunction {
    VectorChainFunction *f;
    uint t;
    arr x_ref;
    uint *evals;
    void fv(arr& y, arr& J, const arr& x) {
      arr yij,Ji,Jj;
      f->fv_i(y, J, t, x); (*evals)++;
      if(t>0) {
        f->fv_ij(yij, (&J?Ji:NoArr), (&J?Jj:NoArr), t-1, t, x_ref[t-1], x);
        y.append(yij);
        if(&J) J.append(Jj);
      }
      if(t<f->get_T()) {
        f->fv_ij(yij, (&J?Ji:NoArr), (&J?Jj:NoArr), t, t+1, x, x_ref[t+1]);
        y.append(yij);
        if(&J) J.append(Ji);
      }
    }
  };
  
  uint evals = 0;
  
  MyVectorFunction f_loc;
  f_loc.f = &f;
  f_loc.evals = &evals;
  
  ofstream fil;
  double fx=evaluateVCF(f, x);
  if(o.verbose>0) fil.open("z.nodewise");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<endl;
  if(o.verbose>1) cout <<"optNodewise initial cost " <<fx <<endl;
  
  OptOptions op;
  op.stopTolerance=o.stopTolerance, op.stopEvals=10, op.maxStep=o.maxStep, op.verbose=0;
  
  uint k;
  for(k=0; k<o.stopIters; k++) {
    arr x_old=x;
    for(uint t=0; t<=f.get_T(); t++) {
      f_loc.x_ref = x;
      f_loc.t = t;
      //checkGradient(loc_f, x[t], 1e-4);
      optGaussNewton(x[t](), f_loc, op);
      if(o.verbose>1) cout <<"optNodewise " <<k <<" > " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    for(uint t=f.get_T()-1; t>0; t--) {
      f_loc.x_ref = x;
      f_loc.t=t;
      //checkGradient(loc_f, x[t], 1e-4);
      optGaussNewton(x[t](), f_loc, op);
      if(o.verbose>1) cout <<"optNodewise " <<k <<" < " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    fx = evaluateVCF(f, x);
    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<endl;
    if(maxDiff(x,x_old)<o.stopTolerance) break;
  }
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.nodewise' us 1:3 w l",NULL,true);
  
  return evals;
}


/// preliminary
//uint optDynamicProgramming(arr& x, QuadraticChainFunction& f, OptOptions o) {

//  uint T=x.d0-1,n=x.d1;
//  uint evals=0;
//  arr y(x);
//  double damping=o.damping;
  
//  MT::Array<SqrPotential> V(T+1);
//  MT::Array<SqrPotential> fi(T+1), fi_at_y(T+1);
//  MT::Array<PairSqrPotential> fij(T), fij_at_y(T);
//  arr Bbarinv(T,n,n);
//  arr bbar(T,n);
//  arr Id = eye(n,n);
  
//  recomputeChainSquarePotentials(fi, fij, f, x, evals);
//  //double fx = evaluateQCF(f, x);
//  double fx = evaluateCSP(fi, fij, x);
  
//  ofstream fil;
//  if(o.verbose>0) fil.open("z.DP");
//  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  if(o.verbose>1) cout <<"optDP initial cost " <<fx <<endl;
  
//  for(uint k=0; k<o.stopIters; k++) {
//    //backward
//    arr Bbar,C_Bbarinv;
//    double cbar;
//    init(V(T),n);
//    for(uint t=T; t--;) {
//      //f.fqi (&fi(t+1) , t+1, x[t+1]);  evals++;   //potentials should always be up-to-date (see recomputeChainSquarePotentials below)
//      //f.fqij(&fij(t), t, t+1, x[t], x[t+1]);
//      Bbar    = fij(t).B + fi(t+1).A + V(t+1).A + damping*Id;
//      bbar[t] = fij(t).b + fi(t+1).a + V(t+1).a + damping*x[t+1];
//      cbar    = fij(t).c + fi(t+1).c + V(t+1).c + damping*sumOfSqr(x[t+1]);
//      inverse_SymPosDef(Bbarinv[t](), Bbar);
//      V(t).c = cbar - scalarProduct(bbar[t], Bbarinv[t] * bbar[t]);
//      C_Bbarinv  = fij(t).C*Bbarinv[t];
//      V(t).a = fij(t).a - C_Bbarinv * bbar[t];
//      V(t).A = fij(t).A - C_Bbarinv * ~fij(t).C;
//    }
    
//    //forward
//    arr step;
//    double fy_from_V0;
//    if(!o.clampInitialState) {
//      arr Bbarinv0,bbar0;
//      Bbar  = fi(0).A + V(0).A + damping*Id;
//      bbar0 = fi(0).a + V(0).a + damping*x[0];
//      cbar  = fi(0).c + V(0).c + damping*sumOfSqr(x[0]);
//      inverse_SymPosDef(Bbarinv0, Bbar);
//      step = Bbarinv0*bbar0 - y[0];
//      if(o.maxStep>0. && norm(step)>o.maxStep)  step *= o.maxStep/norm(step);
//      y[0]() += step;
//      //y[0] = Bbarinv0*bbar0;
//      fy_from_V0 = cbar - scalarProduct(bbar0, Bbarinv0 * bbar0);
//    }
//    for(uint t=0; t<T; t++) {
//      step = Bbarinv[t]*(bbar[t] - (~fij(t).C)*y[t]) - y[t+1];
//      if(o.maxStep>0. && norm(step)>o.maxStep)  step *= o.maxStep/norm(step);
//      y[t+1]() += step;
//      //y[t+1] = Bbarinv[t]*(bbar[t] - (~fij(t).C)*y[t]);
//    }
    
//    recomputeChainSquarePotentials(fi_at_y, fij_at_y, f, y, evals);
//    double fy=evaluateCSP(fi_at_y, fij_at_y, y);
    
//    if(sanityCheck) {
//      double fy_exact=evaluateQCF(f, y);
//      CHECK(fabs(fy-fy_exact)<1e-6,"");
//    }
    
//    if(sanityCheck) {
//      //in the LGQ case, the fy above (V_0(x_0)) is exact and returns the cost-to-go
//      //.. we only need to subtract the damping cost:
//      double damping_cost=damping*sqrDistance(y,x);
//      fy_from_V0 -= damping_cost;
//      //.. but that estimate is useless in the non-linear case and we need to recompute potentials...
//      //CHECK(fabs(fy_from_V0-evaluateQCF(f, y))<1e-6,"");
//    }
    
//    if(fy<=fx) {
//      if(maxDiff(x,y)<o.stopTolerance) { x=y;  fx=fy;  break; }
//      x=y;
//      fx=fy;
//      fi = fi_at_y;
//      fij = fij_at_y;
//      damping /= 5.;
//    } else {
//      damping *= 10.;
//    }
    
//    if(o.verbose>1) cout <<"optDP " <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  }
//  if(o.verbose>0) fil.close();
//  if(o.verbose>1) gnuplot("plot 'z.DP' us 1:3 w l",NULL,true);
//  return evals;
//}

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
//    cout <<"gaussne R = " <<R <<endl;
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
uint optNewton(arr& x, ScalarFunction& f,  OptOptions o, double *fx_user, arr *gx_user, arr *Hx_user) {
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
    arr R=Hx;
//    cout <<"newton R = " <<R <<endl;
    if(lambda) { //Levenberg Marquardt damping
      if(R.special==arr::RowShiftedPackedMatrixST) for(uint i=0; i<R.d0; i++) R(i,0) += lambda; //(R(i,0) is the diagonal in the packed matrix!!)
      else for(uint i=0; i<R.d0; i++) R(i,i) += lambda;
    }
    lapack_Ainv_b_sym(Delta, R, -gx);
    if(o.maxStep>0. && norm(Delta)>o.maxStep)  Delta *= o.maxStep/norm(Delta);
    //if(o.maxStep>0. && absMax(Delta)>o.maxStep)  Delta *= o.maxStep/absMax(Delta);
    if(o.verbose>1) cout <<" \t|Delta|=" <<absMax(Delta) <<flush;

    //lazy stopping criterion: stop without any update
    if(lambda<2. && absMax(Delta)<1e-3*o.stopTolerance) break;

    for(;;) { //stepsize adaptation loop -- doesn't iterate for useDamping option
      y = x + alpha*Delta;
      fy = f.fs(gy, Hy, y);  evals++;
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

//void updateFwdMessage(SqrPotential& Sj, PairSqrPotential& fij, const SqrPotential& Ri, const SqrPotential& Si, double damping, const arr& x_damp) {
//  SqrPotential Sbar;
//  arr Sbarinv, C_Sbarinv;
//  arr Id = eye(Si.A.d0,Si.A.d1);
//  Sbar.A = fij.A + Ri.A + Si.A + damping*Id;
//  Sbar.a = fij.a + Ri.a + Si.a + damping*x_damp;
//  Sbar.c = fij.c + Ri.c + Si.c + damping*sumOfSqr(x_damp);
//  inverse_SymPosDef(Sbarinv, Sbar.A);
//  Sj.c = Sbar.c - scalarProduct(Sbar.a, Sbarinv * Sbar.a);
//  C_Sbarinv  = (~fij.C)*Sbarinv;
//  Sj.a = fij.b - C_Sbarinv * Sbar.a;
//  Sj.A = fij.B - C_Sbarinv * fij.C;
//}

//void updateBwdMessage(SqrPotential& Vi, PairSqrPotential& fij, const SqrPotential& Rj, const SqrPotential& Vj, double damping, const arr& x_damp) {
//  SqrPotential Vbar;
//  arr Vbarinv, C_Vbarinv;
//  arr Id = eye(Vi.A.d0,Vi.A.d1);
//  Vbar.A = fij.B + Rj.A + Vj.A + damping*Id;
//  Vbar.a = fij.b + Rj.a + Vj.a + damping*x_damp;
//  Vbar.c = fij.c + Rj.c + Vj.c + damping*sumOfSqr(x_damp);
//  inverse_SymPosDef(Vbarinv, Vbar.A);
//  Vi.c = Vbar.c - scalarProduct(Vbar.a, Vbarinv * Vbar.a);
//  C_Vbarinv  = fij.C*Vbarinv;
//  Vi.a = fij.a - C_Vbarinv * Vbar.a;
//  Vi.A = fij.A - C_Vbarinv * ~fij.C;
//}

/// preliminary
//uint optMinSumGaussNewton(arr& x, QuadraticChainFunction& f, OptOptions o) {

//  struct LocalQuadraticFunction:QuadraticFunction {
//    QuadraticChainFunction *f;
//    uint t;
//    arr x;
//    SqrPotential *S,*V,*R;
//    uint *evals;
//    bool updateR;
//    double fq(SqrPotential& S_loc, const arr& x) {
//      CHECK(&S_loc,"");
//      if(updateR) {
//        f->fq_i(*R , t, x); (*evals)++;
//      } else updateR = true;
//      S_loc.A = V->A+S->A+R->A;
//      S_loc.a = V->a+S->a+R->a;
//      S_loc.c = V->c+S->c+R->c;
//      return evaluateSP(S_loc,x);
//    }
//  };
  
//  uint T=x.d0-1,n=x.d1;
//  uint evals=0;
//  arr y(x);
//  double damping=o.damping;
//  uint rejects=0;
  
//  MT::Array<SqrPotential> V(T+1); //bwd messages
//  MT::Array<SqrPotential> S(T+1); //fwd messages
//  MT::Array<SqrPotential> Rx(T+1),Ry(T+1); //node potentials at x[t] (=hat x_t)
//  MT::Array<PairSqrPotential> fij(T);
//  for(uint t=0; t<=T; t++) { init(S(t),n);  init(V(t),n); }
  
//  //helpers
//  arr Sbarinv(T+1,n,n),Vbarinv(T+1,n,n);
//  arr Id = eye(n,n);
//  arr C_Sbarinv,C_Vbarinv;
  
//  //get all potentials
//  recomputeChainSquarePotentials(Rx, fij, f, x, evals);
//  //double fy;
//  double fx = evaluateCSP(Rx, fij, x);
//  //fx = evaluateQCF(f, x);
  
//  sanityCheckUptodatePotentials(Rx, f, x);
  
//  //update fwd & bwd messages
//  for(uint t=1; t<=T; t++) updateFwdMessage(S(t), fij(t-1), Rx(t-1), S(t-1), damping, x[t-1]);
//  for(uint t=T-1; t--;)   updateBwdMessage(V(t), fij(t), Rx(t+1), V(t+1), damping, x[t+1]);
  
//  ofstream fil;
//  if(o.verbose>0) fil.open("z.MSGN");
//  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  if(o.verbose>1) cout <<"optMSGN initial cost " <<fx <<endl;
  
//  for(uint k=0; k<o.stopIters; k++) {
//    y=x;
//    //fy=fx;
//    Ry=Rx;
    
//    sanityCheckUptodatePotentials(Ry, f, y);
    
//    bool fwd = (k+1)%2;
//    for(uint t=fwd?0:T-1; fwd?t<=T:t>0; t+=fwd?1:-1) {
//      SqrPotential Vbar,Sbar;
      
//      //update fwd & bwd messages
//      if(t>0) {
//        f.fq_ij(fij(t-1), t-1, t, y[t-1], y[t]);
//        updateFwdMessage(S(t), fij(t-1), Ry(t-1), S(t-1), damping, x[t-1]);
//      }
//      if(t<T) {
//        f.fq_ij(fij(t), t, t+1, y[t], y[t+1]);
//        updateBwdMessage(V(t), fij(t), Ry(t+1), V(t+1), damping, x[t+1]);
//      }
      
//      sanityCheckUptodatePotentials(Ry, f, y);
      
//      if(!t && o.clampInitialState) {
//        y[0] = x[0];
//        Ry(0)=Rx(0);
//        continue;
//      }
      
//      //iterate GaussNewton to find a new local y[t]
//      LocalQuadraticFunction f_loc;
//      f_loc.f = &f;
//      f_loc.t = t;
//      f_loc.evals = &evals;
//      f_loc.S = &S(t);
//      f_loc.V = &V(t);
//      f_loc.R = &Ry(t); //by setting this as reference, each recomputation of R is stored directly in R(t)
//      f_loc.x = x[t];
//      f_loc.updateR=false;
//      OptOptions op;
//      op.stopTolerance=o.stopTolerance, op.stopEvals=10, op.maxStep=o.maxStep, op.verbose=0;
//      NIY//optNewton(y[t](), f_loc, op);
      
//      sanityCheckUptodatePotentials(Ry, f, y);
      
//    }
    
//    //compute total cost
//    double fy=evaluateCSP(Ry, fij, y);
//    if(sanityCheck) {
//      double fy_exact=evaluateQCF(f, y);
//      CHECK(fabs(fy-fy_exact)<1e-6,"");
//    }
    
//    if(fy<=fx) {
//      rejects=0;
//      if(maxDiff(x,y)<o.stopTolerance) { x=y;  fx=fy;  break; }
//      x=y;
//      fx=fy;
//      Rx=Ry;
//      damping *= .2;
//    } else {
//      rejects++;
//      if(rejects>=5 && damping>1e3) break; //give up  //&& maxDiff(x,y)<stoppingTolerance
//      damping *= 10.;
//    }
    
//    if(o.verbose>1) cout <<"optMSGN " <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  }
//  if(o.verbose>1) cout <<"optMSGN " <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<damping <<endl;
//  if(o.verbose>0) fil.close();
//  if(o.verbose>1) gnuplot("plot 'z.MSGN' us 1:3 w l",NULL,true);
//  return evals;
//}



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
