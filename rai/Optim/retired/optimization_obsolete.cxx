/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================
//added Sep 2013

///// return type for a function that returns a square potential $f(x) = x^T A x - 2 a^T x + c
//struct     SqrPotential;
///// return type for a function that returns a square potential $f(x,y) = [x,y]^T [A,C; C^T,B] [x,y] - 2 [a,b]^T [x,y] + c$
//struct PairSqrPotential;

/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i)^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)^T f_{ij}(x_i,x_j)$
/// and we can access local Jacobians of f_i and f_{ij}
struct VectorChainFunction;

///// Given a chain $x_{0:T}$ of variables, implies a cost function
///// $f(x) = \sum_{i=0}^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)$
///// and we can access local SqrPotential approximations of f_i and f_{ij}
//struct QuadraticChainFunction;

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

//double evaluateCSP(const rai::Array<SqrPotential>& fi, const rai::Array<PairSqrPotential>& fij, const arr& x) {
//  double f=0.;
//  uint T=fi.N-1;
//  for(uint t=0; t<=T; t++) {
//    f += evaluateSP(fi(t), x[t]);
//    if(t<T) f += evaluatePSP(fij(t), x[t], x[t+1]);
//  }
//  return f;
//}

//void recomputeChainSquarePotentials(rai::Array<SqrPotential>& fi, rai::Array<PairSqrPotential>& fij, QuadraticChainFunction& f, const arr& x, uint& evals) {
//  uint T=fi.N-1;
//  for(uint t=0; t<=T; t++) {
//    f.fq_i(fi(t) , t, x[t]);  evals++;
//    if(t<T) f.fq_ij(fij(t), t, t+1, x[t], x[t+1]);
//  }
//}

//void sanityCheckUptodatePotentials(const rai::Array<SqrPotential>& R, QuadraticChainFunction& f, const arr& x) {
//  if(!sanityCheck) return;
//  SqrPotential R_tmp;
//  for(uint t=0; t<R.N; t++) {
//    f.fq_i(R_tmp, t, x[t]);
//    CHECK((maxDiff(R(t).A,R_tmp.A) + maxDiff(R(t).a,R_tmp.a) + fabs(R(t).c-R_tmp.c))<1e-6,"potentials not up-to-date");
//  }
//}

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
  double ncost=0., pcost=0.;
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
  for(uint i=0;i<n;i++)    f += fi(nullptr, i, X[i]);
  for(uint i=0;i<E.d0;i++) f += fij(nullptr, nullptr, E(i,0), E(i,1), X[E(i,0)], X[E(i,1)]);
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
      f += sgf->fi((grad?&gi:nullptr), i, X[i]);
      if(grad) (*grad)[i]() += gi;
    }
    for(k=0;k<E.d0;k++){
      i=E(k,0);
      j=E(i,1);
      f += sgf->fij((grad?&gi:nullptr), (grad?&gj:nullptr), i, j, X[i], X[j]);
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

/// preliminary
uint optNodewise(arr& x, VectorChainFunction& f, OptOptions o) {

  struct MyVectorFunction:VectorFunction {
    VectorChainFunction* f;
    uint t;
    arr x_ref;
    uint* evals;
    void fv(arr& y, arr& J, const arr& x) {
      arr yij, Ji, Jj;
      f->fv_i(y, J, t, x); (*evals)++;
      if(t>0) {
        f->fv_ij(yij, (!!J?Ji:NoArr), (!!J?Jj:NoArr), t-1, t, x_ref[t-1], x);
        y.append(yij);
        if(!!J) J.append(Jj);
      }
      if(t<f->get_T()) {
        f->fv_ij(yij, (!!J?Ji:NoArr), (!!J?Jj:NoArr), t, t+1, x, x_ref[t+1]);
        y.append(yij);
        if(!!J) J.append(Ji);
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
  if(o.verbose>0) fil <<0 <<' ' <<eval_count <<' ' <<fx <<endl;
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
      optNewton(x[t](), f_loc, op);
      if(o.verbose>1) cout <<"optNodewise " <<k <<" > " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    for(uint t=f.get_T()-1; t>0; t--) {
      f_loc.x_ref = x;
      f_loc.t=t;
      //checkGradient(loc_f, x[t], 1e-4);
      optNewton(x[t](), f_loc, op);
      if(o.verbose>1) cout <<"optNodewise " <<k <<" < " <<t <<' ' <<evaluateVCF(f, x) <<endl;
    }
    fx = evaluateVCF(f, x);
    if(o.verbose>0) fil <<evals <<' ' <<eval_count <<' ' <<fx <<endl;
    if(maxDiff(x, x_old)<o.stopTolerance) break;
  }
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.nodewise' us 1:3 w l", nullptr, true);

  return evals;
}

/// preliminary
//uint optDynamicProgramming(arr& x, QuadraticChainFunction& f, OptOptions o) {

//  uint T=x.d0-1,n=x.d1;
//  uint evals=0;
//  arr y(x);
//  double damping=o.damping;

//  rai::Array<SqrPotential> V(T+1);
//  rai::Array<SqrPotential> fi(T+1), fi_at_y(T+1);
//  rai::Array<PairSqrPotential> fij(T), fij_at_y(T);
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
//      if(o.maxStep>0. && length(step)>o.maxStep)  step *= o.maxStep/length(step);
//      y[0]() += step;
//      //y[0] = Bbarinv0*bbar0;
//      fy_from_V0 = cbar - scalarProduct(bbar0, Bbarinv0 * bbar0);
//    }
//    for(uint t=0; t<T; t++) {
//      step = Bbarinv[t]*(bbar[t] - (~fij(t).C)*y[t]) - y[t+1];
//      if(o.maxStep>0. && length(step)>o.maxStep)  step *= o.maxStep/length(step);
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
//  if(o.verbose>1) gnuplot("plot 'z.DP' us 1:3 w l",nullptr,true);
//  return evals;
//}

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

//  rai::Array<SqrPotential> V(T+1); //bwd messages
//  rai::Array<SqrPotential> S(T+1); //fwd messages
//  rai::Array<SqrPotential> Rx(T+1),Ry(T+1); //node potentials at x[t] (=hat x_t)
//  rai::Array<PairSqrPotential> fij(T);
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
//  if(o.verbose>1) gnuplot("plot 'z.MSGN' us 1:3 w l",nullptr,true);
//  return evals;
//}

struct sConvert {
  struct VectorChainFunction_ScalarFunction:ScalarFunction { //actual converter objects
    VectorChainFunction* f;
    VectorChainFunction_ScalarFunction(VectorChainFunction& _f):f(&_f) {}
    virtual double fs(arr& grad, arr& H, const arr& x);
  };

  struct VectorChainFunction_VectorFunction:VectorFunction { //actual converter objects
    VectorChainFunction* f;
    VectorChainFunction_VectorFunction(VectorChainFunction& _f):f(&_f) {}
    virtual void   fv(arr& y, arr& J, const arr& x);
  };

  //  struct VectorChainFunction_QuadraticChainFunction:QuadraticChainFunction {
  //    VectorChainFunction *f;
  //    VectorChainFunction_QuadraticChainFunction(VectorChainFunction& _f):f(&_f) {}
  //    virtual uint get_T() { return f->get_T(); }
  //    virtual double fq_i(SqrPotential& S, uint i, const arr& x_i);
  //    virtual double fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j);
  //  };
  // #ifndef libRoboticsCourse
  //   struct ControlledSystem_1OrderMarkovFunction:KOrderMarkovFunction {
  //     ControlledSystem *sys;
  //     ControlledSystem_1OrderMarkovFunction(ControlledSystem& _sys):sys(&_sys){}
  //     uint get_T(){ return sys->get_T(); }
  //     uint get_k(){ return 1; }
  //     uint get_n(){ return sys->get_xDim(); }
  //     uint get_m(uint t);
  //     void phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z=NoArr, const arr& J_z=NoArr);
  //   };

  //   struct ControlledSystem_2OrderMarkovFunction:KOrderMarkovFunction {
  //     ControlledSystem *sys;
  //     ControlledSystem_2OrderMarkovFunction(ControlledSystem& _sys):sys(&_sys){}
  //     uint get_T(){ return sys->get_T(); }
  //     uint get_k(){ return 2; }
  //     uint get_n(){ return sys->get_xDim()/2; }
  //     uint get_m(uint t);
  //     void phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z=NoArr, const arr& J_z=NoArr);
  //   };
  // #endif
};

Convert::operator VectorChainFunction& () {
  if(!self->vcf) {
  }
  if(!self->vcf) HALT("");
  return *self->vcf;
}

//Convert::operator QuadraticChainFunction&() {
//  if(!self->qcf) {
//    if(self->vcf) self->qcf = new sConvert::VectorChainFunction_QuadraticChainFunction(*self->vcf);
//  }
//  if(!self->qcf) HALT("");
//  return *self->qcf;
//}

double sConvert::VectorChainFunction_ScalarFunction::fs(arr& grad, arr& H, const arr& x) {
  uint T=f->get_T();
  arr z;  z.referTo(x);
  z.reshape(T+1, z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)

  double cost=0.;
  arr y, J, Ji, Jj;
  if(!!grad) {
    grad.resizeAs(x);
    grad.setZero();
  }
  if(!!H) NIY;
  for(uint t=0; t<=T; t++) { //node potentials
    f->fv_i(y, (!!grad?J:NoArr), t, z[t]);
    cost += sumOfSqr(y);
    if(!!grad) {
      grad[t]() += 2.*(~y)*J;
    }
  }
  for(uint t=0; t<T; t++) {
    f->fv_ij(y, (!!grad?Ji:NoArr), (!!grad?Jj:NoArr), t, t+1, z[t], z[t+1]);
    cost += sumOfSqr(y);
    if(!!grad) {
      grad[t]()   += 2.*(~y)*Ji;
      grad[t+1]() += 2.*(~y)*Jj;
    }
  }
  return cost;
}

void sConvert::VectorChainFunction_VectorFunction::fv(arr& y, arr& J, const arr& x) {
  uint T=f->get_T();
  arr z;  z.referTo(x);
  z.reshape(T+1, z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)

  //probing dimensionality (ugly..)
  arr tmp;
  f->fv_i(tmp, NoArr, 0, z[0]);
  uint di=tmp.N; //dimensionality at nodes
  if(T>0) f->fv_ij(tmp, NoArr, NoArr, 0, 1, z[0], z[1]);
  uint dij=tmp.N; //dimensionality at pairs

  //resizing things:
  arr yi(T+1, di); //the part of y which will collect all node potentials
  arr yij(T, dij);  //the part of y which will collect all pair potentials
  arr Ji;  Ji .resize(TUP(T+1, di, z.d0, z.d1)); //first indices as yi, last: gradient w.r.t. x
  arr Jij; Jij.resize(TUP(T, dij, z.d0, z.d1));   //first indices as yi, last: gradient w.r.t. x
  Ji.setZero();
  Jij.setZero();

  arr y_loc, J_loc, Ji_loc, Jj_loc;
  uint t, i, j;
  //first collect all node potentials
  for(t=0; t<=T; t++) {
    f->fv_i(y_loc, (!!J?J_loc:NoArr), t, z[t]);
    yi[t] = y_loc;
    if(!!J) {
      for(i=0; i<di; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Ji.elem(TUP(t, i, t, j)) = J_loc(i, j);
    }
  }
  //then collect all pair potentials
  for(t=0; t<T; t++) {
    f->fv_ij(y_loc, (!!J?Ji_loc:NoArr), (!!J?Jj_loc:NoArr), t, t+1, z[t], z[t+1]);
    yij[t] = y_loc;
    if(!!J) {
      for(i=0; i<dij; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Jij.elem(TUP(t, i, t, j)) = Ji_loc(i, j);
      for(i=0; i<dij; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Jij.elem(TUP(t, i, t+1, j)) = Jj_loc(i, j);
    }
  }
  yi.reshape((T+1)*di);
  Ji.reshape((T+1)*di, x.N);
  yij.reshape(T*dij);
  Jij.reshape(T*dij, x.N);
  y=yi;  y.append(yij);
  if(!!J) { J=Ji;  J.append(Jij); }
}

//double sConvert::VectorChainFunction_QuadraticChainFunction::fq_i(SqrPotential& S, uint i, const arr& x_i) {
//  arr y,J;
//  f->fv_i(y, (!!S?J:NoArr), i, x_i);
//  if(!!S) {
//    S.A=~J * J;
//    S.a=~J * (J*x_i - y);
//    S.c=sumOfSqr(J*x_i - y);
//  }
//  return sumOfSqr(y);
//}

//double sConvert::VectorChainFunction_QuadraticChainFunction::fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) {
//  arr y,Ji,Jj;
//  f->fv_ij(y, (!!S?Ji:NoArr), (!!S?Jj:NoArr), i, j, x_i, x_j);
//  if(!!S) {
//    S.A=~Ji*Ji;
//    S.B=~Jj*Jj;
//    S.C=~Ji*Jj;
//    S.a=~Ji*(Ji*x_i + Jj*x_j - y);
//    S.b=~Jj*(Ji*x_i + Jj*x_j - y);
//    S.c=sumOfSqr(Ji*x_i + Jj*x_j - y);
//  }
//  return sumOfSqr(y);
//}

// #ifndef libRoboticsCourse
// uint sConvert::ControlledSystem_1OrderMarkovFunction::get_m(uint t){
//   uint T=get_T();
//   if(t==0)   return sys->get_xDim() + sys->get_phiDim(t) + sys->get_xDim();
//   if(t==T-1) return sys->get_xDim() + sys->get_phiDim(t) + sys->get_phiDim(T);
//   return sys->get_xDim() + sys->get_phiDim(t);
// } //dynamic gap plus task costs

// void sConvert::ControlledSystem_1OrderMarkovFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z, const arr& J_z){
//   arr x0(x_bar,0);
//   arr x1(x_bar,1);

//   sys->setx(x0);

//   //dynamics
//   arr J0, J1;
//   getTransitionCostTerms(*sys, true, phi, J0, J1, x0, x1, t);
//   if(!!J){
//     J.resize(J0.d0, J0.d1+J1.d1);
//     J.setMatrixBlock(J0,0,0);
//     J.setMatrixBlock(J1,0,J0.d1);
//   }

//   //task phi w.r.t. x0
//   arr _phi, _J;
//   sys->getTaskCosts(_phi, _J, t);
//   _J.insColumns(x0.N, x1.N);
//   for(uint i=0;i<_J.d0;i++) for(uint j=x0.N;j<_J.d1;j++) _J(i,j) = 0.;
//   phi.append(_phi);
//   if(!!J) J.append(_J);

//   if(t==get_T()-1){ //second task phi w.r.t. x1 in the final factor
//     sys->setx(x1);
//     sys->getTaskCosts(_phi, _J, t+1);
//     phi.append(_phi);
//     if(!!J){
//       _J.insColumns(0, x0.N);
//       for(uint i=0;i<_J.d0;i++) for(uint j=0;j<x1.N;j++) _J(i,j) = 0.;
//       J.append(_J);
//     }
//   }

//   if(t==0){ //initial x0 constraint
//     double prec=1e4;
//     arr sys_x0;
//     sys->get_x0(sys_x0);
//     phi.append(prec*(x0-sys_x0));
//     if(!!J){
//       _J.setDiag(prec,x0.N);
//       _J.insColumns(x0.N, x1.N);
//       for(uint i=0;i<_J.d0;i++) for(uint j=0;j<x1.N;j++) _J(i,x0.N+j) = 0.;
//       J.append(_J);
//     }
//   }
// }

// uint sConvert::ControlledSystem_2OrderMarkovFunction::get_m(uint t){
//   uint T=get_T();
//   uint nq = get_n();
//   if(t==0)     return 3*nq + sys->get_phiDim(0) + sys->get_phiDim(1);
//   if(t==T-2)   return nq + sys->get_phiDim(t+1) + sys->get_phiDim(T);
//   return nq + sys->get_phiDim(t+1);
// } //dynamic-gap task-costs

// void sConvert::ControlledSystem_2OrderMarkovFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z, const arr& J_z){
//   uint n=get_n();
//   CHECK(x_bar.d0==3 && x_bar.d1==n,"");
//   arr q0(x_bar,0);
//   arr q1(x_bar,1);
//   arr q2(x_bar,2);
//   double tau=sys->get_tau();
//   double _tau2=1./(tau*tau);
//   arr x0=q0; x0.append((q1-q0)/tau);
//   arr x1=q1; x1.append((q2-q0)/(2.*tau));
//   arr x2=q2; x2.append((q2-q1)/tau);

//   //dynamics
//   double h=1e-1;
//   phi = h*_tau2*(q2-2.*q1+q0); //penalize acceleration
//   if(!!J){ //we todoalso need to return the Jacobian
//     J.resize(n,3,n);
//     J.setZero();
//     for(uint i=0;i<n;i++){  J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
//     J.reshape(n,3*n);
//     J *= h*_tau2;
//   }

//   //task phi w.r.t. x2
//   arr _phi, _J;
//   sys->setx(x1);
//   sys->getTaskCosts(_phi, _J, t+1);
//   phi.append(_phi);
//   if(!!J) {
//     arr Japp(_J.d0,3*n);
//     Japp.setZero();
//     Japp.setMatrixBlock(_J.sub(0,-1,0,n-1), 0, 1*n); //w.r.t. q1
//     Japp.setMatrixBlock((-0.5/tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
//     Japp.setMatrixBlock(( 0.5/tau)*_J.sub(0,-1,n,-1), 0, 2*n); //w.r.t. q2
//     J.append(Japp);
//   }

//   if(t==0){
//     double prec=1e4;
//     arr sys_x0;
//     sys->get_x0(sys_x0);
//     phi.append(prec*(x0-sys_x0));
//     _J = diag(prec,x0.N);
//     if(!!J){
//       arr Japp(_J.d0,3*n);
//       Japp.setZero();
//       Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) - (1./tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
//       Japp.setMatrixBlock((1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
//       J.append(Japp);
//     }
//   }

//   if(t==0){ //also add costs w.r.t. q0 and (q1-q0)/tau
//     sys->setx(x0);
//     sys->getTaskCosts(_phi, _J, 0);
//     phi.append(_phi);
//     if(!!J) {
//       arr Japp(_J.d0,3*n);
//       Japp.setZero();
//       Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) - (1./tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
//       Japp.setMatrixBlock((1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
//       J.append(Japp);
//     }
//   }

//   uint T=get_T();
//   if(t==T-2){
//     sys->setx(x2);
//     sys->getTaskCosts(_phi, _J, T);
//     phi.append(_phi);
//     if(!!J) {
//       arr Japp(_J.d0,3*n);
//       Japp.setZero();
//       Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) + (1./tau)*_J.sub(0,-1,n,-1), 0, 2*n); //w.r.t. q2
//       Japp.setMatrixBlock((-1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
//       J.append(Japp);
//     }
//   }
// }
// #endif

VectorChainCost::VectorChainCost(uint _T, uint _n) {
  T=_T; n=_n;
  A.resize(T+1, n, n);  a.resize(T+1, n);
  Wi.resize(T, n, n);  Wj.resize(T, n, n);    w.resize(T, n);
  for(uint t=0; t<=T; t++) generateConditionedRandomProjection(A[t](), n, 100.);
  for(uint t=0; t<T; t++) {
    generateConditionedRandomProjection(Wi[t](), n, 100.);
    generateConditionedRandomProjection(Wj[t](), n, 100.);
  }
  rndUniform(a, -1., 1., false);
  rndUniform(w, -1., 1., false);
  nonlinear=false;
}

void VectorChainCost::fv_i(arr& y, arr* J, uint i, const arr& x_i) {
  if(!nonlinear) {
    y = A[i]*x_i + a[i];
    if(J) *J = A[i];
  } else {
    arr xi=atan(x_i);
    y = A[i]*xi + a[i];
    if(J) {
      arr gi(xi.N);
      for(uint k=0; k<gi.N; k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      *J = A[i]*diag(gi);
    }
  }
}

void VectorChainCost::fv_ij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j) {
  if(!nonlinear) {
    y=Wi[i]*x_i + Wj[i]*x_j + w[i];
    if(Ji) *Ji = Wi[i];
    if(Jj) *Jj = Wj[i];
  } else {
    arr xi=atan(x_i);
    arr xj=atan(x_j);
    y=Wi[i]*xi + Wj[i]*xj + w[i];
    if(Ji && Ji) {
      arr gi(xi.N), gj(xi.N);
      for(uint k=0; k<gi.N; k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      for(uint k=0; k<gj.N; k++) gj(k) = 1./(1.+x_j(k)*x_j(k));
      *Ji = Wi[i]*diag(gi);
      *Jj = Wj[i]*diag(gj);
    }
  }
}

SlalomProblem::SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power) {
  T=_T;
  n=2;
  K=_K;
  margin = _margin;
  w = _w;
  power = _power;
}

double border(double* grad, double x, double power=8.) {
  if(x>0.) { if(grad) *grad=0.; return 0.; }
  double y = pow(x, power);
  if(grad) *grad = power*pow(x, power-1.);
  return y;
}

double tannenbaum(double* grad, double x, double power=8.) {
  double y=x*x;
  if(grad) *grad = power*pow(y-floor(y), power-1.) * (2.*x);
  y=floor(y) + pow(y-floor(y), power);
  return y;
}

void SlalomProblem::fv_i(arr& y, arr& J, uint i, const arr& x_i) {
  eval_count++;
  CHECK_EQ(x_i.N, 2, "");
  y.resize(1);  y(0)=0.;
  if(!!J) { J.resize(1, 2);  J.setZero(); }
  if(!(i%(T/K))) {
    uint obstacle=i/(T/K);
    if(obstacle&1) { //top obstacle
      double d=(x_i(0)-1.)/margin;
//       y(0) = tannenbaum((J?&(*J)(0,0):nullptr), d, power);
      y(0) = border((!!J?&J(0, 0):nullptr), d, power);
      if(!!J) J(0, 0) /= margin;
    } else {
      double d=-(x_i(0)+1.)/margin;
//       y(0) = tannenbaum((J?&J(0,0):nullptr), d, power);
      y(0) = border((!!J?&J(0, 0):nullptr), d, power);
      if(!!J) J(0, 0) /= -margin;
    }
  }
}

void SlalomProblem::fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) {
  y.resize(1);
  double tau=.01;
  arr A= {1., tau, 0., 1.};  A.reshape(2, 2);
  arr M=w*diag({2./(tau*tau), 1./tau});  //penalize variance in position & in velocity (control)
  y=M*(x_j - A*x_i);
  if(!!Ji) { Ji = -M*A; }
  if(!!Jj) { Jj = M; }
}

//===========================================================================

#include "optimization_obsolete.h"

#ifdef RAI_GSL
#include <gsl/gsl_cdf.h>
bool DecideSign::step(double x) {
  N++;
  sumX+=x;
  sumXX+=x*x;
  if(N<=10) return false;
  if(!sumX) return true;
  double m=sumX/N;
  double s=sqrt((sumXX-sumX*m)/(N-1));
  double t=sqrt(N)*fabs(m)/s;
  double T=gsl_cdf_tdist_Pinv(1.-1e-6, N-1); //decide with error-prob 1e-4 if sign is significant
  if(t>T) return true;
  return false;
}
#else
bool DecideSign::step(double x) { NIY; }
#endif

void OnlineRprop::init(OptimizationProblem* _m, double initialRate, uint _N, const arr& w0) {
  rprop.init(initialRate);
  t=0;
  m=_m;
  N=_N;
  perm.setRandomPerm(N);
  w=w0;
  signer.resize(w.N);
  for(uint i=0; i<w.N; i++) signer(i).init();
  l=0.;
  e=0.;
  rai::open(log, "log.sgd");
}

void OnlineRprop::step() {
  arr grad;
  double err;
  l += m->loss(w, perm(t%N), &grad, &err);
  e += err;
  for(uint i=0; i<w.N; i++) {
    if(signer(i).step(grad(i))) { //signer is certain
      grad(i) = signer(i).sign(); //hard assign the gradient to +1 or -1
      rprop.step(w, grad, &i);      //make an rprop step only in this dimension
      signer(i).init();
      //cout <<"making step in " <<i <<endl;
    } else if(signer(i).N>1000) {
      grad(i) = 0.;
      rprop.step(w, grad, &i);      //make an rprop step only in this dimension
      signer(i).init();
      //cout <<"assuming 0 grad in " <<i <<endl;
    }
  }
  log <<t
      <<" time= " <<rai::timerRead()
      <<" loss= " <<l/(t%BATCH+1)
      <<" err= "  <<e/(t%BATCH+1)
      <<endl;
  cout <<t
       <<" time= " <<rai::timerRead()
       <<" loss= " <<l/(t%BATCH+1)
       <<" err= "  <<e/(t%BATCH+1)
       <<endl;
  t++;
  if(!(t%N)) perm.setRandomPerm(N);
  if(!(t%BATCH)) {
    l=0.;
    e=0.;
  }
}
