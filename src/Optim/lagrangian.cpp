/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "lagrangian.h"
#include "../Core/graph.h"

#include <math.h>

//==============================================================================
//
// LagrangianProblem
//

double I_lambda_x(uint i, arr& lambda, arr& g) {
  if(g(i)>0. || (lambda.N && lambda(i)>0.)) return 1.;
  return 0.;
}

//==============================================================================

LagrangianProblem::LagrangianProblem(const shared_ptr<NLP>& P, const rai::OptOptions& opt, arr& lambdaInit)
  : P(P), muLB(0.), mu(0.), useLB(false) {

  CHECK(P, "null problem given");

  ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
    return this->lagrangian(dL, HL, x);
  });

  if(opt.constrainedMethod==rai::logBarrier) useLB=true;

  //switch on penalty terms
  mu=opt.muInit;
  muLB=opt.muLBInit;

  if(!!lambdaInit) lambda = lambdaInit;

  featureTypes.clear();
  for(ObjectiveType& t:P->featureTypes) {
    if(t==OT_f) featureTypes.append(OT_f);                    // direct cost term
    if(t==OT_sos) featureTypes.append(OT_sos);                // sumOfSqr term
    if(useLB    && t==OT_ineq) featureTypes.append(OT_f);     // log barrier
    if(!useLB   && t==OT_ineq) featureTypes.append(OT_sos);   // square g-penalty
    if(t==OT_ineqP) featureTypes.append(OT_sos);              // square g-penalty
    if(t==OT_ineq) featureTypes.append(OT_f);                 // g-lagrange terms
    if(t==OT_ineqB) featureTypes.append(OT_f);                // explicit log barrier
    if(t==OT_ineqB) featureTypes.append(OT_f);                // g-lagrange terms
    if(t==OT_eq) featureTypes.append(OT_sos);                 // square h-penalty
    if(t==OT_eq) featureTypes.append(OT_f);                   // h-lagrange terms
  }

}

void LagrangianProblem::evaluate(arr& phi, arr& J, const arr& _x) {
  //-- evaluate constrained problem and buffer
  if(_x!=x) {
    x=_x;
    P->evaluate(phi_x, J_x, x);
    P->getFHessian(H_x, x);
  } else { //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
  }

  CHECK(x.N, "zero-dim optimization variables!");
  CHECK_EQ(phi_x.N, J_x.d0, "Jacobian size inconsistent");
  CHECK_EQ(phi_x.N, P->featureTypes.N, "termType array size inconsistent");

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(phi_x.N);
  if(phi_x.N) I_lambda_x = false;
  if(!useLB) for(uint i=0; i<phi_x.N; i++) {
      if(P->featureTypes.p[i]==OT_ineq) I_lambda_x.p[i] = (phi_x.p[i]>0. || (lambda.N && lambda.p[i]>0.));
    }

  phi.resize(featureTypes.N).setZero();
  uint nphi=0;
  for(uint i=0; i<phi_x.N; i++) {
    ObjectiveType ot = P->featureTypes.p[i];
    if(ot==OT_f)   phi.p[nphi++] = phi_x.p[i];                                                              // direct cost term
    if(ot==OT_sos) phi.p[nphi++] = phi_x.p[i];                                                  // sumOfSqr term
    if(useLB    && ot==OT_ineq) { if(phi_x.p[i]>0.) phi.p[nphi++] = NAN; else phi.p[nphi++] =  -muLB * ::log(-phi_x.p[i]); }                   //log barrier, check feasibility
    if(!useLB   && ot==OT_ineq) { if(I_lambda_x.p[i]) phi.p[nphi++] = sqrt(mu)*phi_x.p[i]; else phi.p[nphi++] = 0.; }      //g-penalty
    if(ot==OT_ineqP) { if(phi_x.p[i]>0.) phi.p[nphi++] = sqrt(mu)*phi_x.p[i]; else phi.p[nphi++] = 0.; }                 //g-penalty
    if(ot==OT_ineq) { if(lambda.N && lambda.p[i]>0.) phi.p[nphi++] = lambda.p[i] * phi_x.p[i]; else nphi++; }               //g-lagrange terms
    if(ot==OT_ineqB) { if(phi_x.p[i]>0.) phi.p[nphi++] = NAN; else phi.p[nphi++] =  -muLB * ::log(-phi_x.p[i]); }                              //log barrier, check feasibility
    if(ot==OT_ineqB) { if(lambda.N && lambda.p[i]>0.) phi.p[nphi++] = lambda.p[i] * phi_x.p[i]; else nphi++; }              //g-lagrange terms
    if(ot==OT_eq) phi.p[nphi++] =  sqrt(mu) * phi_x.p[i];                        //h-penalty
    if(ot==OT_eq) { if(lambda.N) phi.p[nphi++] =  lambda.p[i] * phi_x.p[i]; else nphi++; }                       //h-lagrange terms
  }
  CHECK_EQ(nphi, phi.N, "");

  if(!!J) { //term Jacobians
    if(isSparse(J_x)) {
      J.sparse().resize(phi.N, J_x.d1, 0);
      J_x.sparse().setupRowsCols();
    } else {
      J.resize(phi.N, J_x.d1).setZero();
    }
    uint nphi=0;
    for(uint i=0; i<phi_x.N; i++) {
      ObjectiveType ot = P->featureTypes.p[i];
//#define J_setRow(fac) { J.setMatrixBlock(fac J_x.sparse().getSparseRow(i), nphi++, 0); }
//#define J_setRow(fac) { J.setMatrixBlock(fac ~J_x[i], nphi++, 0); }
#define J_setRow(fac) { if(!isSparse(J)) J.setMatrixBlock(fac ~J_x[i], nphi++, 0); else J.setMatrixBlock(fac J_x.sparse().getSparseRow(i), nphi++, 0); }
      if(ot==OT_f)   J_setRow()               // direct cost term
        if(ot==OT_sos) J_setRow()               // sumOfSqr terms
          if(useLB    && ot==OT_ineq) J_setRow((-muLB/phi_x.p[i])*)                      //log barrier, check feasibility
            if(!useLB   && ot==OT_ineq) { if(I_lambda_x.p[i]) J_setRow(sqrt(mu)*) else nphi++; }   //g-penalty
      if(ot==OT_ineqP) { if(phi_x.p[i]>0.) J_setRow(sqrt(mu)*) else nphi++; }              //g-penalty
      if(ot==OT_ineq) { if(lambda.N && lambda.p[i]>0.) J_setRow(lambda.p[i]*) else nphi++; }                           //g-lagrange terms
      if(ot==OT_ineqB) J_setRow((-muLB/phi_x.p[i])*)                                  //log barrier, check feasibility
        if(ot==OT_ineqB) { if(lambda.N && lambda.p[i]>0.) J_setRow(lambda.p[i]*) else nphi++; }                          //g-lagrange terms
      if(ot==OT_eq) J_setRow(sqrt(mu)*)                                    //h-penalty
        if(ot==OT_eq) { if(lambda.N) J_setRow(lambda.p[i]*)  else nphi++; }                                               //h-lagrange terms
#undef J_setRow
    }
    CHECK_EQ(nphi, phi.N, "");
  }
}

void LagrangianProblem::getFHessian(arr& H, const arr& x) {
  P->getFHessian(H, x);

  for(uint i=0; i<phi_x.N; i++) {
//    if(muLB     && P->featureTypes.p[i]==OT_ineq) NIY; //add something to the Hessian
//    if(useLB    && ot==OT_ineq) coeff.p[i] += (muLB/rai::sqr(phi_x.p[i]));                  //log barrier, check feasibility
//    if(            ot==OT_ineqB) coeff.p[i] += (muLB/rai::sqr(phi_x.p[i]));                 //log barrier, check feasibility
  }

}

double LagrangianProblem::lagrangian(arr& dL, arr& HL, const arr& _x) {
#if 0
  return eval_scalar(dL, HL, _x);
#else
  //-- evaluate constrained problem and buffer
  if(_x!=x) {
    x=_x;
    P->evaluate(phi_x, J_x, x);
    P->getFHessian(H_x, x);
  } else { //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
  }

  CHECK(x.N, "zero-dim optimization variables!");
  if(!isSparseMatrix(J_x)) {
    CHECK_EQ(phi_x.N, J_x.d0, "Jacobian size inconsistent");
  }
  CHECK_EQ(phi_x.N, P->featureTypes.N, "termType array size inconsistent");

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(phi_x.N);
  if(phi_x.N) I_lambda_x = false;
  if(!useLB) for(uint i=0; i<phi_x.N; i++) {
      if(P->featureTypes.p[i]==OT_ineq) I_lambda_x.p[i] = (phi_x.p[i]>0. || (lambda.N && lambda.p[i]>0.));
    }

  double L=0.; //L value
  for(uint i=0; i<phi_x.N; i++) {
    ObjectiveType ot = P->featureTypes.p[i];
    if(ot==OT_f) L += phi_x.p[i];                                                  // direct cost term
    if(ot==OT_sos) L += rai::sqr(phi_x.p[i]);                                      // sumOfSqr term
    if(useLB    && ot==OT_ineq) { if(phi_x.p[i]>0.) return NAN;  L -= muLB * ::log(-phi_x.p[i]); }  //log barrier, check feasibility
    if(!useLB   && ot==OT_ineq && I_lambda_x.p[i]) L += gpenalty(phi_x.p[i]);      //g-penalty
    if(ot==OT_ineqP && phi_x.p[i]>0.) L += gpenalty(phi_x.p[i]);                  //g-penalty
    if(lambda.N && ot==OT_ineq  && lambda.p[i]>0.) L += lambda.p[i] * phi_x.p[i];   //g-lagrange terms
    if(ot==OT_ineqB) { if(phi_x.p[i]>0.) return NAN;  L -= muLB * ::log(-phi_x.p[i]); }              //log barrier, check feasibility
    if(lambda.N && ot==OT_ineqB && lambda.p[i]>0.) L += lambda.p[i] * phi_x.p[i];  //g-lagrange terms
    if(ot==OT_eq) L += hpenalty(phi_x.p[i]);                                       //h-penalty
    if(lambda.N && ot==OT_eq) L += lambda.p[i] * phi_x.p[i];                       //h-lagrange terms
  }

  if(!!dL) { //L gradient
    arr coeff=zeros(phi_x.N);
    for(uint i=0; i<phi_x.N; i++) {
      ObjectiveType ot = P->featureTypes.p[i];
      if(ot==OT_f) coeff.p[i] += 1.;                                                        // direct cost term
      if(ot==OT_sos) coeff.p[i] += 2.* phi_x.p[i];                                          // sumOfSqr terms
      if(useLB    && ot==OT_ineq) coeff.p[i] -= (muLB/phi_x.p[i]);                          //log barrier, check feasibility
      if(!useLB   && ot==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += gpenalty_d(phi_x.p[i]);  //g-penalty
      if(ot==OT_ineqP && phi_x.p[i]>0.) coeff.p[i] += gpenalty_d(phi_x.p[i]);               //g-penalty
      if(lambda.N && ot==OT_ineq && lambda.p[i]>0.) coeff.p[i] += lambda.p[i];              //g-lagrange terms
      if(ot==OT_ineqB) coeff.p[i] -= (muLB/phi_x.p[i]);                                     //log barrier, check feasibility
      if(lambda.N && ot==OT_ineqB && lambda.p[i]>0.) coeff.p[i] += lambda.p[i];             //g-lagrange terms
      if(ot==OT_eq) coeff.p[i] += hpenalty_d(phi_x.p[i]);                                   //h-penalty
      if(lambda.N && ot==OT_eq) coeff.p[i] += lambda.p[i];                                  //h-lagrange terms
    }
    dL = comp_At_x(J_x, coeff);
    dL.reshape(x.N);
  }

  if(!!HL) { //L hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi_x.N);
    for(uint i=0; i<phi_x.N; i++) {
      ObjectiveType ot = P->featureTypes.p[i];
      //if(ot==OT_f) { if(fterm!=-1) HALT("There must only be 1 f-term (in the current implementation)");  fterm=i; }
      if(ot==OT_sos) coeff.p[i] += 2.;                                                        // sumOfSqr terms
      if(useLB    && ot==OT_ineq) coeff.p[i] += (muLB/rai::sqr(phi_x.p[i]));                  //log barrier, check feasibility
      if(!useLB   && ot==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += gpenalty_dd(phi_x.p[i]);   //g-penalty
      if(ot==OT_ineqP && phi_x.p[i]>0.) coeff.p[i] += gpenalty_dd(phi_x.p[i]);                //g-penalty
      if(ot==OT_ineqB) coeff.p[i] += (muLB/rai::sqr(phi_x.p[i]));                             //log barrier, check feasibility
      if(ot==OT_eq) coeff.p[i] += hpenalty_dd(phi_x.p[i]);                                    //h-penalty
    }
    arr tmp = J_x;
    if(!isSpecial(tmp)) {
      for(uint i=0; i<phi_x.N; i++) tmp[i] *= sqrt(coeff.p[i]);
    } else if(isSparseMatrix(tmp)) {
      arr sqrtCoeff = sqrt(coeff);
      tmp.sparse().rowWiseMult(sqrtCoeff);
    } else if(isRowShifted(tmp)) {
      arr sqrtCoeff = sqrt(coeff);
      tmp.rowShifted().rowWiseMult(sqrtCoeff);
    }
    HL = comp_At_A(tmp); //Gauss-Newton type!

    if(H_x.N) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      HL += H_x;
    }

    if(!HL.special) HL.reshape(x.N, x.N);
  }

  if(logFile)(*logFile) <<"{ lagrangianQuery: True, errors: [" <<get_costs() <<", " <<get_sumOfGviolations() <<", " <<get_sumOfHviolations() <<"] }," <<endl;

  return L;
#endif
}

arr LagrangianProblem::get_totalFeatures() {
  arr feat(OT_ineqP+1);
  feat.setZero();
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes.elem(i)==OT_f) feat.elem(OT_f) += phi_x.elem(i);
    else if(P->featureTypes.elem(i)==OT_sos) feat.elem(OT_sos) += rai::sqr(phi_x.elem(i));
    else if(P->featureTypes.elem(i)==OT_ineq && phi_x.elem(i)>0.) feat.elem(OT_ineq) += phi_x.elem(i);
    else if(P->featureTypes.elem(i)==OT_eq) feat.elem(OT_eq) += fabs(phi_x.elem(i));
    else if(P->featureTypes.elem(i)==OT_ineqB && phi_x.elem(i)>0.) feat.elem(OT_ineqB) += phi_x.elem(i);
    else if(P->featureTypes.elem(i)==OT_ineqP && phi_x.elem(i)>0.) feat.elem(OT_ineqP) += phi_x.elem(i);
  }
  return feat;
}

rai::Graph LagrangianProblem::reportGradients(const StringA& featureNames) {
  //-- build feature -> entry map based on names
  struct Entry { rai::String name; double grad=0.; double err=0.; ObjectiveType ot=OT_none; };
  intA idx2Entry(phi_x.N);
  rai::Array<Entry> entries;
  if(featureNames.N) {
    CHECK_EQ(featureNames.N, phi_x.N, "");
    idx2Entry = -1;

    for(uint i=0; i<featureNames.N; i++) {
      rai::String& s = featureNames.elem(i);
      if(!s.N) continue;
      for(uint j=0; j<entries.N; j++) {
        if(entries(j).name == s) {
          idx2Entry(i) = j;
          break;
        }
      }
      if(idx2Entry(i)==-1) {
        idx2Entry(i) = entries.N;
        entries.append({ s, 0., 0.});
      }
    }
  } else {
    idx2Entry.setStraightPerm(phi_x.N);
    entries.resize(phi_x.N);
    for(uint i=0; i<phi_x.N; i++) entries(i) = { STRING("phi" <<i <<"_" <<rai::Enum<ObjectiveType>(P->featureTypes(i))), 0., 0.};
  }

  //-- collect all gradients
  J_x.sparse().setupRowsCols();
  for(uint i=0; i<phi_x.N; i++) {
    int j = idx2Entry(i);
    if(j>=0) {
      double l=0., g=0.;
      double r = sqrt(sumOfSqr(J_x.sparse().getSparseRow(i)));
      double lambda_i = lambda.N?lambda.p[i]:0.;
      double phi_i = phi_x.p[i];
      ObjectiveType ot = P->featureTypes.p[i];

      if(ot==OT_f)        l += fabs(phi_i);
      else if(ot==OT_sos) l += phi_i*phi_i;
      else if(ot==OT_eq)  l += fabs(phi_i);
      else if(ot==OT_ineq && phi_i>0.)  l += phi_i;
      else if(ot==OT_ineqB && phi_i>0.) l += phi_i;
      else if(ot==OT_ineqP && phi_i>0.) l += phi_i;

      if(ot==OT_f)     g += r;
      if(ot==OT_sos)   g += 2.*fabs(phi_i) * r;
      if(useLB    && ot==OT_ineq)  g += muLB/fabs(phi_i) * r;
      if(!useLB   && ot==OT_ineq)  if(phi_i>0. || lambda_i>0.) g += mu* 2.*fabs(phi_i) * r;
      if(ot==OT_ineqP) if(phi_i>0.) g += mu* 2.*fabs(phi_i) * r;
      if(ot==OT_ineq)  if(lambda.N && lambda_i>0.) g += lambda_i * r;
      if(ot==OT_ineqB) NIY;             //J_setRow( (-muLB/phi_i)* )
      if(ot==OT_ineqB) NIY;             //{ if(lambda.N && lambda_i>0.) J_setRow( lambda_i* ) else nphi++; }
      if(ot==OT_eq)    g += mu * 2.*fabs(phi_i) * r;
      if(ot==OT_eq)    if(lambda.N) g += fabs(lambda_i) * r;

      Entry& e = entries(j);
      e.grad += g;
      e.err += l;
      if(e.ot==OT_none) e.ot = ot; //else CHECK_EQ(ot, e.ot, "");
    }
  }

  entries.sort([](const Entry& a, const Entry& b) -> bool{ return a.grad > b.grad; });

  rai::Graph G;
  for(Entry& e: entries) {
    if(!e.err) continue;
    rai::Graph& g = G.addSubgraph(e.name);
    g.add<double>("err", e.err);
    g.add<double>("grad", e.grad);
    g.add<rai::String>("type", rai::Enum<ObjectiveType>(e.ot).name());
  }
  return G;
}

void LagrangianProblem::reportMatrix(std::ostream& os) {
  arr C = unpack(J_x.sparse().A_At());
//  arr U,s,V;
//  svd(U,s,V,M);
//  cout <<"SVD: U=" <<U <<"s=" <<s <<"V=" <<V <<endl;
  arr sig = sqrt(getDiag(C));
  struct Entry { uint i, j; double c=0.; };
  rai::Array<Entry> entries;
  for(uint i=0; i<C.d0; i++) for(uint j=i+1; j<C.d1; j++) {
      C(i, j) /= (sig(i) *sig(j));
      if(P->featureTypes.p[i]>OT_sos && P->featureTypes.p[j]>OT_sos && C(i, j)<0.) {
        entries.append(Entry{i, j, C(i, j)});
      }
    }

  entries.sort([](const Entry& a, const Entry& b) -> bool{ return a.c < b.c; });

  os <<"== Lagrange constraint conflicts: (c=-1 is maximal conflict) \n";
  for(Entry& e: entries) {
    os <<"  { " <<" c: " <<e.c <<" (" <<e.i <<',' <<e.j <<") }" <<endl;
  }

}

double LagrangianProblem::get_cost_f() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes.p[i]==OT_f) S += phi_x(i);
  }
  return S;
}

double LagrangianProblem::get_cost_sos() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_sos) S += rai::sqr(phi_x(i));
  }
  return S;
}

double LagrangianProblem::get_costs() { return get_cost_f() + get_cost_sos(); }

double LagrangianProblem::get_sumOfGviolations() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_ineq && phi_x(i)>0.) {
      S += phi_x(i);
//      cout <<"g violation" <<i << phi_x(i) <<' '<<(lambda.N?lambda(i):-1.) <<endl;
    }
  }
  return S;
}

double LagrangianProblem::get_sumOfHviolations() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_eq) S += fabs(phi_x(i));
  }
  return S;
}

uint LagrangianProblem::get_dimOfType(const ObjectiveType& ot) {
  uint d=0;
  for(uint i=0; i<P->featureTypes.N; i++) if(P->featureTypes(i)==ot) d++;
  return d;
}

void LagrangianProblem::aulaUpdate(const rai::OptOptions& opt, bool anyTimeVariant, double lambdaStepsize, double* L_x, arr& dL_x, arr& HL_x) {
  if(!lambda.N) lambda=zeros(phi_x.N);

  //-- lambda update
  if(lambdaStepsize>0.) {
    for(uint i=0; i<lambda.N; i++) {
      ObjectiveType ot = P->featureTypes(i);
      if(ot==OT_eq)  lambda(i) += lambdaStepsize * hpenalty_d(phi_x(i));
      if(ot==OT_ineq)  lambda(i) += lambdaStepsize * gpenalty_d(phi_x(i));
      if(ot==OT_ineq && lambda(i)<0.) lambda(i)=0.;  //bound clipping
    }
  }

  if(anyTimeVariant) {
    //collect gradients of active constraints
    arr A;
    rai::RowShifted* Aaux=nullptr, *Jaux=nullptr;
    if(isRowShifted(J_x)) {
      Aaux = &A.rowShifted(); Aaux->resize(0, x.N, J_x.d1);
      Jaux = &J_x.rowShifted();
    }
    //append rows of J_x to A if constraint is active
    for(uint i=0; i<lambda.N; i++) {
      if((P->featureTypes(i)==OT_eq) ||
          (P->featureTypes(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.))) {
        A.append(J_x[i]);
        A.reshape(A.N/J_x.d1, J_x.d1);
        if(isRowShifted(J_x))
          Aaux->rowShift.append(Jaux->rowShift(i));
      }
    }
    if(A.d0>0) {
      arr tmp = comp_A_At(A);
      addDiag(tmp, 1e-6);
      //    if(isRowShifted(J_x)){
      //      CHECK_EQ(castRowShifted(tmp)->symmetric, true, "");
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;
      //    }else{
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
      //    }

      arr AdL = comp_A_x(A, dL_x);
      arr beta;
      bool worked=true;
      try {
        beta = lapack_Ainv_b_sym(tmp, AdL);
      } catch(...) {
        arr sig, eig;
        lapack_EigenDecomp(tmp, sig, eig);
        cout <<endl <<"** hessian inversion failed AulaAnyTimeUpdate: " <<sig <<" -- revert to standard update" <<endl;
        worked=false;
      }

      if(worked) {
        //reinsert zero rows
        for(uint i=0; i<lambda.N; i++) {
          if(!((P->featureTypes(i)==OT_eq) ||
               (P->featureTypes(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.)))) {
            beta.insert(i, 0.);
          }
        }
        lambda -= lambdaStepsize * beta;
        //bound clipping
        for(uint i=0; i<lambda.N; i++) if(lambda(i)<0.) lambda(i)=0.;
      }
    }
  }

  //-- adapt mu as well?
  if(opt.muInc>0.) { mu *= opt.muInc; if(mu>opt.muMax) mu=opt.muMax; }
  if(opt.muLBDec>0. && muLB>1e-8) muLB *= opt.muLBDec;

  if(opt.maxLambda>0.) {
    clip(lambda, -opt.maxLambda, opt.maxLambda);
  }

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || !!dL_x || !!HL_x) {
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

void LagrangianProblem::autoUpdate(const rai::OptOptions& opt, double* L_x, arr& dL_x, arr& HL_x) {
  switch(opt.constrainedMethod) {
//  case squaredPenalty: UCP.mu *= opt.muInc;  break;
    case rai::squaredPenalty: aulaUpdate(opt, false, -1., L_x, dL_x, HL_x);  break;
    case rai::augmentedLag:   aulaUpdate(opt, false, 1., L_x, dL_x, HL_x);  break;
    case rai::anyTimeAula:    aulaUpdate(opt, true,  1., L_x, dL_x, HL_x);  break;
    case rai::logBarrier:     aulaUpdate(opt, false, -1., L_x, dL_x, HL_x);  break;
    case rai::squaredPenaltyFixed: HALT("you should not be here"); break;
    case rai::noMethod: HALT("need to set method before");  break;
  }
}

#if 1
double LagrangianProblem::gpenalty(double g) { return mu*g*g; }
double LagrangianProblem::gpenalty_d(double g) { return 2.*mu*g; }
double LagrangianProblem::gpenalty_dd(double g) { return 2.*mu; }
#else
double LagrangianProblem::gpenalty(double g) { g*=mu;  if(g>0.) return (g*g + g*g*g);  return g*g; }
double LagrangianProblem::gpenalty_d(double g) { g*=mu; if(g>0.) return mu*(2.*g + 3.*g*g);  return 2.*mu*g; }
double LagrangianProblem::gpenalty_dd(double g) { g*=mu; if(g>0.) return mu*mu*(2. + 6.*g);  return 2.*mu*mu; }
#endif

#if 1
double LagrangianProblem::hpenalty(double h) { return mu*h*h;  }
double LagrangianProblem::hpenalty_d(double h) { return 2.*mu*h;  }
double LagrangianProblem::hpenalty_dd(double h) { return 2.*mu;  }
#else
double LagrangianProblem::hpenalty(double h) { h*=nu;  return (h*h + fabs(h)*h*h);  }
double LagrangianProblem::hpenalty_d(double h) { h*=nu; return nu*(2.*h + 3.*fabs(h)*h);  }
double LagrangianProblem::hpenalty_dd(double h) { h*=nu; return nu*nu*(2. + 6.*fabs(h));  }
#endif

