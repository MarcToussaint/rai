/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "primalDual.h"

PrimalDualProblem::PrimalDualProblem(const arr& x, MathematicalProgram& P, OptOptions opt, arr& lambdaInit)
  : L(P, opt, lambdaInit), mu(opt.muLBInit) {

  L.mu = L.nu = L.muLB = 0.;

  L.lagrangian(NoArr, NoArr, x);
//  cout <<"x=" <<x <<endl <<"L=" <<Lval <<endl;

  n_ineq=0;
  n_eq=0;
  for(uint i=0; i<L.phi_x.N; i++) if(L.tt_x.p[i]==OT_eq) n_eq++;
  for(uint i=0; i<L.phi_x.N; i++) if(L.tt_x.p[i]==OT_ineq) n_ineq++;
  x_lambda = x;
  if(n_eq) x_lambda.append(zeros(n_eq));
  x_lambda.append(ones(n_ineq));

  ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
    return this->primalDual(dL, HL, x);
  });

}

double PrimalDualProblem::primalDual(arr& r, arr& R, const arr& x_lambda) {

  //copy the latter part of x into lambdas for the inequalities
  uint n = L.x.N; //x_lambda.N - (n_eq+n_ineq);
  const arr x = x_lambda({0, n-1});
  if(!L.lambda.N) L.lambda = zeros(L.phi_x.N);
  for(uint i=0; i<L.phi_x.N; i++) if(L.tt_x.p[i]==OT_eq) L.lambda(i) = x_lambda(n++);
  for(uint i=0; i<L.phi_x.N; i++) if(L.tt_x.p[i]==OT_ineq) L.lambda(i) = x_lambda(n++);
  CHECK_EQ(n, x_lambda.N, "");
  CHECK_EQ(x.N+n_eq+n_ineq, x_lambda.N, "");

  L.mu = 0.; //1e-2;
  L.nu = L.muLB = 0.;

  arr dL, HL;
  L.lagrangian(dL, HL, x);
//  cout <<"x=" <<x <<endl <<"lambda=" <<L.lambda <<endl <<"L=" <<Lval <<endl;
  if(!L.lambda.N) L.lambda = zeros(L.phi_x.N);

  primalFeasible=true;
  dualityMeasure=0.;
  for(uint i=0; i<L.phi_x.N; i++) {
    if(L.tt_x.p[i]==OT_ineq) {
      if(L.phi_x.p[i] > 0.) { primalFeasible=false; /*break;*/ }
//      if(L.phi_x.p[i] > 0.) dualityMeasure += ::fabs(L.phi_x.p[i]); else
      HALT("Isn't this wrong for positive phi? Should we exclude those from the duality measure? (Because for those we want lambda positive!!)");
      dualityMeasure += ::fabs(L.lambda.p[i] * L.phi_x.p[i]);
    }
    if(L.tt_x.p[i]==OT_eq) {
//      dualityMeasure += ::fabs(L.phi_x.p[i]);
    }
  }
  if(n_ineq) {
    dualityMeasure /= n_ineq;
  }

//  updateMu(primalFeasible);
//  cout <<" \tmu=\t" <<mu <<" primalFeasible=" <<primalFeasible <<std::flush;

  //-- equation system
  if(!!r) {
    // 1st KKT: dL
    r.resize(x_lambda.N).setZero();
    r({0, x.N-1}) = dL;

    // 2nd KKT: primal feasibility of equalities h
    n=x.N;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_eq) r(n++) = L.phi_x.p[i];
    }
    CHECK_EQ(n, x.N+n_eq, "");

    // 4th KKT
    n=x.N+n_eq;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_ineq) {
//        if(L.phi_x.p[i] > 0.) r(n++) = - L.phi_x.p[i] - mu; else
        r(n++) = - L.lambda.p[i] * L.phi_x.p[i] - mu;
      }
    }
    CHECK_EQ(n, r.N, "");
  } else {
    HALT("it does not make sense to call this without r as return value! r is not the gradient of the sumOfSqr(r) ! ")
  }

  //-- Jacobian
  if(!!R) {
    bool sparse=isSparseMatrix(HL);
    rai::SparseMatrix* Rsparse=0;
    rai::SparseMatrix* LJx_sparse=0;

    if(!sparse) {
      R.resize(r.N, r.N).setZero();
      // top-left: 1st KKT: HL
      R.setMatrixBlock(HL, 0, 0);
    } else {
      CHECK(isSparseMatrix(L.J_x), "");
      R = HL;
      Rsparse = &R.sparse();
      Rsparse->reshape(r.N, r.N);
      LJx_sparse = &L.J_x.sparse();
      LJx_sparse->setupRowsCols();
    }

    // top-mid: transposed \del h
    n=x.N;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_eq) {
        if(!sparse) {
          for(uint j=0; j<x.N; j++) R(j, n) = L.J_x(i, j);
        } else {
          uintA& row = LJx_sparse->rows(i);
          for(uint j=0; j<row.d0; j++) {
            Rsparse->addEntry(row(j, 0), n) = LJx_sparse->Z.elem(row(j, 1));
          }
        }
        n++;
      }
    }
    CHECK_EQ(n, x.N+n_eq, "");

    // top-right: transposed \del g
    n=x.N+n_eq;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_ineq) {
        for(uint j=0; j<x.N; j++) {
          if(!sparse) R(j, n) = L.J_x(i, j);
          else Rsparse->addEntry(j, n) = L.J_x(i, j);
        }
        n++;
      }
    }
    CHECK_EQ(n, x.N+n_eq+n_ineq, "");

    // mid-mid and mid-right are zero

    // mid-left:
    n=x.N;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_eq) {
        if(!sparse) {
          for(uint j=0; j<x.N; j++) R(n, j) = L.J_x(i, j);
        } else {
          uintA& row = LJx_sparse->rows(i);
          for(uint j=0; j<row.d0; j++) {
            Rsparse->addEntry(n, row(j, 0)) = LJx_sparse->Z.elem(row(j, 1));
          }
        }
        n++;
      }
    }
    CHECK_EQ(n, x.N+n_eq, "");

    // bottom-left:
    n=x.N+n_eq;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_ineq) {
        for(uint j=0; j<x.N; j++) {
//          if(L.phi_x.p[i] > 0.) R(n,j) = -L.J_x(i, j); else
          if(!sparse) R(n, j) = -L.lambda(i) * L.J_x(i, j);
          else Rsparse->addEntry(n, j) = -L.lambda(i) * L.J_x(i, j);
        }
        n++;
      }
    }
    CHECK_EQ(n, x.N+n_eq+n_ineq, "");

    // bottom-right:
    n=x.N+n_eq;
    for(uint i=0; i<L.phi_x.N; i++) {
      if(L.tt_x.p[i]==OT_ineq) {
//          if(L.phi_x.p[i] > 0.) {} else
        if(!sparse) R(n, n) = - L.phi_x(i);
        else Rsparse->addEntry(n, n) = - L.phi_x(i);
        n++;
      }
    }
    CHECK_EQ(n, x.N+n_eq+n_ineq, "");
  }

//  if(!primalFeasible) return NAN;
  return sumOfSqr(r);
}

void PrimalDualProblem::updateMu() {
#if 1
  mu = .5*dualityMeasure;
#else
  double newMu = .5*dualityMeasure;
  if(newMu < mu) mu *= .5;
#endif
}

//==============================================================================

OptPrimalDual::OptPrimalDual(arr& x, arr& dual, MathematicalProgram& P, int verbose, OptOptions opt)
  : x(x), PD(x, P, opt, dual), newton(PD.x_lambda, PD, opt), opt(opt) {

  if(verbose>=0) opt.verbose=verbose;
  newton.options.verbose = rai::MAX(opt.verbose-1, 0);

  newton.rootFinding = true;
  newton.bounds_lo.resize(newton.x.N).setZero();
  newton.bounds_up.resize(newton.x.N) = -1.;
  for(uint i=x.N+PD.n_eq; i<newton.x.N; i++) newton.bounds_up(i) = 1e10;

  if(opt.verbose>0) cout <<"***** OptPrimalDual" <<endl;
}

uint OptPrimalDual::run(uint maxIt) {
  if(fil)(*fil) <<"constr " <<its <<' ' <<newton.evals <<' ' <<PD.L.get_costs() <<' ' <<PD.L.get_sumOfGviolations() <<' ' <<PD.L.get_sumOfHviolations() <<endl;
  newton.logFile = fil;

  //newton loop (cp newton.run() )
  newton.numTinyFSteps=0;
  for(uint i=0; i<maxIt; i++) {
    newton.step();

    if(PD.primalFeasible) {
      if(opt.stopGTolerance<0.
          || PD.L.get_sumOfGviolations() + PD.L.get_sumOfHviolations() < opt.stopGTolerance) {
        if(newton.stopCriterion==newton.stopStepFailed) continue;
        if(newton.stopCriterion>=newton.stopDeltaConverge) break;
      }
    }

    PD.updateMu();

    newton.reinit(newton.x); //potential to optimize! don't recompute everything naively

    x = newton.x({0, x.N-1});

    if(opt.verbose>0) {
      cout <<"** optPrimalDual it=" <<its
           <<' ' <<newton.evals
           <<" mu=" <<PD.mu
           <<" f(x)=" <<PD.L.get_costs()
           <<" \tg_compl=" <<PD.L.get_sumOfGviolations()
           <<" \th_compl=" <<PD.L.get_sumOfHviolations();
      if(x.N<5) cout <<" \tx=" <<x;
      cout <<endl;
    }
  }

  return newton.evals;
}

OptPrimalDual::~OptPrimalDual() {
}

