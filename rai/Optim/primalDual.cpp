#include "primalDual.h"

PrimalDualProblem::PrimalDualProblem(const arr &x, ConstrainedProblem &P, OptOptions opt, arr &lambdaInit)
  : L(P, opt, lambdaInit), mu(opt.muLBInit){

  L.lagrangian(NoArr, NoArr, x);

  uint n_ineq=0;
  for(uint i=0;i<L.phi_x.N;i++) if(L.tt_x.p[i]==OT_ineq) n_ineq++;
  x_lambda = x;
  x_lambda.append( ones(n_ineq) );


  ScalarFunction::operator=( [this](arr& dL, arr& HL, const arr& x) -> double {
    return this->primalDual(dL, HL, x);
  } );

}

double PrimalDualProblem::primalDual(arr &r, arr &R, const arr &x_lambda){

  //copy the latter part of x into lambdas for the inequalities
  uint n = L.x.N;
  const arr x = x_lambda({0,n-1});
  if(!L.lambda.N) L.lambda = zeros(L.phi_x.N);
  for(uint i=0;i<L.phi_x.N;i++) if(L.tt_x.p[i]==OT_ineq) L.lambda(i) = x_lambda(n++);
  CHECK_EQ(n, x_lambda.N, "");

  L.mu = L.nu = L.muLB = 0.;

  arr dL, HL;
  L.lagrangian(dL, HL, x);
  if(!L.lambda.N) L.lambda = zeros(L.phi_x.N);

  bool primalFeasible=true;
  double dualityMeasure=0.;
  uint n_ineq=0;
  for(uint i=0;i<L.phi_x.N;i++){
    if(L.tt_x.p[i]==OT_ineq    ){
      if(L.phi_x.p[i] > 0.){ primalFeasible=false; break; }
      n_ineq++;
      dualityMeasure += L.lambda.p[i] * L.phi_x.p[i];
    }
  }
  dualityMeasure /= n_ineq;

  mu = -.5*dualityMeasure;

  //-- equation system
  if(&r){
    // 1st KKT: dL
    r.resize(x_lambda.N).setZero();
    r({0,x.N-1}) = dL;

    // 4th KKT
    n=x.N;
    for(uint i=0;i<L.phi_x.N;i++){
      if(L.tt_x.p[i]==OT_ineq    ) r(n++) = - L.lambda.p[i] * L.phi_x.p[i] - mu;
    }
    CHECK_EQ(n, r.N, "");
  }else{
    HALT("it does not make sense to call this without r as return value! r is not the gradient of the sumOfSqr(r) ! ")
  }

  //-- Jacobian
  if(&R){
    R.resize(r.N, r.N).setZero();
    // top-left: 1st KKT: HL
    R.setMatrixBlock(HL, 0, 0);
    // top-right: transposed \del g
    n=x.N;
    for(uint i=0;i<L.phi_x.N;i++){
      if(L.tt_x.p[i]==OT_ineq    ){
        for(uint j=0;j<x.N;j++) R(j,n) = L.J_x(i, j);
        n++;
      }
    }
    // bottom-left:
    n=x.N;
    for(uint i=0;i<L.phi_x.N;i++){
      if(L.tt_x.p[i]==OT_ineq    ){
        for(uint j=0;j<x.N;j++) R(n,j) = -L.lambda(i) * L.J_x(i, j);
        n++;
      }
    }
    // bottom-right:
    n=x.N;
    for(uint i=0;i<L.phi_x.N;i++){
      if(L.tt_x.p[i]==OT_ineq    ){
        for(uint j=0;j<x.N;j++) R(n,n) = - L.phi_x(i);
        n++;
      }
    }
  }

  if(!primalFeasible) return NAN;
  return sumOfSqr(r);
}

//==============================================================================

OptPrimalDual::OptPrimalDual(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt)
  : x(x), PD(x, P, opt, dual), newton(PD.x_lambda, PD, opt), opt(opt){

  newton.rootFinding = true;
  newton.bound_lo.resize(newton.x.N).setZero();
  newton.bound_hi.resize(newton.x.N) = -1.;
  for(uint i=x.N; i<newton.x.N; i++) newton.bound_hi(i) = 1e10;

  if(opt.verbose>0) cout <<"***** OptPrimalDual" <<endl;
}

uint OptPrimalDual::run(){
  if(fil) (*fil) <<"constr " <<its <<' ' <<newton.evals <<' ' <<PD.L.get_costs() <<' ' <<PD.L.get_sumOfGviolations() <<' ' <<PD.L.get_sumOfHviolations() <<endl;
  newton.fil = fil;

  newton.run();

  x = newton.x({0,x.N-1});

  if(opt.verbose>0){
    cout <<"** optPrimalDual it=" <<its
         <<' ' <<newton.evals
         <<" mu=" <<PD.mu
         <<" f(x)=" <<PD.L.get_costs()
         <<" \tg_compl=" <<PD.L.get_sumOfGviolations()
         <<" \th_compl=" <<PD.L.get_sumOfHviolations();
    if(x.N<5) cout <<" \tx=" <<x;
    cout <<endl;
  }

  return newton.evals;
}

OptPrimalDual::~OptPrimalDual(){
}


