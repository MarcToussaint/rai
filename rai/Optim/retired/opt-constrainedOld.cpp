/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

double LagrangianProblem::lagrangian(arr& dL, arr& HL, const arr& _x) {
  //-- evaluate constrained problem and buffer
  if(_x!=x) {
    x=_x;
    f_x = P(df_x, Hf_x, g_x, Jg_x, h_x, Jh_x, x);
    CHECK(df_x.N==x.N && Hf_x.d0==x.N && g_x.N==Jg_x.d0 && h_x.N==Jh_x.d0, "");
//    && Hf_x.d1==x.N && Jg_x.d1==x.N &&  && Jh_x.d1==x.N //those dimensions might be non-equal due to packing...
  } else { //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
    if(!!dL) CHECK(df_x.N && g_x.N==Jg_x.d0 && h_x.N==Jh_x.d0, "");
    if(!!HL) CHECK(Hf_x.N && g_x.N==Jg_x.d0 && h_x.N==Jh_x.d0, "");
  }

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(g_x.N);
  if(mu)       for(uint i=0; i<g_x.N; i++) I_lambda_x(i) = (g_x(i)>0. || (lambda.N && lambda(i)>0.));

  //L value
  double L=f_x;
  if(muLB)     for(uint i=0; i<g_x.N; i++) { if(g_x(i)>0.) return NAN;  L -= muLB * ::log(-g_x(i)); } //log barrier, check feasibility
  if(mu)       for(uint i=0; i<g_x.N; i++) if(I_lambda_x(i)) L += mu * rai::sqr(g_x(i)); //g-penalty
  if(lambda.N) for(uint i=0; i<g_x.N; i++) if(lambda(i)>0.) L += lambda(i) * g_x(i);    //g-lagrange terms
  if(nu)       for(uint i=0; i<h_x.N; i++) L += nu * rai::sqr(h_x(i));                   //h-penalty
  if(kappa.N)  for(uint i=0; i<h_x.N; i++) L += kappa(i) * h_x(i);                      //h-lagrange terms

  if(!!dL) { //L gradient
    dL=df_x;
    //-- g-terms
    if(g_x.N) {
      arr coeff=zeros(g_x.N);
      if(muLB)     for(uint i=0; i<g_x.N; i++) coeff(i) -= (muLB/g_x(i));                 //log barrier
      if(mu)       for(uint i=0; i<g_x.N; i++) if(I_lambda_x(i)) coeff(i) += 2.*mu*g_x(i); //g-penalty
      if(lambda.N) for(uint i=0; i<g_x.N; i++) if(lambda(i)>0.) coeff(i) += lambda(i);    //g-lagrange terms
      dL += comp_At_x(Jg_x, coeff);
    }
    //-- h-terms
    if(h_x.N) {
      arr coeff=zeros(h_x.N);
      if(nu)       for(uint i=0; i<h_x.N; i++) coeff(i) += 2.*nu*h_x(i);                  //h-penalty
      if(kappa.N)  for(uint i=0; i<h_x.N; i++) coeff(i) += kappa(i);                      //h-lagrange terms
      dL += comp_At_x(Jh_x, coeff);
    }

    dL.reshape(x.N);
  }

  if(!!HL) { //L hessian
    // the 2.*Jg_x^T Jg_x terms are considered as in Gauss-Newton type; no real Hg used
    HL=Hf_x;
    //-- g-terms
    if(g_x.N) {
      arr coeff=zeros(g_x.N);
      if(muLB)     for(uint i=0; i<g_x.N; i++) coeff(i) += (muLB/rai::sqr(g_x(i))); //log barrier
      if(mu)       for(uint i=0; i<g_x.N; i++) if(I_lambda_x(i)) coeff(i) += 2.*mu; //g-penalty
      arr tmp = Jg_x;
      for(uint i=0; i<g_x.N; i++) tmp[i]() *= sqrt(coeff(i));
      HL += comp_At_A(tmp); //Gauss-Newton type!
    }
    //-- h-terms
    if(h_x.N) {
      arr coeff=zeros(h_x.N);
      if(nu)       for(uint i=0; i<h_x.N; i++) coeff(i) += 2.*nu;                  //h-penalty
      arr tmp = Jh_x;
      for(uint i=0; i<h_x.N; i++) tmp[i]() *= sqrt(coeff(i));
      HL += comp_At_A(tmp); //Gauss-Newton type!
    }

    if(!HL.special) HL.reshape(x.N, x.N);
  }

  return L;
}

void LagrangianProblem::aulaUpdate(double lambdaStepsize, double muInc, double* L_x, arr& dL_x, arr& HL_x) {
  if(!lambda.N) lambda=zeros(g_x.N);
  if(!kappa .N) kappa =zeros(h_x.N);

  //-- lambda update
  lambda += (lambdaStepsize * 2.*mu)*g_x;
  //bound clipping
  for(uint i=0; i<lambda.N; i++) if(lambda(i)<0.) lambda(i)=0.;
  //-- kappa update
  kappa += (lambdaStepsize * 2.*nu)*h_x;

  //-- adapt mu as well?
  if(muInc>1.) mu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x) {
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

bool LagrangianProblem::anyTimeAulaUpdateStopCriterion(const arr& dL_x) {
//  cout <<"checking gradient norms: dL_x =" <<length(dL_x) <<" df_x=" <<length(df_x) <<endl;
  if(length(dL_x) < .1 * length(df_x)) return true;
  return false;
}

void LagrangianProblem::anyTimeAulaUpdate(double lambdaStepsize, double muInc, double* L_x, arr& dL_x, arr& HL_x) {
  if(!lambda.N) lambda=zeros(g_x.N);
  if(!kappa .N) kappa =zeros(h_x.N);

  //-- kappa update
  kappa += (lambdaStepsize * 2.*nu)*h_x;

  //-- lambda update
  lambda += (lambdaStepsize * 2.*mu)*g_x;
  //bound clipping
  for(uint i=0; i<g_x.N; i++) if(lambda(i)<0.) lambda(i)=0.;

  //collect gradients of active constraints
  arr A;
  RowShifted* Aaux, *Jgaux;
  if(Jg_x.special==arr::RowShiftedST) {
    Aaux = makeRowShifted(A, 0, Jg_x.d1, x.N);
    Jgaux = castRowShifted(Jg_x);
  }
  //append rows of Jg_x to A if constraint is active
  for(uint i=0; i<g_x.N; i++) if(g_x(i)>0. || lambda(i)>0.) {
      A.append(Jg_x[i]);
      A.reshape(A.N/Jg_x.d1, Jg_x.d1);
      if(Jg_x.special==arr::RowShiftedST)
        Aaux->rowShift.append(Jgaux->rowShift(i));
    }
  if(A.d0>0) {
    arr tmp = comp_A_At(A);
    if(Jg_x.special==arr::RowShiftedST) {
      CHECK_EQ(castRowShifted(tmp)->symmetric, true, "");
      for(uint i=0; i<tmp.d0; i++) tmp(i, 0) += 1e-6;
    } else {
      for(uint i=0; i<tmp.d0; i++) tmp(i, i) += 1e-6;
    }

    arr AdL = comp_A_x(A, dL_x);
    arr beta;
    beta = lapack_Ainv_b_sym(tmp, AdL);
    //reinsert zero rows
    for(uint i=0; i<g_x.N; i++) if(!(g_x(i)>0. || lambda(i)>0.)) {
        beta.insert(i, 0.);
      }
    lambda -= lambdaStepsize * beta;
    //bound clipping
    for(uint i=0; i<g_x.N; i++) if(lambda(i)<0.) lambda(i)=0.;
  }

  //-- adapt mu as well?
  if(muInc>1.) mu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x) {
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

uint optConstrained(arr& x, arr& dual, const ConstrainedProblem& P, OptOptions opt) {

  ofstream fil(STRING("z."<<MethodName[opt.constrainedMethod]));

  LagrangianProblem UCP(P);

  //uint stopTolInc;

  //switch on penalty terms
  UCP.nu=1.;
  switch(opt.constrainedMethod) {
    case squaredPenalty: UCP.mu=1.;  break;
    case augmentedLag:   UCP.mu=1.;  break;
    case anyTimeAula:    UCP.mu=1.;  /*stopTolInc=rai::getParameter("/opt/optConstrained/anyTimeAulaStopTolInc",2.);*/ break;
    case logBarrier:     UCP.muLB=.1;  break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;

  OptNewton newton(x, UCP.Lag, opt);

  for(uint k=0;; k++) {
    fil <<k <<' ' <<newton.evals <<' ' <<UCP.f_x <<' ' <<sum(elemWiseMax(UCP.g_x, zeros(UCP.g_x.N, 1))) <<endl;

    if(opt.verbose>0) {
      cout <<"** optConstr. it=" <<k
           <<" mu=" <<UCP.mu <<" nu=" <<UCP.nu <<" muLB=" <<UCP.muLB;
      if(x.N<5) cout <<" \tlambda=" <<UCP.lambda <<" \tkappa=" <<UCP.kappa /*<<" \tg=" <<UCP.g_x <<" \th=" <<UCP.h_x*/;
      cout <<endl;
    }

    arr x_old = x;
    if(opt.constrainedMethod==anyTimeAula) {
      //decide yourselve on when to stop iterating Newton steps
      double stopTol = newton.o.stopTolerance;
      newton.o.stopTolerance*=10.;
#if 0
      for(uint l=0; l<20; l++) {
        OptNewton::StopCriterion res = newton.step();
        if(res>=OptNewton::stopCrit1) break;
//        if(UCP.anyTimeAulaUpdateStopCriterion(newton.gx)) break;
        newton.o.stopTolerance*=stopTolInc;
      }
#else
      newton.run();
#endif
      newton.o.stopTolerance = stopTol;
    } else {
      //use standard 'run()' to iterate Newton steps
      double stopTol = newton.o.stopTolerance;
      newton.o.stopTolerance*=10.;
      newton.run();
      newton.o.stopTolerance = stopTol;
    }

    if(opt.verbose>0) {
      cout <<k <<' ' <<newton.evals <<" f(x)=" <<UCP.f_x
           <<" \tg_compl=" <<sum(elemWiseMax(UCP.g_x, zeros(UCP.g_x.N)))
           <<" \th_compl=" <<sumOfAbs(UCP.h_x);
      if(x.N<5) cout <<" \tx=" <<x;
      cout <<endl;
    }

    //stopping criteron
    if(k>10 && absMax(x_old-x)<opt.stopTolerance) {
      if(opt.verbose>0) cout << " --- optConstrained StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      break;
    }

    //upate unconstraint problem parameters
    switch(opt.constrainedMethod) {
      case squaredPenalty: UCP.mu *= 10;  break;
      case augmentedLag:   UCP.aulaUpdate(1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case anyTimeAula:    UCP.anyTimeAulaUpdate(1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case logBarrier:     UCP.muLB /= 2;  break;
      case noMethod: HALT("need to set method before");  break;
    }

  }
  fil.close();
  if(!!dual) dual=UCP.lambda;

  return newton.evals;
}

bool checkAllGradients(const ConstrainedProblem& P, const arr& x, double tolerance) {
  ScalarFunction F = [&P](arr& df, arr& Hf, const arr& x) {
    return P(df, Hf, NoArr, NoArr, NoArr, NoArr, x);
  };
  VectorFunction G = [&P](arr& g, arr& Jg, const arr& x) {
    return P(NoArr, NoArr, g, Jg, NoArr, NoArr, x);
  };
  VectorFunction H = [&P](arr& h, arr& Jh, const arr& x) {
    return P(NoArr, NoArr, NoArr, NoArr, h, Jh, x);
  };

  bool good=true;
  cout <<"f-gradient: "; good &= checkGradient(F, x, tolerance);
  cout <<"f-hessian:  "; good &= checkHessian(F, x, tolerance);
  cout <<"g-jacobian: "; good &= checkJacobian(G, x, tolerance);
  cout <<"h-jacobian: "; good &= checkJacobian(H, x, tolerance);
  return good;
}
