/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//this is the old implementation of optNewton as single function
//the class should be 100% equivalent

uint optNewton(arr& x, ScalarFunction& f,  OptOptions o, arr* addRegularizer, double* fx_user, arr* gx_user, arr* Hx_user) {
  double alpha = 1.;
  double lambda = o.damping;
  double fx, fy;
  arr gx, Hx, gy, Hy;
  arr Delta, y;
  uint evals=0;

  //compute initial costs
  if(fx_user && gx_user && Hx_user) { //pre-condition!: assumes S is correctly evaluated at x!!
    if(sanityCheck) {
      fx = f.fs(gx, Hx, x);
      CHECK(fabs(fx-*fx_user) <1e-6, "");
      CHECK((maxDiff(gx, *gx_user) + maxDiff(Hx, *Hx_user) + fabs(fx-*fx_user))<1e-6, "");
    }
    fx = *fx_user;
    gx = *gx_user;
    Hx = *Hx_user;
  } else {
    fx = f.fs(gx, Hx, x);  evals++;
    if(addRegularizer)  fx += scalarProduct(x, (*addRegularizer)*vectorShaped(x));
  }

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" lambda=" <<lambda <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  for(uint it=1;; it++) { //iterations and lambda adaptation loop

    if(o.verbose>1) cout <<"optNewton it=" <<std::setw(3) <<it << " \tlambd=" <<std::setprecision(3) <<lambda <<flush;

    //compute Delta
    //RAI_MSG("\nx=" <<x <<"\ngx=" <<gx <<"\nHx=" <<Hx);
    arr R=Hx;
    if(lambda) { //Levenberg Marquardt damping
      if(R.special==arr::RowShiftedST) for(uint i=0; i<R.d0; i++) R(i, 0) += lambda; //(R(i,0) is the diagonal in the packed matrix!!)
      else for(uint i=0; i<R.d0; i++) R(i, i) += lambda;
    }
    if(addRegularizer) {
      if(R.special==arr::RowShiftedST) R = unpack(R);
      //      cout <<*addRegularizer <<R <<endl;
      Delta = lapack_Ainv_b_sym(R + (*addRegularizer), -(gx+(*addRegularizer)*vectorShaped(x)));
    } else {
      Delta = lapack_Ainv_b_sym(R, -gx);
    }
    if(o.maxStep>0. && absMax(Delta)>o.maxStep)  Delta *= o.maxStep/absMax(Delta);
    if(o.verbose>1) cout <<" \t|Delta|=" <<absMax(Delta) <<flush;

    //lazy stopping criterion: stop without any update
    if(lambda<2. && absMax(Delta)<1e-1*o.stopTolerance) {
      if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
      break;
    }

    for(;;) { //stepsize adaptation loop -- doesn't iterate for useDamping option
      y = x + alpha*Delta;
      fy = f.fs(gy, Hy, y);  evals++;
      if(addRegularizer) fy += scalarProduct(y, (*addRegularizer)*vectorShaped(y));
      if(o.verbose>2) cout <<" \tprobing y=" <<y;
      if(o.verbose>1) cout <<" \tevals=" <<evals <<" \talpha=" <<alpha <<" \tf(y)=" <<fy <<flush;
      //CHECK_EQ(fy,fy, "cost seems to be NAN: ly=" <<fy);
      if(fy==fy && fy <= fx) { //fy==fy is for NAN?
        if(o.verbose>1) cout <<" - ACCEPT" <<endl;
        //adopt new point and adapt stepsize|damping
        x = y;
        fx = fy;
        gx = gy;
        Hx = Hy;
        //          if(alpha>.9) lambda = .5*lambda;
        //        alpha = pow(alpha, 0.5);
        lambda = lambda*o.dampingDec;
        alpha = 1. - (1.-alpha)*(1.-o.stepInc);
        break;
      } else {
        if(o.verbose>1) cout <<" - reject" <<std::endl <<"\t\t\t\t\t\t";
        //reject new points and adapte stepsize|damping
        if(alpha*absMax(Delta)<1e-3*o.stopTolerance || evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
        lambda = lambda*o.dampingInc;
        alpha = alpha*o.stepDec;
        if(o.dampingInc!=1.) break; //we need to recompute Delta
      }
    }

    if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
    if(o.verbose>2) fil <<' ' <<x;
    if(o.verbose>0) fil <<endl;

    //stopping criteria
#define STOPIF(expr) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; break; }
    STOPIF(lambda<2. && absMax(Delta)<o.stopTolerance);
    STOPIF(lambda<2. && alpha*absMax(Delta)<1e-3*o.stopTolerance);
    STOPIF(evals>=o.stopEvals);
    STOPIF(it>=o.stopIters);
#undef STOPIF
  }
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef RAI_MSVC
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", nullptr, true);
#endif
  if(o.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
  if(fx_user) *fx_user = fx;
  if(gx_user) *gx_user = gx;
  if(Hx_user) *Hx_user = Hx;
  return evals;
}
