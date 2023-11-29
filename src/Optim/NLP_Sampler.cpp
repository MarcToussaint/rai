#include "NLP_Sampler.h"

#include <Core/util.h>
#include <math.h>

/*
 * void NLP_Sampler::step_ball(uint steps){
#if 0
  arr g = A*x - b;
  arr g2 = (g%g);
  g2 += 1e-6;
  arr g2_1 = ones(g.N);
  g2_1 /= g2;
  arr At = ~A;
  arr H = At*(g2%A);
  arr C;
  lapack_cholesky(C, H);
  arr Cinv = inverse(C);
  //    Cinv.setId();
#endif
  for(uint t=0,s=0;s<steps;t++){
    //      arr y = x + Cinv * (1.*randn(x.N));
    arr y = x + (.2*randn(x.N));
    samples++;
    //      arr y = nlp.getInitializationSample();
    bool inBound = boundCheck(y, nlp.bounds_lo, nlp.bounds_up, 1e-6, false);
    if(inBound && max(g)<=0.){ //accept
      x = y;
      s++;
    }
  }
}
*/

void NLP_Walker::clipBeta(const arr& d, const arr& xbar, double& beta_lo, double& beta_up){
  //cut with constraints
  arr gbar = g + Jg*(xbar-x);
  arr a = Jg*d;
  for(uint i=0;i<gbar.N;i++){
    double ai = a.elem(i);
    if(fabs(ai)>1e-6){
      double beta = -gbar.elem(i) / ai;
      if(ai<0. && beta>beta_lo) beta_lo=beta;
      if(ai>0. && beta<beta_up) beta_up=beta;
    }
  }

  //cut with lower bound
  gbar = nlp.bounds_lo - xbar;
  a = -d;
  for(uint i=0;i<gbar.N;i++){
    double ai = a.elem(i);
    if(fabs(ai)>1e-6){
      double beta = -gbar.elem(i) / ai;
      if(ai<0. && beta>beta_lo) beta_lo=beta;
      if(ai>0. && beta<beta_up) beta_up=beta;
    }
  }

  //cut with upper bound
  gbar = xbar - nlp.bounds_up;
  a = d;
  for(uint i=0;i<gbar.N;i++){
    double ai = a.elem(i);
    if(fabs(ai)>1e-6){
      double beta = -gbar.elem(i) / ai;
      if(ai<0. && beta>beta_lo) beta_lo=beta;
      if(ai>0. && beta<beta_up) beta_up=beta;
    }
  }

}


bool NLP_Walker::step(double maxStep){
  //    eval_gAb_hCd(x, true);
  if(!phi.N) eval(x, true);

  //store:
  arr phi0=phi, J0=J,
      g0=g, Jg0=Jg,
      h0=h, Jh0=Jh,
      s0=s, Js0=Js,
      gpos0=gpos;

  //rnd direction
  arr dir, delta;
  get_rnd_direction(dir, delta);

  //remember sufficient accept conditions
  double expectedSlackDecrease = alpha * sum(Js*delta);

  //get lo/up lambda
  double beta_lo=-maxStep, beta_up=maxStep;

  arr y;
  arr xbar = x;
  if(h.N) xbar += alpha*delta;
  for(uint i=0;;i++){ //``line search''
    clipBeta(dir, xbar, beta_lo, beta_up);

    double beta=0.;
    if(beta_up>beta_lo){
      if(beta_sdv<0.){
        beta = beta_lo + rnd.uni()*(beta_up-beta_lo);
      }else{ //Gaussian beta
        bool good=false;
        for(uint k=0;k<10;k++){
          beta = beta_mean + beta_sdv*rnd.gauss();
          if(beta>=beta_lo && beta<=beta_up){ good=true; break; }
        }
        if(!good){
          beta = beta_lo + rnd.uni()*(beta_up-beta_lo);
//          LOG(0) <<evals <<"uniform";
        }
      }
    }else{
      if(absMax(delta)<1e-6) break; //no step in the feasible -> not ok
    }

    samples++;
    y = xbar + beta*dir;
    eval(y, true);

    if((!g.N || max(gpos) <= max(gpos0)) //constraints are good
       && (sum(s) <= sum(s0) + eps + .1 * expectedSlackDecrease) ){ //Wolfe accept
      if(maxDiff(y, x)<1e-6){
        LOG(-1) <<"why?" <<delta <<s <<s0 <<expectedSlackDecrease;
      }
      x = y;
      return true;
    }

    CHECK(g.N, "you shouldn't be here if there are no ineqs!");
    if(i>=10) break; //give up
    if(beta_up<=beta_lo) break;
  }


  //-- bad
  //reset evaluations:
  phi=phi0; J0=J;
  g=g0; Jg=Jg0;
  h=h0; Jh=Jh0;
  s=s0; Js=Js0;
  gpos=gpos0;
  return false; //line search in 10 steps failed
}

bool NLP_Walker::step_delta(){
  if(!phi.N) eval(x, true);

  arr dir, delta;
  get_rnd_direction(dir, delta);
  x += delta;
  eval(x, true);
  return true;
}

void NLP_Walker::get_rnd_direction(arr& dir, arr& delta){
  //slack step
  if(s.N && absMax(s)>1e-9){
    //Gauss-Newton direction
    arr H = 2. * ~Js * Js;
    arr grad = 2. * ~Js * s;
    arr Hinv = pseudoInverse(H);
    delta = -(Hinv * grad);
  }else{
    delta = zeros(x.N);
  }

  arr Ph;
  if(h.N){ //equality constraints
    //Gauss-Newton direction
    arr H = 2. * ~Jh * Jh;
    arr Hinv = pseudoInverse(H);
    Ph = (eye(x.N) - Hinv * H); //tangent projection
  }else{
    Ph = eye(x.N);
  }

  //rnd direction
  dir = randn(x.N);
  if(h.N) dir = Ph * dir;
  dir /= length(dir);

  //sos step and precision matrix
  beta_mean = 0.;
  beta_sdv = -1.;
  if(r.N){
    arr r_bar = r + Jr*(alpha*delta);
    arr Jr_d = Jr * dir;
    double Jr_d_2 = sumOfSqr(Jr_d);
    if(Jr_d_2>1e-6){
      beta_mean = - scalarProduct(r_bar, Jr_d) / Jr_d_2;
      beta_sdv = sqrt(0.5/Jr_d_2);
    }
  }
}

void NLP_Walker::eval(const arr& _x, bool update_phi){
  if(update_phi){
    evals++;
    phi, J;
    nlp.evaluate(phi, J, _x);
    if(rai::isSparse(J)) J = J.sparse().unsparse();
  }

  {//grab ineqs
    uintA ineqIdx;
    for(uint i=0;i<nlp.featureTypes.N;i++) if(nlp.featureTypes(i)==OT_ineq) ineqIdx.append(i);
    g = phi.sub(ineqIdx);
    Jg = J.sub(ineqIdx);
  }

  {//grab eqs
    uintA eqIdx;
    for(uint i=0;i<nlp.featureTypes.N;i++) if(nlp.featureTypes(i)==OT_eq) eqIdx.append(i);
    h = phi.sub(eqIdx);
    Jh = J.sub(eqIdx);
  }

  {//define slack
    s = g; Js = Jg;
    for(uint i=0;i<s.N;i++) if(s(i)<0.){ s(i)=0.; Js[i]=0.; } //ReLu for g
    gpos = s;

    s.append(h); Js.append(Jh);
    for(uint i=g.N;i<s.N;i++) if(s(i)<0.){ s(i)*=-1.; Js[i]*=-1.; } //make positive

    err = sum(s);
  }

  {//grab sos
    uintA sosIdx;
    for(uint i=0;i<nlp.featureTypes.N;i++) if(nlp.featureTypes(i)==OT_sos) sosIdx.append(i);
    r = phi.sub(sosIdx);
    Jr = J.sub(sosIdx);
  }
}

//===========================================================================

arr sample_direct(NLP& nlp, uint K, int verbose){
  NLP_Walker walk(nlp);

  walk.initialize(nlp.getInitializationSample());

  arr data;

  for(;data.d0<K;){
    arr x0 = walk.x;
    bool good = walk.step();

    if(good){
      data.append(walk.x);
      data.reshape(-1, walk.x.N);
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_direct it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<endl;

  data.reshape(-1, nlp.getDimension());
  return data;
}

//===========================================================================

arr sample_restarts(NLP& nlp, uint K, int verbose){
  NLP_Walker walk(nlp);

  arr data;

  for(;data.d0<K;){
//    arr x = nlp.getInitializationSample();
    arr x = nlp.bounds_lo + rand(nlp.getDimension()) % (nlp.bounds_up - nlp.bounds_lo);
    walk.initialize(x);

    bool good = false;
    for(uint t=0;t<100;t++){
      good = walk.step();
      if(t<50 && walk.err>2.*walk.eps) good=false;
      if(good) break;
//      komo->pathConfig.setJointState(sam.x);
//      komo->view(true, STRING(k <<' ' <<t <<' ' <<sam.err <<' ' <<good));
    }

    if(good){
      for(uint t=0;t<10;t++){
        walk.step_delta();
        if(walk.err<=.01) break;
      }
      if(walk.err>.01) good=false;
    }

    if(good){
      data.append(walk.x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_restarts it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<" #sam: " <<data.d0 <<endl;
  return data;
}

//===========================================================================

arr sample_greedy(NLP& nlp, uint K, int verbose){
  NLP_Walker walk(nlp);

  arr data;

  for(;data.d0<K;){
    arr x = nlp.bounds_lo + rand(nlp.getDimension()) % (nlp.bounds_up - nlp.bounds_lo);
    walk.initialize(x);

    bool good = false;
    for(uint t=0;t<100;t++){
      walk.step_delta();
      if(walk.err<=.01){ good=true; break; }
    }

    if(good){
      data.append(walk.x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_restarts it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<" #sam: " <<data.d0 <<endl;
  return data;
}

//===========================================================================

arr sample_denoise(NLP& nlp, uint K, int verbose){
  AlphaSchedule A(AlphaSchedule::_cosine, 50, .1);
  //  AlphaSchedule A(AlphaSchedule::_constBeta, 50, .2);
  cout <<A.alpha_bar <<endl;

  RegularizedNLP nlp_reg(nlp);
  NLP_Walker walk(nlp_reg);

  arr data;

  for(;data.d0<K;){
    arr x = nlp.bounds_lo + rand(nlp.getDimension()) % (nlp.bounds_up - nlp.bounds_lo);
    arr trace = x;
    trace.append(x);
    walk.initialize(x);
    walk.x = nlp.bounds_lo + rand(nlp.getDimension()) % (nlp.bounds_up - nlp.bounds_lo);

    for(int t=A.alpha_bar.N-1;t>0;t--){
      //get the alpha
      double bar_alpha_t = A.alpha_bar(t);
      double bar_alpha_t1 = A.alpha_bar(t-1);
      double alpha_t = bar_alpha_t/bar_alpha_t1;

      //sample an x0
      nlp_reg.setRegularization(1/sqrt(bar_alpha_t)*x, (1-bar_alpha_t)/bar_alpha_t);
      walk.step();

      //denoise
      x = (  (sqrt(bar_alpha_t1) * (1.-alpha_t)) * walk.x
             + (sqrt(alpha_t) * (1.-bar_alpha_t1)) * x )
          / (1.-bar_alpha_t);
      if(t>1){
        arr z = randn(x.N);
        x += z * sqrt( ((1.-bar_alpha_t1) * (1.-alpha_t)) / (1.-bar_alpha_t));
      }

      trace.append(x);
      trace.append(walk.x);
    }

#if 0
    trace.reshape(A.alpha_bar.N, 2*x.N);
    FILE("z.dat") <<trace.modRaw();
    gnuplot("plot 'z.dat' us 0:1 t 'x', 'z.dat' us 0:2 t 'x', 'z.dat' us 0:3 t 'x0', 'z.dat' us 0:4 t 'x0'");
    rai::wait();
#endif

    bool good = true;
//    if(walk.err>2.*walk.eps) good=false;

    if(good){
      for(uint t=0;t<10;t++){
        walk.step_delta();
        if(walk.err<=.01) break;
      }
      if(walk.err>.01) good=false;
    }

    if(good){
      data.append(x);
      data.reshape(-1, nlp.getDimension());
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(verbose>1 || (good && verbose>0)){
      nlp.report(cout, 2+verbose, STRING("sample_denoise it: " <<data.d0 <<" good: " <<good));
    }
  }
  cout <<"\nsteps/sample: " <<double(walk.samples)/K <<" evals/sample: " <<double(walk.evals)/K <<endl;
  return data;
}

//===========================================================================

AlphaSchedule::AlphaSchedule(AlphaSchedule::Mode mode, uint T, double beta){
  alpha_bar.resize(T+1);

  if(mode==_constBeta){
    CHECK(beta>0, "beta parameter needed");
    double alpha = 1.-beta*beta;
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = pow(alpha, double(t));
    }
  }else if(mode==_cosine){
    double s = .01;
    double f0 = rai::sqr(::cos(s/(1.+s) * RAI_PI/2.));
    for(uint t=0;t<alpha_bar.N;t++){
      double ft = rai::sqr(::cos( ((double(t)/alpha_bar.N)+s)/(1.+s) * RAI_PI/2.));
      alpha_bar(t) = ft/f0;
    }
  }else if(mode==_linear){
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
    }
  }else if(mode==_sqrtLinear){
    for(uint t=0;t<alpha_bar.N;t++){
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
      alpha_bar(t) = sqrt(alpha_bar(t));
    }
  }
}

