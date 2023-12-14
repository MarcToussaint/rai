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

double eval_beta(double beta, const arr& b, const arr& s){
  double p=1.;
  for(uint i=0;i<b.N;i++){
    if(fabs(s(i))>1e-6){
      p *= rai::sigmoid( (beta-b(i))/s(i) );
    }else{
      if(beta<b(i)){ p = 0.; break; } //single hard barrier violated
    }
  }
  return p;
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

  arr x0 = x;
  arr xbar = x;
  if(h.N) xbar += alpha*delta;
  LineSampler LS;
  LS.clip_beta(nlp.bounds_lo - xbar, -dir); //cut with lower bound
  LS.clip_beta(xbar - nlp.bounds_up, dir); //cut with upper bound
  LS.add_constraints(g + Jg*(xbar-x), Jg*dir, temperature);
  for(uint i=0;;i++){ //``line search''
#if 0
    //cut with constraints
    LS.clip_beta(g + Jg*(xbar-x), Jg*dir);

    double beta=0.;
    if(LS.beta_up>LS.beta_lo){
      if(beta_sdv<0.){
        beta = LS.sample_beta_uniform();
      }else{ //Gaussian beta
        bool good=false;
        for(uint k=0;k<10;k++){
          beta = beta_mean + beta_sdv*rnd.gauss();
          if(beta>=LS.beta_lo && beta<=LS.beta_up){ good=true; break; }
        }
        if(!good){
          beta = LS.sample_beta_uniform();
//          LOG(0) <<evals <<"uniform";
        }
      }
    }else{
      if(absMax(delta)<1e-6) break; //no step in the feasible -> not ok
    }
#else
    //LS.add_constraints(g + Jg*(xbar-x), Jg*dir, temperature);
    double beta = LS.sample_beta();
    if(isnan(beta)) break;
//    cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<endl;
    if(LS.p_beta<1e-10) break;
#endif
    x = xbar + beta*dir;
    samples++;
    eval(x, true);

#if 0
    if((!g.N || max(gpos) <= max(gpos0)) //constraints are good
       && (sum(s) <= sum(s0) + eps + .1 * expectedSlackDecrease) ){ //Wolfe accept
      if(maxDiff(x, x0)<1e-6){
        LOG(-1) <<"why?" <<delta <<s <<s0 <<expectedSlackDecrease;
      }
      return true;
    }
#else
    LS.add_constraints(g + Jg*(xbar-x), Jg*dir, temperature);
    double p_beta = sqrt(LS.eval_beta(beta));
    cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<" p(beta) " <<p_beta <<endl;
    if(p_beta >= 1e-2*LS.p_beta){
      return true;
    }
#endif

    CHECK(g.N, "you shouldn't be here if there are no ineqs!");
    if(i>=10) break; //give up
    if(LS.beta_up<=LS.beta_lo) break;
  }


  //-- bad
  //reset evaluations:
  x = x0;
  phi=phi0; J=J0;
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
    arr Hinv = pseudoInverse(H, NoArr, 1e-6);
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

arr sample_direct(NLP& nlp, uint K, int verbose, double temperature){
  NLP_Walker walk(nlp, temperature);

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


double LineSampler::eval_beta(double beta){
  double p=1.;
  for(uint i=0;i<b.N;i++){
    if(fabs(s(i))>1e-6){
      p *= rai::sigmoid( (beta-b(i))/s(i) );
    }else{
      if(beta<b(i)){ p = 0.; break; } //single hard barrier violated
    }
  }
  return p;
}

void LineSampler::add_constraints(const arr& gbar, const arr& gd, double temperature){
  uint n=b.N;
  b.append(zeros(gbar.N));
  s.append(zeros(gbar.N));
  for(uint i=0;i<gbar.N;i++){
    double gdi = gd.elem(i);
    if(fabs(gdi)>1e-6) s(n+i) = -1./gdi;
    b(n+i) = gbar(i) * s(n+i);
  }
  s *= temperature;
}

void LineSampler::clip_beta(const arr& gbar, const arr& gd){
  for(uint i=0;i<gbar.N;i++){
    double gdi = gd.elem(i);
    if(fabs(gdi)>1e-6){
      double beta = -gbar.elem(i) / gdi;
      if(gdi<0. && beta>beta_lo) beta_lo=beta;
      if(gdi>0. && beta<beta_up) beta_up=beta;
    }
  }
}

double LineSampler::sample_beta(){
  //-- find outer interval
  double z = 3.;
  for(uint i=0;i<b.N;i++){
    if(s(i)>0. && b(i)-z*s(i)>beta_lo) beta_lo=b(i)-z*s(i);
    if(s(i)<0. && b(i)-z*s(i)<beta_up) beta_up=b(i)-z*s(i);
  }

  if(beta_up<beta_lo) return NAN;

  //-- sample uniformly in the interval
  arr betas = rand(10);
  betas *= beta_up-beta_lo;
  betas += beta_lo;

  //-- evaluate all samples
  arr p_betas(betas.N);
  for(uint i=0;i<betas.N;i++) p_betas(i) = eval_beta(betas(i));

  //-- SUS from these samples
  arr sum_p = integral(p_betas);
  double total_p = sum_p(-1);
  if(total_p<1e-10) return NAN;
  double r = rnd.uni() * total_p;
  uint i = 0;
  for(;i<sum_p.N;i++){ if(r<sum_p(i)) break; }
  p_beta = p_betas(i);
  return betas(i);
}

double LineSampler::sample_beta_uniform(){ return beta_lo + rnd.uni()*(beta_up-beta_lo); }

void LineSampler::plot(){
  arr betas = range(-2., 2., 100);
  arr p_betas(betas.N);
  for(uint i=0;i<betas.N;i++) p_betas(i) = eval_beta(betas(i));

  FILE("z.dat") <<rai::catCol({betas, p_betas}).modRaw() <<endl;
  gnuplot("plot 'z.dat' us 1:2");
  rai::wait();
}
