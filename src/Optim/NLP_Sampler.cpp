/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "NLP_Sampler.h"

#include <Core/util.h>
#include <math.h>

NLP_Sampler_Options::NLP_Sampler_Options() {
  if(lagevinTauPrime>0.) {
    slackStepAlpha = lagevinTauPrime / penaltyMu;
    noiseSigma = ::sqrt(2.*lagevinTauPrime / penaltyMu);
    LOG(0) <<"lagevinTauPrime: " <<lagevinTauPrime <<" overwriting alpha=" <<slackStepAlpha <<" and sigma=" <<noiseSigma;
  }
  //    LOG(0) <<"loaded params:\n" <<rai::params()();
}

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

void NLP_Walker::set_alpha_bar(double alpha_bar) {
  if(alpha_bar==1.) {
    a = 1.;
    sig = 0.;
  } else {
    a = ::sqrt(alpha_bar);
    sig = ::sqrt(1.-alpha_bar);
  }
  if(!opt.useCentering) {
    a = 1.;
  }
}

bool NLP_Walker::step() {
  ensure_eval();

  bool good = true;
//  if(!ev.Ph.N || trace(ev.Ph)>1e-6) {
//    good = step_hit_and_run_eq(); //(opt.maxStep);
//  }

  step_noise(.2);
  step_bound_clip();

  good = step_slack();
  good = step_slack();

  return good;
}

bool NLP_Walker::step_hit_and_run() {
  ensure_eval();
  Eval ev0 = ev;

  if(opt.hitRunEqMargin>0.) ev.convert_eq_to_ineq(opt.hitRunEqMargin);
  double g0 = rai::MAX(max(ev.g), 0.);
  arr s0 = ev.s;

  arr dir = get_rnd_direction();

  LineSampler LS(2.*opt.slackMaxStep);
  LS.clip_beta(nlp.bounds[0] - x, -dir); //cut with lower bound
  LS.clip_beta(x - nlp.bounds[1], dir); //cut with upper bound
  for(uint i=0; i<10; i++) { //``line search''
    //cut with inequalities
    //TODO: zero a(i) when ev.g(i) <= 0
    LS.clip_beta(ev.g + ev.Jg*(x-ev.x), ev.Jg*dir);

    if(LS.beta_lo >= LS.beta_up) break; //failure

    double beta = LS.sample_beta_uniform();

    x += beta*dir;
    ensure_eval();
    if(opt.hitRunEqMargin>0.) ev.convert_eq_to_ineq(opt.hitRunEqMargin);

    if((!ev.g.N || max(ev.g) <= g0) //ineq constraints are good
        && sum(ev.s) <= sum(s0) + opt.eps) { //total slack didn't increase too much

      //MH accept
      if(ev.r.N){ // we have sos costs
        double Enew = sumOfSqr(ev.r);
        double Eold = sumOfSqr(ev0.r);
        if(Enew<Eold) return true;
        double p_ratio = ::exp(Eold - Enew);
        if(rnd.uni() < p_ratio) return true;
      }else{
        return true;
      }
    }
  }

  ev = ev0;
  x = ev.x;
  return false; //line search in 10 steps failed
}

bool NLP_Walker::step_hit_and_run_old(double maxStep) {
  ensure_eval();
  Eval ev0 = ev;

  arr dir = get_rnd_direction();

  double beta_mean, beta_sdv;
  get_beta_mean(beta_mean, beta_sdv, dir, x);

  boundClip(x, nlp.bounds);

  LineSampler LS(2.*maxStep);
  LS.clip_beta(nlp.bounds[0] - x, -dir); //cut with lower bound
  LS.clip_beta(x - nlp.bounds[1], dir); //cut with upper bound
  LS.add_constraints(a*ev.g + ev.Jg*(x-a*ev.x), ev.Jg*dir);
  for(uint i=0; i<10; i++) { //``line search''
    double beta = NAN;
    if(!sig) {
      //cut with constraints
      LS.clip_beta(ev.g + ev.Jg*(x-ev.x), ev.Jg*dir);

      if(LS.beta_lo >= LS.beta_up) break; //failure

      if(beta_sdv<0.) {
        beta = LS.sample_beta_uniform();
      } else { //Gaussian beta
        bool good=false;
        for(uint k=0; k<10; k++) {
          beta = beta_mean + beta_sdv*rnd.gauss();
          if(beta>=LS.beta_lo && beta<=LS.beta_up) { good=true; break; }
        }
        if(!good) {
          beta = LS.sample_beta_uniform();
          //          LOG(0) <<evals <<"uniform";
        }
      }
    } else {
      if(LS.beta_up>LS.beta_lo) {
        beta = LS.sample_beta();
        //    cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<endl;
        if(LS.p_beta<1e-10) beta=NAN;
      }
      if(isnan(beta)) break;
    }

    x += beta*dir;
    ensure_eval();

    if(!sig) {
      if((!ev.g.N || max(ev.gpos) <= max(ev0.gpos)) //ineq constraints are good
          && sum(ev.s) <= sum(ev0.s) + opt.eps) { //total slack didn't increase too much
        return true;
      }
    } else {
      if(sum(ev.s) <= sum(ev0.s) + sig + opt.eps) { //total slack didn't increase too much
        if(!ev.g.N) return true;
        LS.add_constraints(a*ev.g + ev.Jg*(x-a*ev.x), ev.Jg*dir);
        double p_beta = LS.eval_beta(beta);
        //      cout <<"beta: " <<beta <<" p(beta): " <<LS.p_beta <<" p(beta) " <<p_beta <<endl;
        if(p_beta >= 1e-3*LS.p_beta) {
          return true;
        }
      }
    }
  }

  ev = ev0;
  x = ev.x;
  return false; //line search in 10 steps failed
}

bool NLP_Walker::step_slack(double penaltyMu, double alpha, double maxStep, double lambda) {
  ensure_eval();
  Eval ev0 = ev;

  //compute delta
  arr Hinv = lapack_inverseSymPosDef(((2.*penaltyMu)*~ev.Js)*ev.Js+lambda*eye(x.N));
  arr delta = (-2.*penaltyMu) * Hinv * (~ev.Js) * ev.s;

  //adapt step size
  if(alpha<0.) alpha = opt.slackStepAlpha;
  if(maxStep<0.) maxStep = opt.slackMaxStep;
  delta *= alpha;
  double l = length(delta);
  if(l>maxStep) delta *= maxStep/l;

  //apply
  x += delta;
  ensure_eval();

  if(sum(ev.s) > sum(ev0.s)) {
    x -= .5*delta;
    ensure_eval();
  }

  return true;
}

bool NLP_Walker::step_noise(double sig) {
  CHECK(sig>0., "");

  x += sig * randn(x.N);

  return true;
}

bool NLP_Walker::step_noise_covariant(double sig, double penaltyMu, double lambda) {
  ensure_eval();

  CHECK(sig>0., "");

  arr Hinv = lapack_inverseSymPosDef(((2.*penaltyMu)*~ev.Js)*ev.Js+lambda*eye(x.N));
  arr C;
  lapack_cholesky(C, Hinv);
//  arr cov =  2. * ~ev.Js*ev.Js+lambda*eye(x.N);
  arr z = C * randn(x.N);

  x += sig * z;

  return true;
}

bool NLP_Walker::step_bound_clip() {
  boundClip(x, nlp.bounds);
  return true;
}

void NLP_Walker::run(arr& data, arr& trace) {

  //-- init
  if(data.N && opt.initNovelty>0){
    init_novelty(data, opt.initNovelty);
  }else{
    arr x_init = nlp.getUniformSample();
    initialize(x_init);
  }

  if(opt.verbose>3) {
    ensure_eval();
    nlp.report(cout, 2+opt.verbose, STRING("sampling INIT, err: " <<ev.err));
    rai::wait(.1);
  }

  if(!!trace) {
    trace.append(x);
    trace.reshape(-1, nlp.getDimension());
  }

  bool good=false;
  int interiorSteps = 0;
  CHECK_GE(opt.interiorSteps, opt.interiorStepsBurnIn, "burn in needs to be smaller than steps");

  for(uint t=0;; t++) {

    //-- hit-and-run step
    if(good && opt.interiorSteps>interiorSteps) {
      step_hit_and_run();
      interiorSteps++;
    }

    //-- noise step
    bool noiseStep = (int)t<opt.noiseSteps;
    if(noiseStep) {
      CHECK(opt.noiseSigma>0., "you can't have noise steps without noiseSigma");
      if(opt.noiseCovariant) {
        step_noise_covariant(opt.noiseSigma, opt.penaltyMu, opt.slackRegLambda);
      } else {
        step_noise(opt.noiseSigma);
      }
      step_bound_clip();
    }

    //-- slack step
    if(opt.slackStepAlpha>0.) {
      step_slack(opt.penaltyMu, opt.slackStepAlpha, opt.slackMaxStep, opt.slackRegLambda);
      step_bound_clip();
    }

    //-- accept
    if(opt.acceptBetter) {
      NIY;
    } else if(opt.acceptMetropolis) {
      NIY;
    }

    //-- store trace
    if(!!trace) {
      trace.append(x);
    }

    //-- good?
    good = (ev.err<=.01);

    //-- store?
    if((good && interiorSteps>=opt.interiorStepsBurnIn)) {
      data.append(x);
      data.reshape(-1, x.N);
      if(!(data.d0%10)) cout <<'.' <<std::flush;
    }

    if(opt.verbose>1 || (good && opt.verbose>0)) {
      nlp.report(cout, (good?2:1)+opt.verbose, STRING("sampling t: " <<t <<" err: " <<ev.err <<" data: " <<data.d0 <<" good: " <<good <<" interior: " <<interiorSteps));
      rai::wait(.1);
    }

    //-- stopping
    if((good && interiorSteps>=opt.interiorSteps)
        || (t>=(uint)opt.downhillMaxSteps)) {
      if(opt.verbose>1 && (!!trace)) {
        FILE("z.dat") <<trace.modRaw();
        gnuplot("plot [-2:2][-2:2] 'z.dat' us 1:2 w lp");
        if(opt.verbose>1) rai::wait();
      }
      break;
    }
  }
}

void NLP_Walker::init_novelty(const arr& data, uint D){
  struct Seed{ NLP_Walker::Eval ev; arr delta; double novelty=-1.; };
  rai::Array<Seed> seeds(D);

  //sample D seeds & evaluate novelty
  for(uint k=0;k<seeds.N;k++){
    Seed& seed = seeds(k);
    arr x_init = nlp.getUniformSample();
    initialize(x_init);
    ensure_eval();
    seed.ev = ev;
    seed.delta = compute_slackStep();
    seed.delta /= (length(seed.delta) + 1e-8);

    for(uint i=0;i<data.d0;i++){
      arr del = x_init-data[i];
      double nn = scalarProduct(seed.delta, del);
      if(nn>seed.novelty) seed.novelty = nn;
    }
  }

  //select most novel
  Seed* seed = &seeds(0);
  for(uint k=1;k<seeds.N;k++){
    if(seeds(k).novelty> seed->novelty) seed = &seeds(k);
  }
  ev = seed->ev;
  x = ev.x;
}

void NLP_Walker::get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar) {
  beta_mean = 0.;
  beta_sdv = -1.;
  if(ev.r.N) {
    arr r_bar = ev.r + ev.Jr*(xbar - ev.x);
    arr Jr_d = ev.Jr * dir;
    double Jr_d_2 = sumOfSqr(Jr_d);
    if(Jr_d_2>1e-6) {
      beta_mean = - scalarProduct(r_bar, Jr_d) / Jr_d_2;
      beta_sdv = sqrt(0.5/Jr_d_2);
    }
  }
}

arr NLP_Walker::get_rnd_direction() {
  arr dir = randn(x.N);
//  if(ev.Ph.N) dir = ev.Ph * dir;
  dir /= length(dir);
  return dir;
}

void NLP_Walker::Eval::eval(const arr& _x, NLP_Walker& walker) {
  if(x.N && maxDiff(_x, x)<1e-10) {
    return; //already evaluated
  }
  x = _x;
  walker.evals++;

  phi, J;
  walker.nlp.evaluate(phi, J, _x);
  if(rai::isSparse(J)) J = J.sparse().unsparse();

  {
    //grab ineqs
    uintA ineqIdx;
    for(uint i=0; i<walker.nlp.featureTypes.N; i++) if(walker.nlp.featureTypes(i)==OT_ineq) ineqIdx.append(i);
    g = phi.sub(ineqIdx);
    Jg = J.sub(ineqIdx);
  }

  {
    //grab eqs
    uintA eqIdx;
    for(uint i=0; i<walker.nlp.featureTypes.N; i++) if(walker.nlp.featureTypes(i)==OT_eq) eqIdx.append(i);
    h = phi.sub(eqIdx);
    Jh = J.sub(eqIdx);
  }

  {
    //define slack
    s = g; Js = Jg;
    for(uint i=0; i<s.N; i++) if(s(i)<0.) { s(i)=0.; Js[i]=0.; } //ReLu for g
    gpos = s;

    s.append(h); Js.append(Jh);
    for(uint i=g.N; i<s.N; i++) if(s(i)<0.) { s(i)*=-1.; Js[i]*=-1.; } //make positive

    err = sum(s);
  }

  {
    //grab sos
    uintA sosIdx;
    for(uint i=0; i<walker.nlp.featureTypes.N; i++) if(walker.nlp.featureTypes(i)==OT_sos) sosIdx.append(i);
    r = phi.sub(sosIdx);
    Jr = J.sub(sosIdx);
  }

//  if(h.N) { //projection of equality constraints
//    //Gauss-Newton direction
////    double lambda = 1e-2;
////    arr H = 2. * ~Jh * Jh;
////    for(uint i=0;i<H.d0;i++) H(i,i) += lambda;
////    arr Hinv = inverse_SymPosDef(H);
//    arr H = 2. * ~Jh * Jh;
//    arr Hinv = pseudoInverse(H);
//    Ph = eye(x.N) - Hinv * H; //tangent projection
//  } else {
//    Ph.clear();
//  }
}

void NLP_Walker::Eval::convert_eq_to_ineq(double margin) {
  g.append(h-margin);
  Jg.append(Jh);
  g.append(-h-margin);
  Jg.append(-Jh);
}

//===========================================================================

AlphaSchedule::AlphaSchedule(AlphaSchedule::Mode mode, uint T, double beta) {
  alpha_bar.resize(T+1);

  if(mode==_constBeta) {
    CHECK(beta>0, "beta parameter needed");
    double alpha = 1.-beta*beta;
    for(uint t=0; t<alpha_bar.N; t++) {
      alpha_bar(t) = pow(alpha, double(t));
    }
  } else if(mode==_cosine) {
    double s = .01;
    double f0 = rai::sqr(::cos(s/(1.+s) * RAI_PI/2.));
    for(uint t=0; t<alpha_bar.N; t++) {
      double ft = rai::sqr(::cos(((double(t)/alpha_bar.N)+s)/(1.+s) * RAI_PI/2.));
      alpha_bar(t) = ft/f0;
    }
  } else if(mode==_linear) {
    for(uint t=0; t<alpha_bar.N; t++) {
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
    }
  } else if(mode==_sqrtLinear) {
    for(uint t=0; t<alpha_bar.N; t++) {
      alpha_bar(t) = 1.-double(t)/(alpha_bar.N);
      alpha_bar(t) = sqrt(alpha_bar(t));
    }
  }
}

//===========================================================================

double normalCDF(double value) {
  return 0.5 * erfc(-value * M_SQRT1_2);
}

double LineSampler::eval_beta(double beta) {
  double p=1.;
  for(uint i=0; i<b.N; i++) {
    if(fabs(s(i))>1e-6) {
//      p *= rai::sigmoid( (beta-b(i))/s(i) );
      p *= normalCDF((beta-b(i))/s(i));
    } else {
      if(beta<b(i)) { p = 0.; break; } //single hard barrier violated
    }
  }
  p = ::pow(p, 1./num_constraints);
  return p;
}

void LineSampler::add_constraints(const arr& gbar, const arr& gd) {
  uint n=b.N;
  b.append(zeros(gbar.N));
  s.append(zeros(gbar.N));
  for(uint i=0; i<gbar.N; i++) {
    double gdi = gd.elem(i);
    double si = 0.;
    if(fabs(gdi)>1e-6) si = -1./gdi;
    s(n+i) = si;
    b(n+i) = gbar(i);
  }
  num_constraints++;
}

void LineSampler::add_constraints_eq(const arr& hbar, const arr& hd, double margin) {
  uint n=b.N;
  b.append(zeros(2*hbar.N));
  s.append(zeros(2*hbar.N));
  for(uint i=0; i<hbar.N; i++) {
    double hdi = hd.elem(i);
    double si = 0.;
    if(fabs(hdi)>1e-6) si = -1./hdi;
    s(n+2*i) = si;
    b(n+2*i) = hbar(i) * si - rai::sign(si)*margin;
    s(n+2*i+1) = -si;
    b(n+2*i+1) = hbar(i) * si + rai::sign(si)*margin;
  }
}

void LineSampler::clip_beta(const arr& gbar, const arr& gd) {
  for(uint i=0; i<gbar.N; i++) {
    double gdi = gd.elem(i);
    if(fabs(gdi)>1e-6) {
      double beta = -gbar.elem(i) / gdi;
      if(gdi<0. && beta>beta_lo) beta_lo=beta;
      if(gdi>0. && beta<beta_up) beta_up=beta;
    }
  }
}

double LineSampler::sample_beta() {
  //-- find outer interval
  double z = 3.;
  for(uint i=0; i<b.N; i++) {
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
  for(uint i=0; i<betas.N; i++) p_betas(i) = eval_beta(betas(i));

  //-- SUS from these samples
  arr sum_p = integral(p_betas);
  double total_p = sum_p(-1);
  if(total_p<1e-10) return NAN;
  double r = rnd.uni() * total_p;
  uint i = 0;
  for(; i<sum_p.N; i++) { if(r<sum_p(i)) break; }
  p_beta = p_betas(i);
  return betas(i);
}

double LineSampler::sample_beta_uniform() { return beta_lo + rnd.uni()*(beta_up-beta_lo); }

void LineSampler::plot() {
  arr betas = range(-2., 2., 100);
  arr p_betas(betas.N);
  for(uint i=0; i<betas.N; i++) p_betas(i) = eval_beta(betas(i));

  FILE("z.dat") <<rai::catCol({betas, p_betas}).modRaw() <<endl;
  gnuplot("plot 'z.dat' us 1:2");
  rai::wait();
}

