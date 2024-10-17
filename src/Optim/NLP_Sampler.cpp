/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "NLP_Sampler.h"

#include <Core/util.h>
#include <math.h>
#include <Algo/ann.h>

NLP_Sampler_Options::NLP_Sampler_Options() {
//  if(langevinTauPrime>0.) {
//    slackStepAlpha = langevinTauPrime / penaltyMu;
//    noiseSigma = ::sqrt(2.*langevinTauPrime / penaltyMu);
//    LOG(0) <<"lagevinTauPrime: " <<langevinTauPrime <<" overwriting alpha=" <<slackStepAlpha <<" and sigma=" <<noiseSigma;
//  }
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
    //      arr y = nlp->getInitializationSample();
    bool inBound = boundCheck(y, nlp->bounds_lo, nlp->bounds_up, 1e-6, false);
    if(inBound && max(g)<=0.){ //accept
      x = y;
      s++;
    }
  }
}
*/

bool NLP_Sampler::step_hit_and_run() {
  store_eval();

  if(opt.hitRunEqMargin>0.) ev.convert_eq_to_ineq(opt.hitRunEqMargin);
  double g0 = rai::MAX(max(ev.g), 0.);
  arr s0 = ev.s;

  arr dir = get_rnd_direction();

  LineSampler LS(2.*opt.slackMaxStep);
  LS.clip_beta(nlp->bounds[0] - x, -dir); //cut with lower bound
  LS.clip_beta(x - nlp->bounds[1], dir); //cut with upper bound
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
        double gamma=1., mu=1e3;
        double Enew = gamma*sumOfSqr(ev.r) + mu*sum(ev.s);
        double Eold = gamma*sumOfSqr(ev_stored.r) + mu*sum(ev_stored.s);
        if(Enew<Eold) return true;
        double p_ratio = ::exp(Eold - Enew);
        if(rnd.uni() < p_ratio) return true;
        //reject! restore previous state
        ev = ev_stored;
        x = ev.x;
        return false;
      }else{
        return true;
      }
    }
  }

  ev = ev_stored;
  x = ev.x;
  return false; //line search in 10 steps failed
}

/*
bool NLP_Sampler::step_hit_and_run_old(double maxStep) {
  ensure_eval();
  Eval ev0 = ev;

  arr dir = get_rnd_direction();

  double beta_mean, beta_sdv;
  get_beta_mean(beta_mean, beta_sdv, dir, x);

  boundClip(x, nlp->bounds);

  LineSampler LS(2.*maxStep);
  LS.clip_beta(nlp->bounds[0] - x, -dir); //cut with lower bound
  LS.clip_beta(x - nlp->bounds[1], dir); //cut with upper bound
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
*/

bool NLP_Sampler::step_GaussNewton(bool slackMode, double penaltyMu, double alpha, double maxStep, double lambda) {
  ensure_eval();
  store_eval();

  //compute delta
  arr delta;
  if(slackMode){
    arr Hinv = lapack_inverseSymPosDef(((2.*penaltyMu)*~ev.Js)*ev.Js+lambda*eye(x.N));
    delta = (-2.*penaltyMu) * Hinv * (~ev.Js) * ev.s;
  }else{
    arr Hinv = lapack_inverseSymPosDef(((2.*penaltyMu)*~ev.Jr)*ev.Jr+lambda*eye(x.N));
    delta = (-2.*penaltyMu) * Hinv * (~ev.Jr) * ev.r;
  }

  //adapt step size
  if(alpha<0.) alpha = opt.slackStepAlpha;
  if(maxStep<0.) maxStep = opt.slackMaxStep;
  delta *= alpha;
  double l = length(delta);
  if(l>maxStep) delta *= maxStep/l;

  //apply
  x += delta;
  ensure_eval();

  return true;
}

void NLP_Sampler::step_PlainGrad(bool slackMode, double penaltyMu, double alpha, double maxStep) {
  ensure_eval();
  store_eval();

  //compute delta
  arr delta;
  if(slackMode){
    delta = (-2.*penaltyMu) * (~ev.Js) * ev.s;
  }else{
    delta = (-2.*penaltyMu) * (~ev.Jr) * ev.r;
  }

  //adapt step size
  if(alpha<0.) alpha = opt.slackStepAlpha;
  if(maxStep<0.) maxStep = opt.slackMaxStep;
  delta *= alpha;
  double l = length(delta);
  if(l>maxStep) delta *= maxStep/l;

  //apply
  x += delta;
  ensure_eval();
}

bool NLP_Sampler::step_noise(double sig) {
  CHECK(sig>0., "");

  x += sig * randn(x.N);

  return true;
}

bool NLP_Sampler::step_noise_covariant(double sig, double penaltyMu, double lambda) {
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

void NLP_Sampler::bound_clip() {
  boundClip(x, nlp->bounds);
}

void NLP_Sampler::step_Langevin(bool slackMode, double tauPrime, double penaltyMu){
  store_eval();
  double alpha = tauPrime / penaltyMu;
  double sigma = ::sqrt(2.*tauPrime / penaltyMu);
//  LOG(0) <<"lagevinTauPrime: " <<tauPrime <<" overwriting alpha=" <<alpha <<" and sigma=" <<sigma;
  arr x_old = x;
  step_GaussNewton(slackMode, penaltyMu, alpha, opt.slackMaxStep, 1e-6);
  arr del = x-x_old;
  step_noise(sigma);
  if(slackMode){
    reject_MH(0., opt.penaltyMu, del, sigma);
  }else{
    double mu=1e3;
    if(ev.h.N) mu=0.; //no penalties in eq-case!
    reject_MH(1., mu, del, sigma);
  }
}

bool NLP_Sampler::reject_MH(double gamma, double mu, const arr& asymmetric_del, double sigma){
  ensure_eval();
  double Enew = gamma*sumOfSqr(ev.r) + mu*sum(ev.s);
  double Eold = gamma*sumOfSqr(ev_stored.r) + mu*sum(ev_stored.s);
  if(asymmetric_del.N){
    arr& x_old = ev_stored.x;
    arr& x_new = ev.x;
    Enew += (-.5/(sigma*sigma)) * sumOfSqr(x_old - (x_new + asymmetric_del)); //log q(old|new)
    Eold += (-.5/(sigma*sigma)) * sumOfSqr(x_new - (x_old + asymmetric_del)); //log q(new|old)
  }
  if(Enew<Eold) return true;
  double p_ratio = ::exp(Eold - Enew);
  if(rnd.uni() < p_ratio) return true;
  //reject! restore previous state
  ev = ev_stored;
  x = ev.x;
  return false;
}

bool NLP_Sampler::run_downhill(){
  opt.slackStepAlpha = rai::getParameter("sam/slackStepAlpha", 1.);

  for(int t=0;t<opt.downhillMaxSteps; t++) {
    //-- noise step
    if(opt.downhillNoiseMethod=="none") {
      //none
    }else if(opt.downhillNoiseMethod=="iso") {
      CHECK(opt.downhillNoiseSigma>0., "you can't have noise steps without noiseSigma");
      step_noise(opt.downhillNoiseSigma);
      bound_clip();
    }else if(opt.downhillNoiseMethod=="cov") {
      CHECK(opt.downhillNoiseSigma>0., "you can't have noise steps without noiseSigma");
      step_noise_covariant(opt.downhillNoiseSigma, opt.penaltyMu, opt.slackRegLambda);
      bound_clip();
    }else NIY;

    //-- slack step
    if(opt.slackStepAlpha>0.) {
      if(opt.downhillMethod=="GN") {
        step_GaussNewton(true, opt.penaltyMu, opt.slackStepAlpha, opt.slackMaxStep, opt.slackRegLambda);
      }else if(opt.downhillMethod=="grad") {
        step_PlainGrad(true, opt.penaltyMu, opt.slackStepAlpha, opt.slackMaxStep);
      }else NIY;
      bound_clip();
    }

    //-- accept
    if(opt.downhillRejectMethod=="none") {
      //none
    } else if(opt.downhillRejectMethod=="Wolfe") {
      CHECK_EQ(opt.downhillNoiseMethod, "none", "Wolfe doesn't work with noise");
      if(sum(ev.s) > sum(ev_stored.s)) {
        opt.slackStepAlpha *= .5;
        ev = ev_stored;
        x = ev.x;
      }else{
        opt.slackStepAlpha *= 1.2;
        rai::clip(opt.slackStepAlpha, 0., 1.);
      }
    } else if(opt.downhillRejectMethod=="MH") {
      CHECK(opt.downhillNoiseMethod != "none", "MH only with noise");
      reject_MH(0., opt.penaltyMu, ev.x - ev_stored.x, opt.downhillNoiseSigma);
    }

    //-- good?
    bool good = (ev.err<=opt.tolerance);

    if(opt.verbose>2 || (good && opt.verbose>1)) {
      nlp->report(cout, (good?1:0)+opt.verbose, STRING("phase1 t: " <<t <<" err: " <<ev.err <<" good: " <<good));
      rai::wait(.1);
    }

    //-- stopping
    if(good) return true;
  }
  return false;
}

void NLP_Sampler::run_interior(arr& data, uintA& dataEvals){
  if(opt.interiorBurnInSteps<0) opt.interiorBurnInSteps=0;
  if(opt.interiorSampleSteps<1) opt.interiorSampleSteps=1; //at least 1 interior sample
  int interiorSteps = opt.interiorBurnInSteps + opt.interiorSampleSteps - 1;

  shared_ptr<ANN> ann;
  arr annPh;
  if(opt.interiorMethod=="manifoldRRT"){
    ann = make_shared<ANN>();
  }

  for(int t=0;; t++) {
    //-- good?
    ensure_eval();
    bool good = (ev.err<=opt.tolerance);

    //-- manifoldRRT builds tree from all points (previously slack-stepped)
    if(opt.interiorMethod=="manifoldRRT"){
      ann->append(x);
      //arr H = 2. * ~ev.Jh * ev.Jh;
      //arr Ph = eye(x.N) - pseudoInverse(H, NoArr, 1e-6) * H; //tangent projection
      arr Ph = eye(x.N) - ~ev.Jh * pseudoInverse(ev.Jh * ~ev.Jh, NoArr, 1e-6) * ev.Jh; //tangent projection
      annPh.append(Ph);
      annPh.reshape(ann->X.d0, x.N, x.N); //tensor of eq-constraint projections
    }

    //-- store?
    if(good && t>=opt.interiorBurnInSteps) {
      data.append(x);
      data.reshape(-1, x.N);
      dataEvals.append(evals);
      CHECK_EQ(data.d0, dataEvals.d0, "");
      if(!(data.d0%10)) cout <<'.' <<std::flush;
      if(opt.verbose>=2) {
        nlp->report(cout, 9, STRING("data stored phase2 t: " <<t <<" err: " <<ev.err <<" data: " <<data.d0 <<" good: " <<good));
        rai::wait(.1);
      }
    }

    //-- stopping
    if(t>=interiorSteps) break;

    if(opt.interiorMethod=="HR"){
      //-- hit-and-run step
      step_hit_and_run();

    }else if(opt.interiorMethod=="MCMC"){
      store_eval();
      if(opt.interiorNoiseMethod=="iso") {
        step_noise(opt.interiorNoiseSigma);
      }else if(opt.interiorNoiseMethod=="cov") {
        step_noise_covariant(opt.interiorNoiseSigma, 1e3, 1e-2);
      }else NIY;
      double mu=1e3;
      if(ev.h.N) mu=0.; //no penalties in eq-case!
      reject_MH(1., mu);

    }else if(opt.interiorMethod=="Langevin"){
      step_Langevin(false, opt.langevinTauPrime, opt.penaltyMu);

    }else if(opt.interiorMethod=="manifoldRRT"){
      arr x_target = nlp->getUniformSample();
      uint p = ann->getNN(x_target);
      x = ann->X[p].copy();
      arr dir = x_target - x;
      dir = annPh[p] * dir;
      double stepsize = opt.interiorNoiseSigma;
      dir *= stepsize/length(dir);
      x += dir;

    }else HALT("interior method not define: " <<opt.interiorMethod);

    ensure_eval();
    good = (ev.err<=opt.tolerance);

    //-- slack step
    if(opt.slackStepAlpha>0. && !good) {
      step_GaussNewton(true, opt.penaltyMu, 1., opt.slackMaxStep, opt.slackRegLambda);
      bound_clip();
    }

    ensure_eval();
    good = (ev.err<=opt.tolerance);

    if(opt.verbose>2 || (good && opt.verbose>1)) {
      nlp->report(cout, (good?1:0)+opt.verbose, STRING("phase2 t: " <<t <<" err: " <<ev.err <<" data: " <<data.d0 <<" good: " <<good));
      rai::wait(.1);
    }
  }
}

void NLP_Sampler::run(arr& data, uintA& dataEvals) {

  //-- novelty init?
  if(data.N && opt.seedMethod=="nov"){
    init_novelty(data, opt.seedCandidates);
  }else if(data.N && opt.seedMethod=="dist"){
    init_distance(data, opt.seedCandidates);
  }else if(opt.seedMethod=="gauss"){
    arr x_init = nlp->getInitializationSample();
    initialize(x_init);
  }else if(!data.N || opt.seedMethod=="uni"){
    arr x_init = nlp->getUniformSample();
    initialize(x_init);
  }else NIY;

  if(opt.verbose>3) {
    ensure_eval();
    nlp->report(cout, 2+opt.verbose, STRING("sampling INIT, err: " <<ev.err));
    rai::wait(.1);
  }

  bool good = run_downhill();

  if(!good) return;

  run_interior(data, dataEvals);
}

std::shared_ptr<SolverReturn> NLP_Sampler::sample(){
  arr data;
  uintA dataEvals;
  std::shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();

  ret->time = -rai::cpuTime();
  run(data, dataEvals);
  ret->time += rai::cpuTime();

  ret->x = ev.x;
  ret->evals = evals;
  ret->sos = sumOfSqr(ev.r);
  ret->f = 0.;
  ret->eq = sumOfAbs(ev.h);
  ret->ineq = sumOfPos(ev.g);
  ret->done = true;
  ret->feasible = (ret->ineq<.1) && (ret->eq<.1);
//  if(!dataEvals.N){
//    ret->feasible=false;
//    ret->x.clear();
//  }else{
//    ret->x.reshape(-1);
//    ret->evals = dataEvals.elem();

//    ret->feasible = true; //(ret->ineq<.1) && (ret->eq<.1);
//  }

  return ret;
}

void NLP_Sampler::init_novelty(const arr& data, uint D){
  struct Seed{ NLP_Sampler::Eval ev; arr delta; double align=-1.; };
  rai::Array<Seed> seeds(D);

  //sample D seeds & evaluate align
  for(uint k=0;k<seeds.N;k++){
    Seed& seed = seeds(k);
    arr x_init = nlp->getUniformSample();
    initialize(x_init);
    ensure_eval();
    seed.ev = ev;
    seed.delta = compute_slackStep();
    seed.delta /= (length(seed.delta) + 1e-8);

    for(uint i=0;i<data.d0;i++){
      arr del = data[i]-x_init;
      del /= (length(del) + 1e-8);
      double align = scalarProduct(seed.delta, del);
      //large align is bad: you walk into data[i] direction
      if(align>seed.align) seed.align = align;
    }
  }

  //select least aligned seed
  Seed* seed = &seeds(0);
  for(uint k=1;k<seeds.N;k++){
    if(seeds(k).align < seed->align) seed = &seeds(k);
  }
  ev = seed->ev;
  x = ev.x;
}

void NLP_Sampler::init_distance(const arr& data, uint D){
    arr x_init;
    double d_init = -1.;
#if 0
  for(uint k=0;k<D;k++){
    arr x = nlp->getUniformSample();
    double d=1e10;
    for(uint i=0;i<data.d0;i++){
      double di = length(data[i]-x);
      if(di<d) d=di;
    }
    if(d > d_init){ d_init=d; x_init=x; }
  }
#else
    ANN ann;
    ann.setX(data);
    arr x, sqrDists;
    uintA idx;
    for(uint k=0;k<D;k++){
      x = nlp->getUniformSample();
      ann.getkNN(sqrDists, idx, x, 1);
      double d = sqrDists.elem();
      if(d > d_init){ d_init=d; x_init=x; }
    }
#endif
  initialize(x_init);
}

void NLP_Sampler::get_beta_mean(double& beta_mean, double& beta_sdv, const arr& dir, const arr& xbar) {
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

arr NLP_Sampler::get_rnd_direction() {
  arr dir = randn(x.N);
  if(ev.h.N) { //projection of equality constraints
    //Gauss-Newton direction
    //  double lambda = 1e-2;
    //  arr H = 2. * ~Jh * Jh;
    //  for(uint i=0;i<H.d0;i++) H(i,i) += lambda;
    //  arr Hinv = inverse_SymPosDef(H);
    arr H = 2. * ~ev.Jh * ev.Jh;
    arr Ph = eye(x.N) - pseudoInverse(H) * H; //tangent projection
    dir = Ph * dir;
  }

  dir /= length(dir);
  return dir;
}

void NLP_Sampler::Eval::eval(const arr& _x, NLP_Sampler& walker) {
  if(x.N && maxDiff(_x, x)<1e-10) {
    return; //already evaluated
  }
  x = _x;
  walker.evals++;

  phi, J;
  walker.nlp->evaluate(phi, J, _x);
  if(rai::isSparse(J)) J = J.sparse().unsparse();

  {
    //grab ineqs
    uintA ineqIdx;
    for(uint i=0; i<walker.nlp->featureTypes.N; i++) if(walker.nlp->featureTypes(i)==OT_ineq) ineqIdx.append(i);
    g = phi.sub(ineqIdx);
    Jg = J.sub(ineqIdx);
  }

  {
    //grab eqs
    uintA eqIdx;
    for(uint i=0; i<walker.nlp->featureTypes.N; i++) if(walker.nlp->featureTypes(i)==OT_eq) eqIdx.append(i);
    h = phi.sub(eqIdx);
    Jh = J.sub(eqIdx);
  }

  {
    //define slack
    s = g; Js = Jg;
    for(uint i=0; i<s.N; i++) if(s(i)<0.) { s(i)=0.; Js[i]=0.; } //ReLu for g
    gpos = s;
    if(walker.opt.ineqOverstep>1.){
      Js /= walker.opt.ineqOverstep;
    }

    s.append(h); Js.append(Jh);
    for(uint i=g.N; i<s.N; i++) if(s(i)<0.) { s(i)*=-1.; Js[i]*=-1.; } //make positive

    err = sum(s);
  }

  {
    //grab sos
    uintA sosIdx;
    for(uint i=0; i<walker.nlp->featureTypes.N; i++) if(walker.nlp->featureTypes(i)==OT_sos) sosIdx.append(i);
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

void NLP_Sampler::Eval::convert_eq_to_ineq(double margin) {
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

