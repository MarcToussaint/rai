/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "SlackGaussNewton.h"

#include <Core/util.h>
#include <math.h>
#include <Algo/ann.h>

#include <iomanip>

namespace rai {

void SlackGaussNewton::step() {
  ev.eval(x, *this);

  //compute delta
  arr R = comp_At_A(ev.Js);
  for(uint i=0;i<R.d1;i++) R.elem(i,i) += opt.damping;
  arr g = comp_At_x(ev.Js, ev.s);
  arr delta = - lapack_Ainv_b_sym(R, g);

  //restrict stepsize
  double maxDelta = absMax(delta);
  if(opt.maxStep>0. && maxDelta>opt.maxStep) {
    delta *= opt.maxStep/maxDelta;
    maxDelta = opt.maxStep;
  }

  //apply
  x += delta;
  boundClip(x, nlp->bounds);
  ev.eval(x, *this);

  if(opt.verbose>1) {
    cout <<"--resolve-- it:" <<std::setw(4) <<iters;
    cout <<"  |Delta|:" <<std::setw(11) <<maxDelta;
    cout <<"  evals:" <<std::setw(4) <<evals;
    cout <<"  s:" <<std::setw(11) <<::sum(ev.s);
    cout <<"  h:" <<std::setw(11) <<ev.err(OT_eq);
    cout <<"  g:" <<std::setw(11) <<ev.err(OT_ineq);
    cout <<endl;
    // nlp->report(cout, opt.verbose, STRING("phase1 iter: " <<iters <<" err: " <<::sum(ev.s)));
    // rai::wait(.1);
  }
}

std::shared_ptr<SolverReturn> SlackGaussNewton::solve(){
  std::shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();

  ret->time = -rai::cpuTime();
  {
    x = nlp->getInitializationSample();
    boundClip(x, nlp->bounds);
    ev.eval(x, *this);

    if(opt.verbose>0) {
      cout <<"==resolve== initialization ";
      cout <<"  s:" <<std::setw(11) <<::sum(ev.s);
      cout <<"  h:" <<std::setw(11) <<ev.err(OT_eq);
      cout <<"  g:" <<std::setw(11) <<ev.err(OT_ineq);
      cout <<endl;
      // nlp->report(cout, 2+opt.verbose, STRING("sampling INIT, err: " <<::sum(ev.s)));
      // rai::wait(.1);
    }

    bool good;
    for(;;) {
      step();
      // good = (::sum(ev.s)<=opt.tolerance);
      good = ev.err(OT_ineq)<=opt.tolerance && ev.err(OT_eq)<1e-10;
      if(good || evals>=opt.maxEvals) break;
      iters++;
    }

    if(opt.verbose>0) {
      cout <<"==resolve== done ";
      cout <<"  s:" <<std::setw(11) <<::sum(ev.s);
      cout <<"  h:" <<std::setw(11) <<ev.err(OT_eq);
      cout <<"  g:" <<std::setw(11) <<ev.err(OT_ineq);
      if(!good) cout <<" [exceeded maxEvals]";
      cout <<endl;
      // nlp->report(cout, 2+opt.verbose, STRING("sampling INIT, err: " <<::sum(ev.s)));
      // rai::wait(.1);
    }
  }
  ret->time += rai::cpuTime();

  ret->x = ev.x;
  ret->evals = evals;
  ret->sos = ev.err(OT_sos);
  ret->f = ev.err(OT_f);
  ret->eq = ev.err(OT_eq);
  ret->ineq = ev.err(OT_ineq);
  ret->done = true;
  ret->feasible = (ret->ineq<.1) && (ret->eq<.1);

  return ret;
}

void SlackGaussNewton::Eval::eval(const arr& _x, SlackGaussNewton& walker) {
  if(x.N && maxDiff(_x, x)<1e-10) {
    return; //already evaluated
  }
  x = _x;
  walker.evals++;

  phi, J;
  walker.nlp->evaluate(phi, J, _x);
  if(rai::isSparse(J)) J.sparse().setupRowsCols();
  err = walker.nlp->summarizeErrors(phi);

  //grab constraints
  ObjectiveTypeA& ft = walker.nlp->featureTypes;
  uintA idx;
  for(uint i=0; i<ft.N; i++) if(ft(i)==OT_ineq || ft(i)==OT_eq) idx.append(i);
  s = phi.sub(idx);
  if(!rai::isSparse(J)){
    Js = J.sub(idx);
  }else{
    Js.sparse().resize(idx.N, J.d1, 0);
    for(uint i=0;i<idx.N;i++) Js.sparse().add(J.sparse().getSparseRow(idx(i)), i, 0);
    Js.sparse().setupRowsCols();
  }

  // make positive errors
  for(uint i=0; i<s.N; i++){
    if(ft(idx(i))==OT_ineq) s(i) += walker.opt.margin;
    if(s(i)<0.) {
      if(ft(idx(i))==OT_ineq) {//ReLu for g
        s(i)=0.;
        if(!rai::isSparse(J)) Js[i]=0.; else Js.sparse().multRow(i, 0.);
      }else{
        CHECK_EQ(ft(idx(i)), OT_eq, ""); //L1 for eq
        s(i)*=-1.;
        if(!rai::isSparse(J)) Js[i]*=-1.; else Js.sparse().multRow(i, -1.);
      }
    }
  }
}


} //namespace
