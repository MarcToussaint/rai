/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlProblem.h"

#include "CtrlTargets.h"

CtrlProblem::CtrlProblem(rai::Configuration& _C, double _tau, uint k_order)
  : tau(_tau) {
  komo.setModel(_C, true);
  komo.setTiming(1., 1, _tau, k_order);
  komo.setupConfigurations(_C.getJointState());
  komo.verbose=0;
}

std::shared_ptr<CtrlObjective> CtrlProblem::add_qControlObjective(uint order, double scale, const arr& target) {
  ptr<Objective> o = komo.add_qControlObjective({}, order, scale, target);
  return addObjective(o->feat, o->type);
}

void CtrlProblem::addObjectives(const rai::Array<ptr<CtrlObjective>>& O) {
  for(auto& o:O) {
    objectives.append(o.get());
    o->selfRemove = &objectives;
  }
}

void CtrlProblem::delObjectives(const rai::Array<ptr<CtrlObjective>>& O) {
  for(auto& o:O) {
    objectives.removeValue(o.get());
    o->selfRemove = 0;
  }
}

ptr<CtrlObjective> CtrlProblem::addObjective(const ptr<Feature>& _feat, ObjectiveType _type) {
  std::shared_ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = _feat;
  t->type = _type;
  addObjectives({t});
  return t;
}

ptr<CtrlObjective> CtrlProblem::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order) {
  return addObjective(symbols2feature(feat, frames, komo.getConfiguration(0), scale, target, order), type);
}

void CtrlProblem::update(rai::Configuration& C) {
  //-- update the KOMO configurations (push one step back, and update current configuration)
  for(uint s=1; s<komo.configurations.N; s++) {
    komo.configurations(s-1)->setJointState(komo.configurations(s)->getJointState());
  }
  arr X = C.getFrameState();
  komo.getConfiguration_t(-1).setFrameState(X);
  komo.getConfiguration_t(0).setFrameState(X);
  for(auto* c:komo.configurations) c->ensure_q();

  //-- step the targets forward, if they have a target
  for(CtrlObjective* o: objectives) if(o->active){
    if(!o->name.N) o->name = o->feat->shortTag(C);

    if(o->movingTarget) {
      o->y_buffer = o->getValue(*this);
      ActStatus s_new = o->movingTarget->step(tau, o, komo.configurations);
      if(o->status != s_new) {
        o->status = s_new;
        //callbacks
      }
    } else {
      if(o->status != AS_running) {
        o->status = AS_running;
        //callbacks
      }
    }
  }
}

void CtrlProblem::report(std::ostream& os) {
  os <<"    control objectives:" <<endl;
  for(CtrlObjective* o: objectives) o->reportState(os);
  os <<"    optimization result:" <<endl;
  os <<optReport <<endl;
}

arr CtrlProblem::solve() {
#if 0
  TaskControlMethods M(komo.getConfiguration_t(0).getHmetric());
  arr q = komo.getConfiguration_t(0).getJointState();
  q += M.inverseKinematics(objectives, NoArr, {});
  return q;
#elif 1
  komo.clearObjectives();
  for(CtrlObjective* o: objectives) if(o->active){
    komo.addObjective({}, o->feat, o->type);
  }
  OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
//  opt.stopIters = 2;
//  opt.nonStrictSteps=-1;
  opt.maxStep = .1; //*tau; //maxVel*tau;
//  opt.maxStep = 1.;
//  komo.verbose=4;
  komo.optimize(0., opt);
  optReport = komo.getReport(false);
  return komo.getPath().reshape(-1);
#else
  return solve_optim(*this);
#endif
}
