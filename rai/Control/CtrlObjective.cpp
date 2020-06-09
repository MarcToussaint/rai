/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlObjective.h"
#include "CtrlTargets.h"
#include "CtrlSolvers.h"
#include "komoControl.h"
#include "../KOMO/komo.h"
#include "../Core/graph.h"
#include "../Kin/frame.h"
#include "../Kin/kin_swift.h"
#include "../Kin/TM_default.h"
#include "../Kin/F_qFeatures.h"

//===========================================================================

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>=0., "this does not define proper gains!");
  double lambda = decayTime*dampingRatio/(-log(.1));
//  double lambda = decayTime/(-log(.1)); //assume the damping ratio always 1. -- just so that setting ratio to zero still gives a reasonable value
  double freq = 1./lambda;
  Kp = freq*freq;
  Kd = 2.*dampingRatio*freq;
}

//===========================================================================

//CtrlObjective::CtrlObjective(const char* _name, const ptr<Feature>& _feat, const ptr<CtrlReference>& _ref, double _kp, double _kd, const arr& _C)
//  : feat(_feat), name(name), ref(_ref), active(true), kp(_kp), kd(_kd), C(_C), status(AS_init){
//}

void CtrlObjective::resetState() { if(ref) ref->resetState(); status=AS_init; }

arr CtrlObjective::update_y(const ConfigurationL& Ctuple) {
  arr y_old = y;
  feat->__phi(y, J_y, Ctuple);
  if(y_old.N==y.N) return y-y_old;
  return zeros(y.N);
}

void CtrlObjective::setRef(const ptr<CtrlTarget>& _ref) {
  CHECK(!ref, "ref is already set");
  ref = _ref;
}

void CtrlObjective::setTarget(const arr& y_target) {
  CHECK(ref, "need a ref to set target");
  feat->setTarget(y_target);
  ref->resetState();
}

void CtrlObjective::setTimeScale(double d) { CHECK(ref, ""); ref->setTimeScale(d); ref->resetState(); }

void CtrlObjective::reportState(ostream& os) {
  os <<"  CtrlObjective " <<name <<':';
  if(!active) cout <<" INACTIVE";
  if(ref){
    if(feat->target.N==y.N){
      os <<" \ty_ref=" <<feat->target <<" \ty=" <<y <<" \ty-err=" <<length(feat->target-y);
    }
//    if(ref->v_ref.N==y.N){
//      os <<" \tv_ref=" <<ref->v_ref;
//    }
    os <<endl;
  } else {
    os <<" -- no reference defined " <<endl;
  }
}


CtrlProblem::CtrlProblem(rai::Configuration& _C, double _tau, uint k_order)
  : tau(_tau) {
  komo.setModel(_C, true);
  komo.setTiming(1., 1, _tau, k_order);
  komo.setupConfigurations( _C.getJointState() );
  komo.verbose=0;
}

ptr<CtrlObjective> CtrlProblem::addObjective(const ptr<Feature>& f, ObjectiveType type) {
  ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = f;
  t->update_y(komo.configurations);
//  if(t->feat->order==0){
//    t->ref = make_shared<CtrlReference_MaxCarrot>(.1, target);
//  }else if(t->feat->order==1){
//    t->ref = make_shared<CtrlReference_ConstVel>();
//  }else{
//    NIY
//  }
  objectives.append(t);
  return t;
}

ptr<CtrlObjective> CtrlProblem::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order) {
  return addObjective(symbols2feature(feat, frames, komo.getConfiguration(0), scale, target, order), type);
}

void CtrlProblem::update(rai::Configuration& C) {
  //push configurations
  for(uint s=1;s<komo.configurations.N;s++){
    komo.configurations(s-1)->setJointState( komo.configurations(s)->getJointState() );
  }
  arr q = C.getJointState();
  komo.setConfiguration(-1, q);
  komo.setConfiguration(0, q);

  for(std::shared_ptr<CtrlObjective>& o: objectives){
    if(!o->name.N) o->name = o->feat->shortTag(C);

    arr y = o->y;
    if(o->feat->scale.N)   y *= (1./o->feat->scale.scalar());
    if(o->feat->target.N)  y += o->feat->target;

    if(o->ref){
      ActStatus s_new = o->ref->step(o->feat->target, tau, y);
      if(s_new != o->status){
        o->status = s_new;
        //callbacks
      }
    }

    o->update_y(komo.configurations);
  }
}

void CtrlProblem::report(std::ostream& os){
  for(ptr<CtrlObjective>& o: objectives){
    o->reportState(os);
  }
}

arr CtrlProblem::solve() {
#if 0
  TaskControlMethods M(komo.getConfiguration_t(0).getHmetric());
  arr q = komo.getConfiguration_t(0).getJointState();
  q += M.inverseKinematics(objectives, NoArr, {});
  return q;
#elif 1
  komo.clearObjectives();
  for(std::shared_ptr<CtrlObjective>& o: objectives){
    komo.addObjective({}, o->feat, o->type);
  }
  OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  opt.stopIters = 10;
//  opt.nonStrictSteps=-1;
//  opt.maxStep = .1*tau; //maxVel*tau;
  opt.maxStep = 1.;
  komo.optimize(0., opt);
  return komo.getPath().reshape(-1);
#else
  return solve_optim(*this);
#endif
}
