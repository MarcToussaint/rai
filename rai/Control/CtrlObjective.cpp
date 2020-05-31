/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlObjective.h"
#include "CtrlReference.h"
#include "CtrlMethods.h"
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

arr CtrlObjective::update_y(const rai::Configuration& C) {
  arr y_old = y;
  feat->__phi(y, J_y, C);
  if(y_old.N==y.N) return y-y_old;
  return zeros(y.N);
}

void CtrlObjective::setRef(const ptr<CtrlReference>& _ref) {
  CHECK(!ref, "ref is already set");
  ref = _ref;
}

void CtrlObjective::setTarget(const arr& y_target) {
  CHECK(ref, "need a ref to set target");
  ref->setTarget(y_target);
  ref->resetState();
}

void CtrlObjective::setTimeScale(double d) { CHECK(ref, ""); ref->setTimeScale(d); ref->resetState(); }

void CtrlObjective::reportState(ostream& os) {
  os <<"  CtrlObjective " <<name;
  if(!active) cout <<" INACTIVE";
  if(ref && ref->y_ref.N==y.N && ref->v_ref.N==y.N) {
    os <<": "/*y_target=" <<PD().y_target */<<" \ty_ref=" <<ref->y_ref <<" \ty=" <<y
       <<"  y-err=" <<length(ref->y_ref-y)
//       <<"  v-err=" <<length(v_ref-v)
       <<endl;
  } else {
    os <<" -- y_ref.N!=y.N or ydot_ref.N!=v.N -- not initialized? -- " <<endl;
  }
}


ptr<CtrlObjective> CtrlProblem::addObjective(const FeatureSymbol& feat, const StringA& frames, ObjectiveType type, const arr& scale, const arr& target, int order) {
  ptr<CtrlObjective> t = make_shared<CtrlObjective>();
  t->feat = symbols2feature(feat, frames, C, scale, target, order);
  t->ref = make_shared<CtrlReference_MaxCarrot>(arr{}, .1);
  t->update_y(C);
  t->ref->setTarget(0.*t->y);
  objectives.append(t);
  return t;
}

void CtrlProblem::update(rai::Configuration& C, double tau) {
  for(ptr<CtrlObjective>& o: objectives){
    arr dy = o->update_y(C);

    if(o->ref){
      ActStatus s_old = o->status;
      ActStatus s_new = s_old;
      s_new = o->ref->step(tau, o->y, dy/tau);
      if(s_new!=s_old) o->status=s_new;
    }
  }
}

arr CtrlProblem::solve() {
  TaskControlMethods M(C.getHmetric());
  arr q = C.getJointState();
  arr dq, qdot;
  dq = M.inverseKinematics(objectives, qdot, {});
  q += dq;
  return q;


}
