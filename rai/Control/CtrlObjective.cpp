/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "CtrlObjective.h"
#include "CtrlSolver.h"
#include "CtrlTargets.h"

#include "../KOMO/komo.h"
#include "../Core/graph.h"
#include "../Kin/frame.h"
#include "../Kin/kin_swift.h"
#include "../Kin/TM_default.h"
#include "../Kin/F_qFeatures.h"

//===========================================================================

//CtrlObjective::CtrlObjective(const char* _name, const ptr<Feature>& _feat, const ptr<CtrlReference>& _ref, double _kp, double _kd, const arr& _C)
//  : feat(_feat), name(name), ref(_ref), active(true), kp(_kp), kd(_kd), C(_C), status(AS_init){
//}

arr CtrlObjective::getResidual(CtrlSolver& cp) {
  return movingTarget->getResidual(getValue(cp));
}

arr CtrlObjective::getValue(CtrlSolver& cp) {
  FrameL F = groundFeatureFrames(feat, cp.komo.pathConfig, 2);
  arr y, J;
  feat->__phi2(y, J, F);
  return y;
}

void CtrlObjective::resetState() { if(movingTarget) movingTarget->resetState(); status=AS_init; }

//arr CtrlObjective::update_y(const ConfigurationL& Ctuple) {
//  arr y_old = y;
//  feat->__phi(y, J_y, Ctuple);
//  if(y_old.N==y.N) return y-y_old;
//  return zeros(y.N);
//}

void CtrlObjective::setRef(const ptr<CtrlMovingTarget>& _ref) {
  CHECK(!movingTarget, "ref is already set");
  movingTarget = _ref;
}

void CtrlObjective::setTarget(const arr& y_target) {
  CHECK(movingTarget, "need a ref to set target");
  feat->setTarget(y_target);
  if(movingTarget) {
    movingTarget->resetGoal(y_target);
    movingTarget->resetState();
  }
  status = AS_init;
}

void CtrlObjective::setTimeScale(double d) { CHECK(movingTarget, ""); movingTarget->setTimeScale(d); movingTarget->resetState(); }

void CtrlObjective::reportState(ostream& os) const {
  os <<"  CtrlObjective " <<name <<':';
  if(!active) cout <<" INACTIVE";
  cout <<rai::Enum<ActStatus>(status) <<' ';
  if(movingTarget) {
    os <<" -- moving target:" <<feat->target <<' ';
    movingTarget->reportState(os);
//    if(ref->v_ref.N==y.N){
//      os <<" \tv_ref=" <<ref->v_ref;
//    }
    os <<endl;
  } else {
    os <<" -- fixed target:" <<feat->target <<endl;
  }
}

