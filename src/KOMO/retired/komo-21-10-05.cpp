/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void KOMO::addSwitch_stable(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch) {
  addModeSwitch({time, endTime}, SY_stable, {from, to}, firstSwitch);
}

void KOMO::addSwitch_stableOn(double time, double endTime, const char* prevFrom, const char* from, const char* to, bool firstSwitch) {
  addModeSwitch({time, endTime}, SY_stableOn, {from, to}, firstSwitch);
}

void KOMO::addSwitch_dynamic(double time, double endTime, const char* from, const char* to, bool dampedVelocity) {
  addSwitch({time}, true, JT_free, SWInit_copy, from, to);
  if(!dampedVelocity)
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, 2, +0, -1);
  else
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(), {to}, OT_eq, {1e2}, NoArr, 1, +0, -1);
//  addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2); //this should be implicit in the NE equations!
}

void KOMO::addSwitch_dynamicTrans(double time, double endTime, const char* from, const char* to) {
  HALT("deprecated")
//  addSwitch({time}, true, JT_trans3, SWInit_copy, from, to);
//  addObjective({time, endTime}, make_shared<F_NewtonEuler>(true), {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
}

void KOMO::addSwitch_dynamicOn(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, FS_pose, {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
//  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
}

void KOMO::addSwitch_dynamicOnNewton(double time, double endTime, const char* from, const char* to) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, from, to, rel);
  if(k_order>=2) addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, k_order, +0, -1);
}

void KOMO::addSwitch_magic(double time, double endTime, const char* from, const char* to, double sqrAccCost, double sqrVelCost) {
  addSwitch({time}, true, JT_free, SWInit_copy, from, to);
  if(sqrVelCost>0. && k_order>=1) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrVelCost}, NoArr, 1);
  }
  if(sqrAccCost>0. && k_order>=2) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrAccCost}, NoArr, 2);
  }
}

void KOMO::addSwitch_magicTrans(double time, double endTime, const char* from, const char* to, double sqrAccCost) {
  addSwitch({time}, true, JT_transZ, SWInit_copy, from, to);
  if(sqrAccCost>0.) {
    addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_eq, {sqrAccCost}, NoArr, 2);
  }
}
