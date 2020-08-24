/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "motionHeuristics.h"
#include "taskMaps.h"
#include "../Optim/optimization.h"
#include "../Kin/kin_swift.h"
#include "../Gui/opengl.h"

//===========================================================================
//
// the rest is on the three base routines
//

void threeStepGraspHeuristic(arr& x, KOMO& MP, uint shapeId, uint verbose) {
  uint T = MP.T;
  //double duration = sys.getTau() * T;

  listDelete(MP.tasks());

  uint side=0;

  //-- optimize ignoring hand -- testing different options for aligning with the object
  if(MP.world.shapes(shapeId)->type==rai::ST_box) {
    arr cost_side(3), x_side(3, MP.world.q.N);
    for(side=0; side<3; side++) {
      setGraspGoals_PR2(MP, T, shapeId, side, 0);
      cost_side(side) = keyframeOptimizer(x, MP, false, verbose);
      listDelete(MP.tasks());
      if(verbose>=2) {
        displayState(x, MP.world, STRING("posture estimate phase 0 side " <<side));
      }
      x_side[side]() = x;
    }
    cout <<"3 side costs=" <<cost_side <<endl;
    side = cost_side.argmin();
    x = x_side[side];
  } else {
    setGraspGoals_PR2(MP, T, shapeId, side, 0);
    keyframeOptimizer(x, MP, false, verbose);
    listDelete(MP.tasks());
    if(verbose>=2) {
      displayState(x, MP.world, "posture estimate phase 0");
    }
  }

  //-- open hand
  //x({7,13}) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  x(MP.world.getJointByName("l_gripper_l_finger_joint")->qIndex) = 1.;

  if(verbose>=2) {
    displayState(x, MP.world, "posture estimate phase 1");
  }

  //-- reoptimize with close hand
  setGraspGoals_PR2(MP, T, shapeId, side, 1);
  keyframeOptimizer(x, MP, true, verbose);
  if(verbose>=1) displayState(x, MP.world, "posture estimate phase 2");
}

#if 0 //setInterpolatingCosts is need refactoring...
void setGraspGoals_Schunk(KOMO& MP, uint T, uint shapeId, uint side, uint phase) {
  MP.setState(MP.x0, MP.v0);;

  //load parameters only once!
  double positionPrec = rai::getParameter<double>("graspPlanPositionPrec");
  double oppositionPrec = rai::getParameter<double>("graspPlanOppositionPrec");
  double alignmentPrec = rai::getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = rai::getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = rai::getParameter<double>("graspPlanColPrec");
  double limPrec = rai::getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = rai::getParameter<double>("graspPlanZeroQPrec");

  //set the time horizon
  CHECK_EQ(T, MP.T, "");

  //deactivate all variables
  MP.activateAllTaskCosts(false);

  //activate collision testing with target shape
  rai::Shape* target_shape = MP.world.shapes(shapeId);
  target_shape->cont=true;
  MP.world.swift().initActivations(MP.world);

  //
  arr target, initial;
  rai::Vector ivec, jvec;

  //general target
  target = conv_vec2arr(target_shape->X.pos);
  //xtarget(2) += .02; //grasp it 2cm above center

  //-- graspCenter -> predefined point (xtarget)
  Task* c;
  c = MP.addTask("graspCenter",
                 new TM_Default(TMT_pos, "graspCenter"));
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, positionPrec, NoArr, -1., .8);

  //-- up: align either with cylinder axis or one of the box sides -- works good
  ivec.set(0, 1, 0);
  jvec.set(0, 0, 1);
  switch(target_shape->type) {
    case rai::ST_cylinder:
      target = ARR(0.);  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case rai::ST_box: {
      //jrel=target_shape->X;
      if(side==1) jvec.set(0, 1, 0);
      if(side==2) jvec.set(1, 0, 0);
      target = ARR(1.);  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  c = MP.addTask("upAlign",
                 new TM_Default(TMT_vecAlign, MP.world, "graspCenter", ivec, target_shape->name, jvec));
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, alignmentPrec, NoArr, -1., .8);
  //test current state: flip if necessary
  c->feat.phi(initial, NoArr, MP.world);
  if(initial(0)<0.)((TM_Default*)&c->feat)->ivec.set(0., -1., 0.);   //flip vector to become positive

  if(phase==0) return;

  //-- finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
  {"tip1Shape", "tip2Shape", "tip3Shape"}, MP.world.shapes);
  shapes.append(shapeId); shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2, 3); shapes = ~shapes;
  c = MP.addTask("graspContacts", new TM_Proxy(TMT_vectorP, shapes, .05, true));
  double grip=.8; //specifies the desired proxy value
  target = ARR(grip, grip, grip);
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, fingerDistPrec, ARR(0., 0., 0.), 0., 0.8);
  for(uint t=0; t<=T; t++) {  //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    if(5*t<4*T) c->target[t]()=0.;
    else c->target[t]() = (grip*double(5*t-4*T))/T;
  }

  //-- collisions with other objects
  shapes = {shapeId};
  c = MP.addTask("otherCollisions", new TM_Proxy(TMT_allExceptListedP, shapes, .04, true));
  target = ARR(0.);
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, colPrec, target, colPrec);
  c->feat.phi(initial, NoArr, MP.world);
  if(initial(0)>0.) {  //we are in collision/proximity -> depart slowly
    double a=initial(0);
    for(uint t=0; t<=T/5; t++)
      c->target[t]() = a*double(T-5*t)/T;
  }

  //-- opposing fingers
  c = MP.addTask("oppose12",
                 new TM_Default(TMT_vecAlign, MP.world, "tipNormal1", NoVector, "tipNormal2", NoVector));
  target = ARR(-1.);
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, oppositionPrec, ARR(0., 0., 0.), 0., 0.8);
  //M.setInterpolatingCosts(c, KOMO::constFinalMid, target, oppositionPrec);

  c = MP.addTask("oppose13",
                 new TM_Default(TMT_vecAlign, MP.world, "tipNormal1", NoVector, "tipNormal3", NoVector));
  target = ARR(-1.);
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, oppositionPrec);

  //RAI_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");

  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  c = MP.addTask("limits",
                 new TM_qLimits(limits));
  target=0.;
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, limPrec, target, limPrec);

  //-- homing
  c = MP.addTask("qitself",
                 new TM_qItself());
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, zeroQPrec, target, zeroQPrec);
}

void setGraspGoals_PR2(KOMO& MP, uint T, uint shapeId, uint side, uint phase) {
  MP.setState(MP.x0, MP.v0);;

  //load parameters only once!
  double positionPrec = rai::getParameter<double>("graspPlanPositionPrec");
  double alignmentPrec = rai::getParameter<double>("graspPlanAlignmentPrec");
  double fingerDistPrec = rai::getParameter<double>("graspPlanFingerDistPrec");
  double colPrec = rai::getParameter<double>("graspPlanColPrec");
  double limPrec = rai::getParameter<double>("graspPlanLimPrec");
  double zeroQPrec = rai::getParameter<double>("graspPlanZeroQPrec");

  //set the time horizon
  CHECK_EQ(T, MP.T, "");

  //delete all previous variables
  MP.tasks.clear();

  //activate collision testing with target shape
  rai::Shape* target_shape = MP.world.shapes(shapeId);
  target_shape->cont=true;
  MP.world.swift().initActivations(MP.world);

  //
  arr target, initial;
  rai::Vector ivec, jvec;

  //general target
  target = conv_vec2arr(target_shape->X.pos);

  //-- graspCenter -> predefined point (xtarget)
  Task* c;
  c = MP.addTask("graspCenter",
                 new TM_Default(TMT_pos, MP.world, "graspCenter"));
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, positionPrec, NoArr, -1., .8);

  //-- align either with cylinder axis or one of the box sides -- works good
  ivec.set(0, 1, 0); //we want to align the y-axis of the hand with something
  jvec.set(0, 0, 1);
  switch(target_shape->type) {
    case rai::ST_cylinder:
      target = ARR(0.);  //y-axis of m9 is orthogonal to world z-axis (tricky :-) )
      break;
    case rai::ST_mesh:
      target = ARR(0.);  //works for simple cylinder-like objects
      break;
    case rai::ST_box: {
      //jrel=target_shape->X;
      //  side =1; //! Hack for PR2
      if(side==1) jvec.set(0, 1, 0);
      if(side==2) jvec.set(1, 0, 0);
      target = ARR(1.);  //y-axis of m9 is aligned with one of the 3 sides of the cube
    } break;
    default: NIY;
  }
  c = MP.addTask("upAlign",
                 new TM_Default(TMT_vecAlign, MP.world, "graspCenter", ivec, target_shape->name, jvec));
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, alignmentPrec, NoArr, -1., .8);
  //test current state: flip if necessary
  c->feat.phi(initial, NoArr, MP.world);
  if(initial(0)<0.)((TM_Default*)&c->feat)->ivec.set(0, -1, 0); //flip vector to become positive

  if(phase==0) return;

  //-- finger tips close to surface : using ProxyTaskVariable
  uintA shapes = stringListToShapeIndices(
  {"l_gripper_l_finger_tip_link_0", "l_gripper_r_finger_tip_link_0"}, MP.world.shapes);
  shapes.append(shapeId); shapes.append(shapeId);
  shapes.reshape(2, 2); shapes = ~shapes;
  c = MP.addTask("graspContacts", new TM_Proxy(TMT_vectorP, shapes, .1, false));
  for(rai::Shape* s: MP.world.shapes) cout <<' ' <<s->name;
  double grip=.98; //specifies the desired proxy value
  target = ARR(grip, grip);
  MP.setInterpolatingCosts(c, KOMO::early_restConst,
                           target, fingerDistPrec, ARR(0., 0.), 0., 0.8);
  for(uint t=.8*T; t<=T; t++) {  //interpolation: 0 up to 4/5 of the trajectory, then interpolating in the last 1/5
    double a=double(t-.8*T)/(.2*T);
    c->target[t]() = .7*(1.-a) + grip*a;
  }

#if 1
  //-- collisions with other objects
  shapes = {shapeId};
  c = MP.addTask("otherCollisions",
                 new TM_Proxy(TMT_allExceptListedP, shapes, .04, true));
  target = ARR(0.);
  c->setCostSpecs(0, MP.T, NoArr, colPrec);
//  arr initial;
  c->feat.phi(initial, NoArr, MP.world);
  if(initial(0)>0.) { //we are in collision/proximity -> depart slowly
    for(uint t=0; t<=T/5; t++) {
      double a = double(T-5*t)/T;
      c->target[t]() = a*initial(0);
    }
  }
#endif

  //-- homing
  c = MP.addTask("qitself",
                 new TM_qItself());
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, zeroQPrec, target, zeroQPrec);

  return;

  //RAI_MSG("TODO: fingers should be in relaxed position, or aligned with surface (otherwise they remain ``hooked'' as in previous posture)");

  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  c = MP.addTask("limits",
                 new TM_qLimits(limits));
  target=0.;
  MP.setInterpolatingCosts(c, KOMO::final_restConst, target, limPrec, target, limPrec);

}
#else
void setGraspGoals_PR2(KOMO& MP, uint T, uint shapeId, uint side, uint phase) {
  NIY
}
#endif

void reattachShape(rai::Configuration& ors, SwiftInterface* swift, const char* objShape, const char* toBody);

#if 0
void setPlaceGoals(KOMO& MP, uint T, uint shapeId, int belowToShapeId, const arr& locationTo) {
  CHECK(belowToShapeId == -1 || &locationTo == nullptr, "Only one thing at a time");
  MP.setState(MP.x0, MP.v0);;

  double midPrec          = rai::getParameter<double>("placeMidPrec");
  double alignmentPrec    = rai::getParameter<double>("placeAlignmentPrec");
  double limPrec          = rai::getParameter<double>("placePlanLimPrec");
  double colPrec          = rai::getParameter<double>("placePlanColPrec");
  double zeroQPrec        = rai::getParameter<double>("placePlanZeroQPrec");
  double positionPrec     = rai::getParameter<double>("placePositionPrec");
  double upDownVelocity   = rai::getParameter<double>("placeUpDownVelocity");
  double upDownVelocityPrec = rai::getParameter<double>("placeUpDownVelocityPrec");

  //set the time horizon
  CHECK_EQ(T, MP.T, "");

  //deactivate all variables
  MP.activateAllTaskCosts(false);

  //activate collision testing with target shape
  rai::Shape* obj  = MP.world.shapes(shapeId);
  rai::Shape* onto = nullptr;
  if(belowToShapeId != -1)
    onto = MP.world.shapes(belowToShapeId);
  if(obj->body!=MP.world.getBodyByName("m9")) {
    reattachShape(MP.world, nullptr, obj->name, "m9");
  }
  CHECK_EQ(obj->body, MP.world.getBodyByName("m9"), "called planPlaceTrajectory without right object in hand");
  obj->cont=true;
  if(onto) onto->cont=false;
  MP.swift->initActivations(MP.world, 3); //the '4' means to deactivate collisions between object and fingers (which have joint parents on level 4)

  TaskVariable* V;

  //general target
  arr xtarget;
  if(onto) {
    xtarget = conv_vec2arr(onto->X.pos);
    xtarget(2) += .5*(onto->size(2)+obj->size(2))+.005; //above 'place' shape
  } else {
    xtarget = locationTo;
  }

  //endeff
  V = new DefaultTaskVariable("graspCenter", MP.world, posTVT, "graspCenter", nullptr, NoArr);
  ((DefaultTaskVariable*)V)->irel = obj->rel;
  V->updateState(MP.world);
  V->y_target = xtarget;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, positionPrec, 0., 0.);
  //special: condition effector velocities:
  uint t, M=T/8;
  for(t=0; t<M; t++) {
    V -> v_trajectory[t]() = (1./M*t)*ARR(0., 0., upDownVelocity);
    V -> v_prec_trajectory(t) = upDownVelocityPrec;
  }
  for(t=T-M; t<T; t++) {
    V -> v_trajectory[t]() = (1./M*(T-t))*ARR(0., 0., -upDownVelocity); //0.2
    V -> v_prec_trajectory(t) = upDownVelocityPrec; // 1e1
  }
  MP.vars().append(V);

  //up1
  V = new DefaultTaskVariable("up1", MP.world, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel; ((DefaultTaskVariable*)V) -> irel.addRelativeRotationDeg(90, 1, 0, 0);
  V->updateState(MP.world);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  MP.vars().append(V);

  //up2
  V = new DefaultTaskVariable("up2", MP.world, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  ((DefaultTaskVariable*)V)->irel = obj->rel; ((DefaultTaskVariable*)V)-> irel.addRelativeRotationDeg(90, 0, 1, 0);
  V->updateState(MP.world);
  V->y_target = 0.;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, alignmentPrec, 0., 0.);
  MP.vars().append(V);

  //collisions except obj-from and obj-to
  uintA shapes = {shapeId, shapeId, belowToShapeId};
  V = new ProxyTaskVariable("otherCollisions", MP.world, TMT_allExceptListedP, shapes, .04, true);
  V->y_target = ARR(0.);  V->v_target = ARR(.0);
  V->y_prec = colPrec;
  V->setConstTargetsConstPrecisions(T);
  if(V->y(0)>0.) {  //we are in collision/proximity -> depart slowly
    double a=V->y(0);
    for(uint t=0; t<=T/5; t++)
      V->y_target[t]() = a*double(T-5*t)/T;
  }
  MP.vars().append(V);

  //col lim and relax
  //TODO: there are no collisions!
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //TODO: limits as parameter!
  V = new DefaultTaskVariable("limits", MP.world, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  MP.vars().append(V);
  V = new DefaultTaskVariable("qitself", MP.world, qItselfTVT, 0, 0, 0, 0, 0);
  V->y_prec=zeroQPrec;
  V->y=0.;  V->y_target=V->y;  V->v=0.;  V->v_target=V->v;  V->setConstTargetsConstPrecisions(T);
  MP.vars().append(V);
}

void setHomingGoals(KOMO& M, uint T) {
  M.setState(M.x0, M.v0);;

  //deactivate all variables
  M.activateAllTaskCosts(false);

  M.swift->initActivations(*M.ors);

  TaskVariable* V;

  //general target
  double midPrec, endPrec, limPrec, colPrec;
  rai::getParameter(midPrec, "homingPlanMidPrec");
  rai::getParameter(endPrec, "homingPlanEndPrec");
  rai::getParameter(limPrec, "homingPlanLimPrec");
  rai::getParameter(colPrec, "homingPlanColPrec");

  //-- limits
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  V = new DefaultTaskVariable("limits", *M.ors, qLimitsTVT, 0, 0, 0, 0, limits);
  V->y=0.;  V->y_target=0.;  V->y_prec=limPrec;  V->setConstTargetsConstPrecisions(T);
  M.vars().append(V);

  //-- standard collisions
  double margin = .05;
  V = new DefaultTaskVariable("collision", *M.ors, collTVT, 0, 0, 0, 0, ARR(margin));
  V->y=0.;  V->y_target=0.;  V->y_prec=colPrec;  V->setConstTargetsConstPrecisions(T);
  M.vars().append(V);

  //-- qitself
  V = new DefaultTaskVariable("qitself", *M.ors, qItselfTVT, 0, 0, 0, 0, 0);
  V->updateState(*M.ors);
  V->y_target.resizeAs(V->y);  V->y_target.setZero();
  V->v=0.;  V->v_target=V->v;
  V->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 0.);
  M.vars().append(V);

}
#endif

double keyframeOptimizer(arr& x, KOMO& MP, bool x_is_initialized, uint verbose) {

//  MotionProblem_EndPoseFunction MF(MP);

  if(!x_is_initialized) x=MP.world.q;

  double cost;

  NIY;
//  optNewton(x, Convert(MF), OPT(fmin_return=&cost, verbose=verbose, stopIters=200, damping=1e-0, maxStep=.5, stopTolerance=1e-2));

  return cost;
}

void interpolate_trajectory(arr& q, const arr& q0, const arr& qT, uint T) {
  q.resize(T+1, q0.N);
  for(uint t=0; t<=T; t++) {
    double a=double(t)/T;
    q[t] = (1.-a)*q0 + a*qT;
  }
}
