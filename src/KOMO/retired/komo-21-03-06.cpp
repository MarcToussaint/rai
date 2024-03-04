/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void KOMO::addSwitch_mode(SkeletonSymbol prevMode, SkeletonSymbol newMode, double time, double endTime, const char* prevFrom, const char* from, const char* to) {
  //-- creating a stable kinematic linking
  if(newMode==SY_stable || newMode==SY_stableOn) {
    // create the kinematic switch
    if(newMode==SY_stable) {
      addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    } else { //SY_stableOn
      Transformation rel = 0;
      rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
      addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    }

    // ensure the DOF is constant throughout its existance
    if((endTime<0. && stepsPerPhase*time<T) || stepsPerPhase*endTime>stepsPerPhase*time+1) {
      addObjective({time, endTime}, make_shared<F_qZeroVel>(), {to}, OT_eq, {1e1}, NoArr, 1, +1, -1);
      //      addObjective({time, endTime}, FS_poseRel, {from, to}, OT_eq, {1e1}, NoArr, 1, +1, -1);
    }

    //-- no relative jump at end
    //    if(endTime>0.) addObjective({endTime, endTime}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);

    if(prevMode==SY_initial || prevMode==SY_stable || prevMode==SY_stableOn) {
      //-- no acceleration at start: +0 INCLUDES (x-2, x-1, x0)
      //        if(k_order>1) addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);

      if(k_order>1) {
#if 1   //no jump: zero pose velocity
        if(prevFrom) addObjective({time}, FS_poseRel, {prevFrom, to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e0}, NoArr, 1, +0, +1);
#elif 1 //no jump: zero pose acceleration
        addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 2, +0, +1);
#elif 1 //no jump: zero linang velocity
        if(prevFrom) addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e2}, NoArr, 2, +0, +1);
        else addObjective({time}, make_shared<TM_LinAngVel>(world, to), OT_eq, {1e1}, NoArr, 1, +0, +1);
#endif
      } else {
//        addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
        if(prevFrom) addObjective({time}, FS_poseRel, {to, prevFrom}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    } else {
      //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
      if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {to}, OT_eq, {1e2}, NoArr, 2, +1, +1);
//      else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
      else {
        if(prevFrom) addObjective({time}, FS_poseRel, {to, prevFrom}, OT_eq, {1e2}, NoArr, 1, 0, 0);
        else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
//        addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);
      }
    }
  }

  if(newMode==SY_dynamic) {
    addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    //new contacts don't exist in step [-1], so we rather impose only zero acceleration at [-2,-1,0]
    addObjective({time}, FS_pose, {to}, OT_eq, {1e0}, NoArr, k_order, +0, +0);
    //... and physics starting from [-1,0,+1], ... until [-3,-2,-1]
    addObjective({time, endTime}, make_shared<F_NewtonEuler>(), {to}, OT_eq, {1e0}, NoArr, k_order, +1, -1);
  }

  if(newMode==SY_dynamicTrans) {
    HALT("deprecated")
//    addSwitch({time}, true, JT_trans3, SWInit_copy, from, to);
//    addObjective({time, endTime}, make_shared<F_NewtonEuler>(true), {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
  }

  if(newMode==SY_dynamicOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
    addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
    if(k_order>=2) addObjective({time, endTime}, FS_pose, {to}, OT_eq, {3e1}, NoArr, k_order, +0, -1);
    //  else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
  }

  if(newMode==SY_quasiStatic) {
    addSwitch({time}, true, JT_free, SWInit_copy, from, to);
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(), {to}, OT_eq, {1e1}, NoArr, 1, +0, -1);
  }

  if(newMode==SY_quasiStaticOn) {
    Transformation rel = 0;
    rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
    addSwitch({time}, true, JT_transXYPhi, SWInit_copy, from, to, rel);
#if 0
    addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(world, to, 0., false), OT_eq, {1e2}, NoArr, 1, +0, -1);
#else
    //eq for 3DOFs only
    shared_ptr<Objective> o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(false), {to}, OT_eq, {1e2}, NoArr, 1, +0, -1);
    o->feat->scale=1e2 * arr({3, 6}, {
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1
    });
    //sos penalty of other forces
    o = addObjective({time, endTime}, make_shared<F_NewtonEuler_DampedVelocities>(false), {to}, OT_sos, {1e2}, NoArr, 1, +0, -1);
    o->feat->scale=1e1 * arr({3, 6}, {
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0
    });
#endif
//    addObjective({time, endTime}, make_shared<F_pushed>(world, to), OT_eq, {1e0}, NoArr, 1, +0, -1);

    //-- no acceleration at start: +1 EXCLUDES (x-2, x-1, x0), ASSUMPTION: this is a placement that can excert impact
    if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {to}, OT_eq, {1e2}, NoArr, 2, +0, +1);
//    else addObjective({time}, make_shared<TM_NoJumpFromParent>(world, to), OT_eq, {1e2}, NoArr, 1, 0, 0);
    else addObjective({time}, FS_pose, {to}, OT_eq, {1e2}, NoArr, 1, 0, 0);

  }

  if(newMode==SY_magicTrans) {
    addSwitch({time}, true, JT_transZ, SWInit_copy, from, to);
    double sqrAccCost=0.;
    if(sqrAccCost>0.) {
      addObjective({time, endTime}, make_shared<F_LinAngVel>(), {to}, OT_sos, {sqrAccCost}, NoArr, 2);
    }
  }
}

void KOMO::setDiscreteOpt(uint k) {
  solver = rai::KS_sparse;
//  stepsPerPhase = 1;
//  T = k;
//  tau = 1.;
//  k_order = 1;
  setTiming(k, 1, 1., 1);
  addQuaternionNorms();
}

void KOMO::addSwitch_on(double time, const char* from, const char* to, bool copyInitialization) {
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(from)) + shapeSize(world.getFrame(to))));
  addSwitch({time}, true, JT_transXYPhi, (copyInitialization?SWInit_copy:SWInit_zero), from, to, rel);
}

void KOMO::activateCollisions(const char* s1, const char* s2) {
  if(!computeCollisions) return;
  Frame* sh1 = world.getFrame(s1);
  Frame* sh2 = world.getFrame(s2);
  if(sh1 && sh2) world.swift()->activate(sh1, sh2);
}

void KOMO::deactivateCollisions(const char* s1, const char* s2) {
  if(!computeCollisions) return;
  Frame* sh1 = world.getFrame(s1);
  Frame* sh2 = world.getFrame(s2);
  if(sh1 && sh2) world.swift()->deactivate(sh1, sh2);
  else LOG(-1) <<"not found:" <<s1 <<' ' <<s2;
}
/// standard place on a table
void KOMO_ext::setPlace(double time, const char* endeff, const char* object, const char* placeRef, int verbose) {
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" obj=" <<object <<" place=" <<placeRef <<endl;

//  if(stepsPerPhase>2){ //velocities down and up
//    if(endeff){
//      setTask(time-.15, time-.10, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//      setTask(time-.05, time+.05, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,0. }, 1e1, 1); //hold still
//      setTask(time+.10, time+.15, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,+.1}, 3e0, 1); //move up
//    }else{
////      setTask(time-.15, time, new TM_Default(TMT_pos, world, object), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//    }
//  }

  //place upright
//  setTask(time-.02, time, new TM_Default(TMT_vec, world, object, Vector_z), OT_sos, {0.,0.,1.}, 1e1);

  //place inside box support
//  setTask(time, time, new TM_StaticStability(world, placeRef, .01), OT_ineq);
  addObjective({time}, FS_aboveBox, {object, placeRef}, OT_ineq, {1e1});

  //connect object to placeRef
#if 0
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(object)) + shapeSize(world.getFrame(placeRef))));
//  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );
  addSwitch({time}, true, new KinematicSwitch(SW_effJoint, JT_transXYPhi, placeRef, object, world, SWInit_zero, 0, rel));

  addFlag(time, new Flag(FL_clear, world[object]->ID, 0, true));
  addFlag(time, new Flag(FL_zeroQVel, world[object]->ID, 0, true));
#else
  addSwitch_stableOn(time, -1., placeRef, object);
#endif
}

void setTasks(KOMO_ext& MP,
              Frame& endeff,
              Frame& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration) {

#if 1
  HALT("deprecated");
#else
  //-- parameters
  double posPrec = getParameter<double>("KOMO/moveTo/precision", 3e1);
  double colPrec = getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 3e0);
  double alignPrec = getParameter<double>("KOMO/moveTo/alignPrecision", 3e1);

  //-- set up the KOMO
  target.shape->cont=false; //turn off contact penalization with the target

//  MP.world.swift().initActivations(MP.world);
  //MP.world.view(false);

  MP.setTiming(1., getParameter<uint>("timeSteps", 50), getParameter<double>("duration", 5.));
  if(timeSteps>=0) MP.setTiming(1., timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task* t;

  t = MP.addTask("transitions", new TM_Transition(MP.world), OT_sos);
  if(timeSteps!=0) {
    t->feat->order=2; //make this an acceleration task!
  } else {
    t->feat->order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T-1, {0.}, 1e0);

  if(timeSteps!=0) {
    t = MP.addTask("final_vel", new TM_qItself(), OT_sos);
    t->feat->order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T-1, {0.}, zeroVelPrec);
  }

  if(colPrec<0) { //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), OT_ineq);
    t->setCostSpecs(0, MP.T-1, {0.}, 1.);
  } else { //cost term
    t = MP.addTask("collision", new TM_Proxy(TMT_allP, {}, margin), OT_sos);
    t->setCostSpecs(0, MP.T-1, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new TM_Default(TMT_pos, endeff.ID, NoVector, target.ID, NoVector), OT_sos);
  t->setCostSpecs(MP.T-1, MP.T-1, {0.}, posPrec);

  for(uint i=0; i<3; i++) if(whichAxesToAlign&(1<<i)) {
      Vector axis;
      axis.setZero();
      axis(i)=1.;
      t = MP.addTask(STRING("endeff_align_"<<i),
                     new TM_Default(TMT_vecAlign, endeff.ID, axis, target.ID, axis),
                     OT_sos);
      t->setCostSpecs(MP.T-1, MP.T-1, {1.}, alignPrec);
    }
#endif
}

#if 0
void KOMO::setState(const arr& x, const uintA& selectedVariablesOnly) {
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  if(!!selectedVariablesOnly) { //we only set some configurations: those listed in selectedConfigurationsOnly
    CHECK(selectedVariablesOnly.isSorted(), "");
    uint selCount=0;
    uint vCount=0;
    for(uint t=0; t<T; t++) {
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0; i<n; i++) {
        if(vCount == selectedVariablesOnly(selCount)) {
          configurations(s)->vars_activate(i);
          selCount++;
          if(selCount == selectedVariablesOnly.N) break;
        } else {
          configurations(s)->vars_deactivate(i);
        }
        vCount++;
      }
      if(selCount == selectedVariablesOnly.N) break;
    }
  } else {
    for(uint t=0; t<T; t++) {
      uint s = t+k_order;
      uint n = configurations(s)->vars_getNum();
      for(uint i=0; i<n; i++) {
        configurations(s)->vars_activate(i);
      }
    }
  }

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0; t<T; t++) {
    uint s = t+k_order;
    uint x_dim = dim_x(t);
    if(x_dim) {
      rai::timerRead(true);
      if(x.nd==1)  configurations(s)->setJointState(x({x_count, x_count+x_dim-1}));
      else         configurations(s)->setJointState(x[t]);
      timeKinematics += rai::timerRead(true);
      if(useSwift) {
#ifndef FCLmode
        configurations(s)->stepSwift();
#else
        configurations(s)->stepFcl();
#endif
        //configurations(s)->proxiesToContacts(1.1);
      }
      timeCollisions += rai::timerRead(true);
      x_count += x_dim;
    }
//    configurations(s)->checkConsistency();
  }
  CHECK_EQ(x_count, x.N, "");

  if(animateOptimization>0) {
    if(animateOptimization>1) {
//      if(animateOptimization>2)
//        cout <<report(false, true, true) <<endl;
      displayPath(true);
    } else {
      displayPath(false);
    }
//    komo.plotPhaseTrajectory();
//    rai::wait();
  }
}
#endif

void KOMO_ext::useJointGroups(const StringA& groupNames, bool notThese) {
  world.selectJointsByGroup(groupNames, notThese);

  world.optimizeTree();
  world.getJointState();

//  world.meldFixedJoints();
//  world.removeUselessBodies();

//  FILE("z.komo.model") <<world;
}

void KOMO_ext::setKS_slider(double time, double endTime, bool before, const char* obj, const char* slider, const char* table) {
  //disconnect object from grasp ref
//  setKinematicSwitch(time, before, "delete", nullptr, obj);

  //the two slider objects
  String slidera = STRING(slider <<'a');
  String sliderb = STRING(slider <<'b');

  Transformation rel = 0;
  rel.addRelativeTranslation(0., 0., .5*(shapeSize(world.getFrame(obj)) + shapeSize(world.getFrame(table))));

//  setKinematicSwitch(time, true, "transXYPhiZero", table, slidera, rel);
//  setKinematicSwitch(time, true, "hingeZZero", sliderb, obj);
  addSwitch({time}, true, JT_transXYPhi, SWInit_zero, table, slidera, rel);
  addSwitch({time}, true, JT_hingeZ, SWInit_zero, sliderb, obj);

  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {slidera}, OT_eq, {3e1}, NoArr, 1, +1, +0);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {obj}, OT_eq, {3e1}, NoArr, 1, +1, -1);
  addObjective({time}, make_shared<F_LinAngVel>(), {obj}, OT_eq, {1e2}, NoArr, 1);

//  setKinematicSwitch(time, before, "sliderMechanism", table, obj, rel );

//  if(!actuated)
//    setKinematicSwitch(time, before, "hingeZZero", slider, obj, rel );
//  else
  //    setKinematicSwitch(time, before, "transXActuated", slider, obj, rel );
}

void KOMO_ext::setHoming(double startTime, double endTime, double prec, const char* keyword) {
  uintA bodies;
  Joint* j;
  for(Frame* f:world.frames) if((j=f->joint) && j->qDim()>0 && (!keyword || f->ats[keyword])) bodies.append(f->ID);
//  cout <<"HOMING: "; for(uint i:bodies) cout <<' ' <<world.frames(i)->name;  cout <<endl;
  addObjective({startTime, endTime}, make_shared<F_qItself>(bodies, true), {}, OT_sos, {prec}, NoArr); //world.q, prec);
}

//void KOMO_ext::setSquaredQAccelerations(double startTime, double endTime, double prec) {
//  CHECK_GE(k_order, 2,"");
//  addObjective({startTime, endTime}, make_shared<TM_Transition>(world), OT_sos, {}, NoArrprec);
//}

void KOMO_ext::setHoldStill(double startTime, double endTime, const char* shape, double prec) {
  Frame* s = world.getFrame(shape);
  addObjective({startTime, endTime}, make_shared<F_qItself>(uintA{s->ID}), {}, OT_sos, {prec}, NoArr, 1);
}

void KOMO_ext::setPosition(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<F_PositionDiff>(), {shape, shapeRel}, type, {prec}, target);
}

void KOMO_ext::setOrientation(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
//  setTask(startTime, endTime, new TM_Align(world, shape, shapeRel), type, target, prec);
  addObjective({startTime, endTime}, make_shared<F_QuaternionDiff>(), {shape, shapeRel}, type, {prec}, target);
}

void KOMO_ext::setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, make_shared<F_PositionDiff>(), {shape, shapeRel}, type, {prec}, target, 1);
}

void KOMO_ext::setLastTaskToBeVelocity() {
  objectives.last()->feat->order = 1; //set to be velocity!
}

void KOMO_ext::setImpact(double time, const char* a, const char* b) {
  add_touch(time, time, a, b);
  HALT("obsolete");
//  add_impulse(time, a, b);
}

void KOMO_ext::setOverTheEdge(double time, const char* object, const char* from, double margin) {
//  double negMargin = margin + .5*shapeSize(world.getFrame(object), 0); //how much outside the bounding box?
  NIY;
//  addObjective({time, time+.5},
//               make_shared<F_Max>(make_shared<TM_AboveBox>(world, object, from, -negMargin), true), //this is the max selection -- only one of the four numbers need to be outside the BB
//               OT_ineq, {3e0}); //NOTE: usually this is an inequality constraint <0; here we say this should be zero for a negative margin (->outside support)
}

void KOMO_ext::setInertialMotion(double startTime, double endTime, const char* object, const char* base, double g, double c) {
  addSwitch({startTime}, true, JT_trans3, SWInit_zero, base, object);
//  setFlag(time, new Flag(FT_gravityAcc, world[object]->ID, 0, true),+1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
//  setFlag(startTime, new Flag(FL_noQControlCosts, world[object]->ID, 0, true), +2);
//  setFlag(endTime, new Flag(FL_noQControlCosts, world[object]->ID, 0, true, false), +1);
  if(k_order>=2) {
    NIY;
//    setTask(startTime, endTime, new TM_InertialMotion(world, object, g, c), OT_sos, {}, 1e1, 2);
  }
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO_ext::setGrasp(double time, double endTime, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift) {
  if(verbose>0) cout <<"KOMO_setGrasp t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;
  //  String& endeffRef = world.getFrameByName(graspRef)->body->inLinks.first()->from->shapes.first()->name;

  //-- position the hand & graspRef
  //hand upright
  //  setTask(time, time, new TM_Default(TMT_vec, world, endeffRef, Vector_z), OT_sos, {0.,0.,1.}, weightFromTop);

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TM_Default(TMT_posDiff, world, endeffRef, NoVector, object, NoVector), OT_eq, NoArr, 3e1);

  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TM_Default(TMT_vecAlign, world, endeffRef, Vector_x, object, Vector_x), OT_sos, NoArr, 3e0);
  //hand grip axis orthogonal to object length axis
//  setTask(time, time, new TM_Default(TMT_vecAlign, world, endeffRef, Vector_y, object, Vector_x), OT_sos, {-1.}, 3e0);

  //hand touches object
//  Shape *endeffShape = world.getFrameByName(endeffRef)->body->shapes.first();
//  setTask(time, time, new TM_GJK(endeffShape, world.getFrameByName(object), false), OT_eq, NoArr, 3e1);

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", nullptr, object);
  //connect graspRef with object
#if 0
  setKinematicSwitch(time, true, new KinematicSwitch(SW_effJoint, JT_quatBall, endeffRef, object, world));
  setKinematicSwitch(time, true, new KinematicSwitch(SW_insertEffJoint, JT_trans3, nullptr, object, world));
  setTask(time, time, new TM_InsideBox(world, endeffRef, NoVector, object), OT_ineq, NoArr, 1e1);
#else
//  addSwitch({time}, true, new KinematicSwitch(SW_effJoint, JT_free, endeffRef, object, world));
  addSwitch_stable(time, endTime, 0, endeffRef, object);
  addObjective({time}, make_shared<F_InsideBox>(), {endeffRef, object}, OT_ineq, {1e1});
//  setTouch(time, time, endeffRef, object);
#endif

//  if(stepsPerPhase>2 && timeToLift>0.){ //velocities down and up
//    setTask(time-timeToLift, time-2.*timeToLift/3, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,-.1}, 1e0, 1); //move down
//    setTask(time-timeToLift/3,  time+timeToLift/3, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,0.}, 3e0, 1); //move down
//    setTask(time+2.*timeToLift/3, time+timeToLift, new TM_Default(TMT_pos, world, endeffRef), OT_sos, {0.,0.,.1}, 1e0, 1); // move up
//  }

//  setFlag(time, new Flag(FL_clear, world[object]->ID, 0, true));
//  setFlag(time, new Flag(FL_zeroQVel, world[object]->ID, 0, true));
//  setFlag(time, new Flag(FL_kinematic, world[object]->getUpwardLink()->ID, 0, true));
}

/// a standard pick up: lower-attached-lift; centered, from top
void KOMO_ext::setGraspStick(double time, const char* endeffRef, const char* object, int verbose, double weightFromTop, double timeToLift) {
  if(verbose>0) cout <<"KOMO_setGraspStick t=" <<time <<" endeff=" <<endeffRef <<" obj=" <<object <<endl;

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", nullptr, object);

  //connect graspRef with object
  addSwitch({time}, true, JT_quatBall, SWInit_zero, endeffRef, object);
  HALT("deprecated"); //addSwitch({time}, true, "insert_transX", nullptr, object);
//  setTask(time, time,
//          new TM_LinTrans(
//              new TM_Default(TMT_posDiff, world, endeffRef, NoVector, object, NoVector),
//              arr(2,3,{0,1,0,0,0,1}), {}),
//          OT_eq, NoArr, 3e1);
  addObjective({time}, make_shared<F_InsideBox>(), {endeffRef, object}, OT_ineq, {1e1});

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({time-timeToLift, time}, make_shared<F_Position>(), {endeffRef}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({time, time+timeToLift}, make_shared<F_Position>(), {object}, OT_sos, {3e0}, {0., 0., .1}, 1); // move up
  }
}

/// place with a specific relative pose -> no effective DOFs!
void KOMO_ext::setPlaceFixed(double time, const char* endeff, const char* object, const char* placeRef, const Transformation& relPose, int verbose) {
  if(verbose>0) cout <<"KOMO_setPlace t=" <<time <<" endeff=" <<endeff <<" obj=" <<object <<" place=" <<placeRef <<endl;

  //connect object to table
  addSwitch({time}, true, JT_rigid, SWInit_zero, placeRef, object, relPose);

  if(stepsPerPhase>2) { //velocities down and up
    if(endeff) {
      addObjective({time-.15, time-.10}, make_shared<F_Position>(), {endeff}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
      addObjective({time-.05, time+.05}, make_shared<F_Position>(), {endeff}, OT_sos, {1e1}, {0., 0., 0.}, 1); //hold still
      addObjective({time+.10, time+.15}, make_shared<F_Position>(), {endeff}, OT_sos, {3e0}, {0., 0., +.1}, 1); //move up
    }
  }
}

/// switch attachemend (-> ball eDOF)
void KOMO_ext::setHandover(double time, const char* oldHolder, const char* object, const char* newHolder, int verbose) {
#if 1
  setGrasp(time, -1., newHolder, object, verbose, -1., -1.);
#else
  if(verbose>0) cout <<"KOMO_setHandover t=" <<time <<" oldHolder=" <<oldHolder <<" obj=" <<object <<" newHolder=" <<newHolder <<endl;

  //hand center at object center (could be replaced by touch)

  //disconnect object from table
//  setKinematicSwitch(time, true, "delete", oldHolder, object);
  //connect graspRef with object
#if 0
  setKinematicSwitch(time, true, "ballZero", newHolder, object); //why does this sometimes lead to worse motions?
#else
  setKinematicSwitch(time, true, "freeZero", newHolder, object);
  setTask(time, time, new TM_Default(TMT_posDiff, world, newHolder, NoVector, object, NoVector), OT_eq, NoArr, 3e1);
#endif

  if(stepsPerPhase>2) { //velocities: no motion
    setTask(time-.15, time+.15, new TM_Default(TMT_pos, world, object), OT_sos, {0., 0., 0.}, 3e0, 1); // no motion
  }
#endif
}

void KOMO_ext::setPush(double startTime, double endTime, const char* stick, const char* object, const char* table, int verbose) {
  if(verbose>0) cout <<"KOMO_setPush t=" <<startTime <<" stick=" <<stick <<" object=" <<object <<" table=" <<table <<endl;

#if 1
  //stick normal alignes with slider direction
  addObjective({startTime, endTime}, make_shared<F_ScalarProduct>(-Vector_y, Vector_x), {stick, "slider1b"}, OT_sos, {1e1}, {1.});
  //stick horizontal is orthogonal to world vertical
//  setTask(startTime, endTime, new TM_Default(TMT_vecAlign, world, stick, Vector_x, nullptr, Vector_z), OT_sos, {0.}, 1e1);
  add_touch(startTime, endTime, stick, table);

  double dist = .05; //.5*shapeSize(world.getFrame(object), 0)+.01;
  addObjective({startTime, endTime}, make_shared<F_InsideBox>(), {"slider1b", stick}, OT_ineq);
  HALT("ivec = Vector(dist, .0, .0) is missing");
  Vector(dist, .0, .0),
//  setTask(startTime, endTime, new TM_Default(TMT_posDiff, world, stick, NoVector, "slider1b", {dist, .0, .0}), OT_sos, {}, 1e1);
#else
  setTouch(startTime, endTime, stick, object);
#endif

         setKS_slider(startTime, endTime, true, object, "slider1", table);

  addObjective({startTime, endTime-.1}, FS_aboveBox, {object, table}, OT_ineq, {1e1});

#if 0
  //connect object to placeRef
  Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(world.getFrame(object)) + shapeSize(world.getFrame(table))));
  addSwitch({endTime}, true, "transXYPhiZero", table, object, rel);
//  auto *o = addObjective({startTime, endTime}, make_shared<TM_ZeroQVel>(world, object), OT_eq, {3e1}, NoArr, 1, +1);
//  o->prec(-1)=o->prec(-2)=0.;
#endif

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({startTime-.3, startTime-.1}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., -.1}, 1); //move down
    addObjective({startTime-.05, startTime-.0}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.0, endTime+.05}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., .1}, 1); // move up
  }
}

void KOMO_ext::setGraspSlide(double time, const char* endeff, const char* object, const char* placeRef, int verbose) {

  double startTime = time;
  double endTime = time+1.;

  if(verbose>0) cout <<"KOMO_setSlide t=" <<startTime <<" endeff=" <<endeff <<" obj=" <<object <<endl;

  //-- grasp part
  //hand upright
//  setTask(startTime, startTime, new TM_Default(TMT_vec, world, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e-2);

  //disconnect object from table
//  setKinematicSwitch(startTime, true, "delete", placeRef, object);
  //connect graspRef with object
//  setKinematicSwitch(startTime, true, "ballZero", endeff, object);
//  setKinematicSwitch(time, true, "insert_trans3", nullptr, object);
//  setTask(time, time, new TM_InsideBox(world, endeff, NoVector, object), OT_ineq, NoArr, 1e1);

//  addSwitch_stable(startTime, endTime+1., endeff, object);
  addSwitch({startTime}, true, JT_free, SWInit_zero, endeff, object);
  addObjective({time, endTime}, make_shared<F_qZeroVel>(), {object}, OT_eq, {3e1}, NoArr, 1, +1, -1);
  if(k_order>1) addObjective({time}, make_shared<F_LinAngVel>(), {object}, OT_eq, {1e2}, NoArr, 2, 0);
  else addObjective({time}, make_shared<F_NoJumpFromParent_OBSOLETE>(), {object}, OT_eq, {1e2}, NoArr, 1, 0, 0);

  add_touch(startTime, startTime, endeff, object);

  //-- place part
  //place inside box support
//  setTask(endTime, endTime, new TM_AboveBox(world, object, placeRef), OT_ineq, NoArr, 1e1);
  add_aboveBox(endTime, endTime, object, placeRef);

  //disconnect object from grasp ref
//  setKinematicSwitch(endTime, true, "delete", endeff, object);

  //connect object to table
//  Transformation rel = 0;
//  rel.pos.set(0,0, h);
//  setKinematicSwitch(endTime, true, "transXYPhiZero", placeRef, object, rel );

  //-- slide constraints!
  //keep height of object above table
//  double h = .5*(shapeSize(world.getFrame(object)) + shapeSize(world.getFrame(placeRef)));
  HALT("TODO: fix syntax:")
//  addObjective(startTime, endTime,
//          make_shared<TM_Default>(TMT_posDiff, world, object, NoVector, placeRef),
//          OT_sos, arr{h}, ~arr{0,0,1e1});
  //keep object vertial
  addObjective({startTime, endTime},
               make_shared<F_VectorDiff>(Vector_z, Vector_z), {object, placeRef}, OT_sos, {1e1});

//  if(stepsPerPhase>2){ //velocities down and up
//    setTask(startTime-.15, startTime, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,-.1}, 3e0, 1); //move down
//    setTask(endTime, endTime+.15, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,.1}, 3e0, 1); // move up
//  }
}

void KOMO_ext::setSlideAlong(double time, const char* stick, const char* object, const char* wall, int verbose) {
  if(verbose>0) cout <<"KOMO_setSlideAlong t=" <<time <<" obj=" <<object<<" wall=" <<wall <<endl;

  double endTime = time+1.;

  //stick normal alignes with slider direction
  addObjective({time, time+1.}, make_shared<F_ScalarProduct>(-Vector_y, Vector_x), {stick, object}, OT_sos, {1e0}, {1.});
  //stick horizontal is orthogonal to world vertical
  addObjective({time, time+1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_z), {stick}, OT_sos, {1e1}, {0.});

//  double dist = .5*shapeSize(world.getFrame(object), 0)+.01;
  addObjective({time, time+1.}, make_shared<F_InsideBox>(), {object, stick}, OT_ineq);
  HALT("ivec = Vector(dist, .0, .0), is missing");

  add_touch(time, time+1., stick, wall);

  //    //disconnect object from table
  //    setKinematicSwitch(time, true, "delete", nullptr, object);
  //    //connect graspRef with object
  //    setKinematicSwitch(startTime, true, "ballZero", endeff, object);

  Transformation rel = 0;
  rel.rot.setDeg(-90, {1, 0, 0});
  rel.pos.set(0, -.5*(shapeSize(world.getFrame(wall), 1) - shapeSize(world.getFrame(object))), +.5*(shapeSize(world.getFrame(wall), 2) + shapeSize(world.getFrame(object), 1)));
  addSwitch({time}, true, JT_transX, SWInit_zero, wall, object);
  HALT("deprecated")//addSwitch({time}, true, new KinematicSwitch(SW_insertEffJoint, JT_transZ, nullptr, object, world, SWInit_zero, 0, rel));
  //    setKinematicSwitch(time, true, "insert_trans3", nullptr, object);
  //    setTask(time, time, new TM_InsideBox(world, endeff, NoVector, object), OT_ineq, NoArr, 1e1);

  if(stepsPerPhase>2) { //velocities down and up
    addObjective({endTime+.0, endTime+.05}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., 0}, 1); //hold still
    addObjective({endTime+.1, endTime+.3}, make_shared<F_Position>(), {stick}, OT_sos, {3e0}, {0., 0., .05}, 1); // move up
  }
}

void KOMO_ext::setDropEdgeFixed(double time, const char* object, const char* to, const Transformation& relFrom, const Transformation& relTo, int verbose) {

  //disconnect object from anything
//  setKinematicSwitch(time, true, "delete", nullptr, object);

  //connect to world with lift
//  setKinematicSwitch(time, true, "JT_trans3", "world", object);

  addSwitch({time}, true, JT_hingeX, SWInit_zero, to, object, relFrom, relTo);
//  setKinematicSwitch(time, true, new KinematicSwitch(insertActuated, JT_transZ, nullptr, object, world, 0));
//  setKinematicSwitch(time, true, new KinematicSwitch(SW_insertEffJoint, JT_trans3, nullptr, object, world, 0));

//  if(stepsPerPhase>2){ //velocities down and up
//    setTask(time, time+.2, new TM_Default(TMT_pos, world, object), OT_sos, {0.,0.,-.1}, 3e0, 1); // move down
//  }
}

void KOMO_ext::setAttach(double time, const char* endeff, const char* object1, const char* object2, Transformation& rel, int verbose) {
  if(verbose>0) cout <<"KOMO_setAttach t=" <<time <<" endeff=" <<endeff <<" obj1=" <<object1 <<" obj2=" <<object2 <<endl;

  //hand center at object center (could be replaced by touch)
//  setTask(time, time, new TM_Default(TMT_pos, world, object2, NoVector, object1, NoVector), OT_sos, rel.pos.getArr(), 3e1);
//  setTask(time, time, new TM_Default(TMT_quatDiff, world, object2, NoVector, object1, NoVector), OT_sos, conv_quat2arr(rel.rot), 3e1);

//  setTask(time, time, new TM_Default(TMT_vecAlign, world, newHolder, Vector_y, object, Vector_x), OT_sos, {-1.}, 3e0);

  //disconnect object from grasp ref
//  setKinematicSwitch(time, true, "delete", endeff, object2);

//  Transformation rel = 0;
//  rel.addRelativeTranslation( 0., 0., .5*(shapeSize(world.getFrameByName(object)) + shapeSize(world.getFrameByName(placeRef))));
  addSwitch({time}, true, JT_rigid, SWInit_zero, object1, object2, rel);

}
void KOMO_ext::setFine_grasp(double time, const char* endeff, const char* object, double above, double gripSize, const char* gripper, const char* gripper2) {
  double t1=-.25; //time when gripper is positined above
  double t2=-.1;  //time when gripper is lowered
  double t3=-.05; //time when gripper is closed

  //position above
  addObjective({time+t1, 1.}, make_shared<F_Vector>(Vector_z), {endeff}, OT_sos, {1e0}, {0., 0., 1.});
  addObjective({time+t1, t1}, make_shared<F_PositionDiff>(), {endeff, object}, OT_sos, {3e1}, {0., 0., above+.1});
  addObjective({time+t1, 1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_y), {endeff, object}, OT_sos, {3e0});
  addObjective({time+t1, 1.}, make_shared<F_ScalarProduct>(Vector_x, Vector_z), {endeff, object}, OT_sos, {3e0});
  //open gripper
  if(gripper)  addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), {}, OT_sos, {gripSize + .05});
  if(gripper2) addObjective({time+t1, .85}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), {}, OT_sos, {::asin((gripSize + .05)/(2.*.10))});
  //lower
  addObjective({time+t2, 1.}, make_shared<F_PositionDiff>(), {endeff, object}, OT_sos, {3e1}, {0., 0., above});
  //close gripper
  if(gripper)  addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper}), world), {}, OT_sos, {gripSize});
  if(gripper2) addObjective({time+t3, 1.}, make_shared<F_qItself>(F_qItself::byJointNames, StringA({gripper2}), world), {}, OT_sos, {::asin((gripSize)/(2.*.10))});
  setSlowAround(time, .05, 3e1);
}

/// translate a list of facts (typically facts in a FOL state) to LGP tasks
/*
void KOMO_ext::setAbstractTask(double phase, const Graph& facts, int verbose) {
//  CHECK_LE(phase, maxPhase,"");
//  listWrite(facts, cout,"\n");  cout <<endl;
  for(Node *n:facts) {
    if(!n->parents.N) continue;
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());
    double time=1.; //n->get<double>(); //komo tag needs to be double valued!
    if(n->keys.N==1 && n->keys.scalar() == "komo") {
      if(*symbols(0)=="grasp")                      setGrasp(phase+time, *symbols(1), *symbols(2), verbose);
      else if(*symbols(0)=="push")                  setPush(phase+time, phase+time+1., *symbols(1), *symbols(2), *symbols(3), verbose); //TODO: the +1. assumes pushes always have duration 1
      else if(*symbols(0)=="place" && symbols.N==3) setPlace(phase+time, nullptr, *symbols(1), *symbols(2), verbose);
      else if(*symbols(0)=="place" && symbols.N==4) setPlace(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);
      else if(*symbols(0)=="graspSlide")            setGraspSlide(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);
      else if(*symbols(0)=="handover")              setHandover(phase+time, *symbols(1), *symbols(2), *symbols(3), verbose);

      //elementary
      else if(*symbols(0)=="flagClear")  {} //           setFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true));
      else if(*symbols(0)=="touch")                 add_touch(phase+time, phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="lift")                  setLiftDownUp(phase+time, *symbols(1));
      else if(*symbols(0)=="impulse")               add_impulse(phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="inside")                add_insideBox(phase+time, phase+time, *symbols(1), *symbols(2));
      else if(*symbols(0)=="above")                 add_aboveBox(phase+time, phase+time+.9, *symbols(1), *symbols(2));
      else if(*symbols(0)=="stable")                addSwitch_stable(phase+time, phase+time+1., *symbols(1), *symbols(2));
      else if(*symbols(0)=="stableOn")              addSwitch_stableOn(phase+time, phase+time+1., *symbols(1), *symbols(2));
      else if(*symbols(0)=="dynamic")               addSwitch_dynamic(phase+time, phase+time+1., "base", *symbols(1));
      else if(*symbols(0)=="dynamicTrans")          addSwitch_dynamicTrans(phase+time, phase+time+1., "base", *symbols(1));
      else if(*symbols(0)=="dynamicOn")             addSwitch_dynamicOn(phase+time, phase+time+1., *symbols(1), *symbols(2));

      else if(*symbols(0)=="notAbove") {
        double margin = .05;
        double negMargin = margin + .5*shapeSize(world, *symbols(1), 0); //how much outside the bounding box?
        addObjective(phase+time, phase+time+.9,
                new TM_Max(new TM_AboveBox(world, *symbols(1),*symbols(2), -negMargin), true), //this is the max selection -- only one of the four numbers need to be outside the BB
                OT_ineq, {}, 3e0); //NOTE: usually this is an inequality constraint <0; here we say this should be zero for a negative margin (->outside support)
      } else if(*symbols(0)=="fricSlide") {
        Transformation rel = 0;
        rel.pos.set(0,0, .5*(shapeSize(world, *symbols(1)) + shapeSize(world, *symbols(2))));
        addSwitch(phase+time, false, JT_transXYPhi, SWInit_zero, *symbols(2), *symbols(1), rel);
        addFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true), +1);
        addFlag(phase+time, new Flag(FL_impulseExchange, world[*symbols(1)]->ID), 0);
        addFlag(phase+time, new Flag(FL_xPosVelCosts, world[*symbols(1)]->ID, 0, true), +1);
      } else if(*symbols(0)=="dynVert")               {
        addSwitch(phase+time, true, JT_transZ, SWInit_zero, *symbols(2), *symbols(1));
        HALT("deprecated")//addSwitch(phase+time, true, new KinematicSwitch(SW_insertEffJoint, JT_transXY, nullptr, *symbols(1), world));
        addFlag(phase+time, new Flag(FL_clear, world[*symbols(1)]->ID, 0, true), +1);
//        setFlag(phase+time, new Flag(FL_zeroAcc, world[*symbols(1)]->ID, 0, true), +1);
        addFlag(phase+time, new Flag(FL_xPosAccCosts, world[*symbols(1)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0

      } else HALT("UNKNOWN komo symbol: '" <<*symbols(0) <<"'");
    } else if(n->keys.N && n->keys.last().startsWith("komo")) {
      if(n->keys.last()=="komoSlideAlong") setSlideAlong(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      else if(n->keys.last()=="komoDrop") {
        if(symbols.N==2) setDrop(phase+time, *symbols(0), nullptr, *symbols(1), verbose);
        else setDrop(phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
      } else if(n->keys.last()=="komoThrow") {
//        setInertialMotion(phase+time, phase+time+1., *symbols(0), "base", -.1, 0.);
        addSwitch(phase+time, true, JT_trans3, SWInit_zero, "base", *symbols(0));
        addFlag(phase+time, new Flag(FL_gravityAcc, world[*symbols(0)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
      } else if(n->keys.last()=="komoHit") {
        setImpact(phase+time, *symbols(0), *symbols(1));
        if(symbols.N==2) {
          addSwitch(phase+time, true, JT_trans3, SWInit_zero, "base", *symbols(1));
          addFlag(phase+time, new Flag(FL_gravityAcc, world[*symbols(1)]->ID, 0, true), +1); //why +1: the kinematic switch triggers 'FixSwitchedObjects' to enforce acc 0 for time slide +0
        } else if(symbols.N==3) {
          //const char* bat = *symbols(0);
          const char* object = *symbols(1);
          const char* placeRef = *symbols(2);
          Transformation rel = 0;
          rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));

          addSwitch(phase+time, true, JT_transXYPhi, SWInit_zero, placeRef, object, rel);
          addFlag(phase+time, new Flag(FL_clear, world[object]->ID, 0, true));
          addFlag(phase+time, new Flag(FL_zeroAcc, world[object]->ID, 0, true));

//          setKinematicSwitch(phase+time, false, new KinematicSwitch(SW_actJoint, JT_transXYPhi, placeRef, bat, world, 0, rel));
//          setFlag(phase+time, new Flag(FL_clear, world[bat]->ID, 0, true), +1);
//          setFlag(phase+time, new Flag(FL_xPosVelCosts, world[bat]->ID, 0, true), +1);
        } else NIY;
      } else if(n->keys.last()=="komoAttach") {
        Node *attachableSymbol = facts["attachable"];
        CHECK(attachableSymbol!=nullptr,"");
        Node *attachableFact = facts.getEdge({attachableSymbol, n->parents(1), n->parents(2)});
        Transformation rel = attachableFact->get<Transformation>();
        setAttach(phase+time, *symbols(0), *symbols(1), *symbols(2), rel, verbose);
      } else HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}
*/
void KOMO_ext::setAlign(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, ObjectiveType type, const arr& target, double prec) {
#if 0
  String map;
  map <<"map=vecAlign ref1="<<shape;
  if(whichAxis) map <<" vec1=[" <<whichAxis <<']';
  if(shapeRel) map <<" ref2=" <<shapeRel <<" vec2=" <<;
  if(whichAxisRel) map <<" vec2=[" <<whichAxisRel <<']';
  setTask(startTime, endTime, map, type, target, prec);
#else
  addObjective({startTime, endTime}, make_shared<F_ScalarProduct>(Vector(whichAxis), Vector(whichAxisRel)), {shape, shapeRel}, type, {prec}, target);
#endif

}

void KOMO_ext::add_touch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type, const arr& target, double prec) {
  addObjective({startTime, endTime}, FS_pairCollision_negScalar, {shape1, shape2}, type, {prec}, target);
}

void KOMO_ext::add_aboveBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, FS_aboveBox, {shape1, shape2}, OT_ineq, {prec}, NoArr);
}

void KOMO_ext::add_insideBox(double startTime, double endTime, const char* shape1, const char* shape2, double prec) {
  addObjective({startTime, endTime}, make_shared<F_InsideBox>(), {shape1, shape2}, OT_ineq, {prec}, NoArr);
}

//void KOMO::add_impulse(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
////    setTask(time, time, new TM_ImpulsExchange(world, a, b), OT_sos, {}, 3e1, 2, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//  if(k_order>=2) {
//    addObjective({time}, make_shared<TM_ImpulsExchange>(world, shape1, shape2), type, {}, prec, 2, +1, +1); //+1 deltaStep indicates moved 1 time slot backward (to cover switch)
//    addFlag(time, new Flag(FL_impulseExchange, world[shape1]->ID), +0);
//    addFlag(time, new Flag(FL_impulseExchange, world[shape2]->ID), +0);
//  }
//}

void KOMO_ext::add_stable(double time, const char* shape1, const char* shape2, ObjectiveType type, double prec) {
  addObjective({time}, make_shared<F_PoseRel>(), {shape1, shape2}, type, {prec}, NoArr, 1, 0);
}
void KOMO_ext::setSquaredQAccVelHoming(double startTime, double endTime, double accPrec, double velPrec, double homingPrec, int deltaFromStep, int deltaToStep) {
  auto F = getCtrlFramesAndScale(world);
  F.scale *= sqrt(tau);

  if(accPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), {}, OT_sos, accPrec*F.scale, NoArr, 2, deltaFromStep, deltaToStep);
  }
  if(velPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames), {}, OT_sos, velPrec*F.scale, NoArr, 1, deltaFromStep, deltaToStep);
  }
  if(homingPrec) {
    addObjective({startTime, endTime}, make_shared<F_qItself>(F.frames, true), {}, OT_sos, {homingPrec*sqrt(tau)}, NoArr, 0, deltaFromStep, deltaToStep);
  }
}

//void KOMO::setSquaredQVelocities(double startTime, double endTime, double prec) {
//  auto *map = new TM_Transition(world);
//  map->velCoeff = 1.;
//  map->accCoeff = 0.;
//  addObjective(startTime, endTime, map, OT_sos, NoArr, prec, 1);
//}

//void KOMO::setFixEffectiveJoints(double startTime, double endTime, double prec) {
////  setTask(startTime, endTime, new TM_Transition(world, true), OT_eq, NoArr, prec, 1); //NOTE: order=1!!
//  addObjective({startTime, endTime}, make_shared<TM_FlagConstraints>(), OT_eq, {}, NoArrprec, k_order);
//  addObjective({startTime, endTime}, make_shared<TM_FlagCosts>(), OT_sos, {1.}, NoArr, k_order);
//}

//void KOMO::setFixSwitchedObjects(double startTime, double endTime, double prec) {
//  addObjective({startTime, endTime}, make_shared<TM_FixSwichedObjects>(), OT_eq, {}, NoArrprec, k_order);
//}

void KOMO_ext::setConfigFromFile() {
  Configuration C(getParameter<String>("KOMO/modelfile"));
//  K.optimizeTree();
  setConfig(
    C,
    getParameter<bool>("KOMO/useSwift", true)
  );
  setTiming(
    getParameter<uint>("KOMO/phases"),
    getParameter<uint>("KOMO/stepsPerPhase"),
    getParameter<double>("KOMO/durationPerPhase", 5.),
    getParameter<uint>("KOMO/k_order", 2)
  );
}

void KOMO_ext::setMoveTo(Configuration& world, Frame& endeff, Frame& target, byte whichAxesToAlign) {
//  if(MP) delete MP;
//  MP = new KOMO(world);
  setConfig(world);
  this->world.checkConsistency();

  setTasks(*this, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

struct DrawPaths : GLDrawer {
  arr& X;
  DrawPaths(arr& X): X(X) {}
  void glDraw(OpenGL& gl) {
#ifdef RAI_GL
    glColor(0., 0., 0.);
    for(uint i=0; i<X.d1; i++) {
      glBegin(GL_LINES);
      for(uint t=0; t<X.d0; t++) {
        rai::Transformation pose;
        pose.set(&X(t, i, 0));
//          glTransform(pose);
        glVertex3d(pose.pos.x, pose.pos.y, pose.pos.z);
      }
      glEnd();
    }
#endif
  }
};
