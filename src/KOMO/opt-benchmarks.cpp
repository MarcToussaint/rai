/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "opt-benchmarks.h"
#include "../Optim/utils.h"
#include "../Optim/benchmarks.h"
#include "../Kin/frame.h"
#include "../KOMO/komo.h"
#include "../Kin/F_geometrics.h"

void Problem::load(str problem){
  std::shared_ptr<Problem> P = make_shared<Problem>();

  if(problem == "box"){
    nlp = make_shared<BoxNLP>();
  }
  else if(problem == "modes"){
    nlp = make_shared<ModesNLP>();
  }
  if(problem == "tbox"){
    auto box = make_shared<BoxNLP>();
    uint n = box->dimension;
    arr A = eye(n);
    arr b = zeros(n);
    for(uint i=0;i<n;i++) A(i,i) = (box->bounds(1,i)-box->bounds(0,i))/2.;
    nlp = make_shared<NLP_LinTransformed>(box, A, b);
  }

  if(problem == "linear-program"){
    nlp = getBenchmarkFromCfg();
  }

  if(problem == "IK"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    rai::Frame* f = C.addFrame("target", "table");
    f->setRelativePosition({.3, .2, .2});
    f->setShape(rai::ST_sphere, {.02}) .setColor({1., 1., 0.});

    komo = make_shared<KOMO>();
    komo->setConfig(C, false);
    komo->setTiming(1., 1, 1., 0);

    komo->addControlObjective({}, 0, 1e-1);
    komo->add_jointLimits();
    komo->addObjective({}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1});

    nlp = komo->nlp();
  }

  if(problem == "IK-obstacle"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    // #IK (table): {Q:"t(.1 .5 .3)  d(0 0 0 1) d(90 0 1 0)" shape:capsule, size:[.3, .02] }
    C.addFrame("dot", "table", " {Q:\"t(.2 .5 .3)\", shape:sphere, size:[.02]" );
    C.addFrame("obstacle", "table", "Q:[.1 .2 .5], shape: capsule, size:[1. .1], color: [.2] " );

    manip = make_shared<ManipulationModelling>();
    manip->setup_inverse_kinematics(C, 1e-1, false);
    //    manip.grasp_cylinder(1., "l_gripper", "cylinder", "l_palm");
    manip->komo->addObjective({}, FS_positionDiff, {"l_gripper", "dot"}, OT_eq, {1e1});
    for(uint coll=3;coll<=7;coll++){
      manip->no_collision({1.}, {STRING("l_panda_coll"<<coll), "obstacle"});
    }
    manip->no_collision({1.}, {"l_palm", "obstacle"});

    komo = manip->komo;
    nlp = komo->nlp();
  }

  if(problem == "torus-grasp"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.addFrame("torus", "table", "Q:[.1, .2, .4]") ->setMesh2(rai::Mesh().readFile(rai::raiPath("../rai-robotModels/objects/torus.ply"))) .setColor({.9});
    C.addFrame("coll_torus", "torus", "shape: ssCylinder, size:[.04 .23 .02], color:[1 .1] " );

    rai::Frame* torus = C.getFrame("torus");
    torus->set_X()->rot.setRandom();

    komo = make_shared<KOMO>();
    komo->setConfig(C, false);
    komo->setTiming(1., 1, 1., 0);
    komo->addControlObjective({}, 0, 1e-1);
    //    komo->add_jointLimits();
    komo->addObjective({}, make_shared<F_TorusGraspEq>(.2, .02), {"l_gripper", "torus"}, OT_eq, {1e1});
    komo->addObjective({}, FS_negDistance, {"l_palm", "coll_torus"}, OT_ineq, {1e1});

    nlp = komo->nlp();
  }

  if(problem == "push"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.addFrame("box", "table", "joint: rigid, Q:[.4, .0, .25], shape: ssBox, size: [.15, .4, .4, .005], contact: 1, mass: .1" );
    C.addFrame("dot", "table", "Q: [-.6, .6, .1], shape:sphere, size:[.02], color: [1., 1., .5]" );
    C.addFrame("stick", "table", "joint: rigid, shape: capsule, Q: \"t(.75 .0 .1) d(90 1 0 0)\", size: [.6, .02], color: [.6], contact: 1" );
    //Edit panda_collCameraWrist: { shape: marker, contact: 0 }

    rai::Joint *j = C["l_panda_finger_joint1"]->joint;
    j->setDofs(arr{.02});

    auto gripper = "l_gripper";
    auto palm = "l_palm";
    auto stick = "stick";
    auto obj = "box";
    auto table = "table";

    manip = make_shared<ManipulationModelling>(str{"push"});

    manip->setup_sequence(C, 4, 1e-1, 1e-1, false);
    manip->komo->addQuaternionNorms();

    //1,2: push
    manip->komo->addModeSwitch({1., 2.}, rai::SY_stable, {gripper, obj}, true);
    manip->komo->addModeSwitch({2., -1.}, rai::SY_stableOn, {table, obj}, false);
    manip->straight_push({1.,2.}, obj, gripper, table);
    manip->no_collision({2.}, {stick, obj}, .02);

    //3: pick
    manip->grasp_cylinder(3., gripper, stick, palm);
    manip->no_collision({3.}, {"l_panda_coll5", obj,
                                  "l_panda_coll6", obj,
                                  "l_panda_coll7", obj,
                                  "l_palm", obj}, .02);

    //3,4: carry
    manip->komo->addModeSwitch({3., -1.}, rai::SY_stable, {gripper, stick}, true);

    //4: touch
    manip->komo->addObjective({4.}, FS_negDistance, {stick, "dot"}, OT_eq, {1e1});
    manip->no_collision({4.}, {stick, table,
                                  palm, table}, .01);

    komo = manip->komo;
    nlp = komo->nlp();
  }

  CHECK(nlp, "");
  //  NLP_Viewer(nlp).display();  rai::wait();
}

//===========================================================================

BoxNLP::BoxNLP(){
  dimension = rai::getParameter<uint>("problem/dim", 2);
  featureTypes.resize(2*dimension);
  featureTypes = OT_ineq;
  bounds.resize(2, dimension);
  bounds[0] = -2.;
  bounds[1] = +2.;
  if(rai::getParameter<bool>("problem/costs", false)){
    featureTypes.append(rai::consts<ObjectiveType>(OT_sos,dimension));
  }
}

void BoxNLP::evaluate(arr& phi, arr& J, const arr& x){
  phi.resize(2*dimension);
  phi({0,dimension-1}) = -(x + 1.);
  phi({dimension,-1}) = x - 1.;

  J.resize(phi.N, x.N).setZero();
  for(uint i=0;i<dimension;i++){
    J(i,i) = -1.;
    J(dimension+i,i) = 1.;
  }

  if(featureTypes.N>2*dimension){
    arr d = x;
    //    if(d(0)>0.) d(0) -= 1.; else d(0) += 1.;
    d -= ones(d.N);
    double w = 2.;
    phi.append(w*d);
    J.append(w*eye(dimension));
  }
}

//===========================================================================

ModesNLP::ModesNLP(){
  dimension = rai::getParameter<uint>("problem/dim", 2);
  uint k = rai::getParameter<uint>("problem/modes", 5);
  cen = randn(k, dimension);
  cen.reshape(k, dimension);
  radii = .2 +10*rand(k);

#if 1
  k = 1+(uint(1)<<dimension);
  cen = randn(k, dimension);
  radii = consts(.1, k);
  cen[-1] = 0.;
  radii(-1) = .5;
  for(uint i=0;i<k-1;i++){
    for(uint d=0;d<dimension;d++){
      cen(i,d) = (i&(1<<d))? -1.: 1.;
    }
  }
#endif

  cen.reshape(-1, dimension);
  featureTypes.resize(1);
  featureTypes = OT_ineq;
  bounds.resize(2,dimension);
  bounds[0] = -1.2;
  bounds[1] = +1.2;
}

void ModesNLP::evaluate(arr& phi, arr& J, const arr& x){
  arr _x = x;
  _x.J_setId();

  uint k = cen.d0;
  arrA y(k);
  arr yval(k);
  for(uint i=0;i<k;i++){
    arr d = cen[i]-_x;
    double s = 1./(radii(i)*radii(i));
    y(i) = s * ~d*d - 1.;
    yval(i) = y(i).scalar();
  }

  phi = y(argmin(yval));
  J = phi.J_reset();
}

//===========================================================================

OptBench_InvKin_Simple::OptBench_InvKin_Simple() {
  rai::Configuration C(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  rai::Frame* f = C.addFrame("target", "table");
  f->setRelativePosition({.3, .2, .2});
  f->setShape(rai::ST_sphere, {.02}) .setColor({1., 1., 0.});

  komo = make_shared<KOMO>();
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 0);

  komo->addControlObjective({}, 0, 1e-1);
//  komo->add_jointLimits();
  komo->addObjective({}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1});

  nlp = komo->nlp();
}

OptBench_InvKin_Endeff::OptBench_InvKin_Endeff(const char* modelFile, bool unconstrained) {
  rai::Configuration C(modelFile);
  komo = make_unique<KOMO>();
  komo->opt.sparse = false;
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 1);
  komo->addControlObjective({}, 1, 1.);
  //    komo->addQuaternionNorms(-1., -1., 1e3); //when the kinematics includes quaternion joints, keep them roughly regularized

  ObjectiveType ot = OT_eq;
  double prec = 1e0;
  if(unconstrained) { ot = OT_sos;  prec = 1e2;  }

  komo->addObjective({}, FS_positionDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_vectorZDiff, {"gripper", "box"}, ot, {prec});
  komo->addObjective({}, FS_scalarProductXX, {"gripper", "box"}, ot, {prec});

  nlp = komo->nlp();
}

void OptBench_Skeleton::create(const char* modelFile, const rai::Skeleton& S, rai::ArgWord sequenceOrPath) {
  rai::Configuration C(modelFile);

  komo = make_unique<KOMO>();
  komo->setConfig(C, false);

  double maxPhase = S.getMaxPhase();
  if(sequenceOrPath==rai::_sequence) {
    komo->setTiming(maxPhase, 1, 2., 1);
    komo->addControlObjective({}, 1, 1e-1);
  } else {
    komo->setTiming(maxPhase, 30, 5., 2);
    komo->addControlObjective({}, 2, 1e0);
  }
  komo->addQuaternionNorms();

  S.addObjectives(*komo);

  nlp = komo->nlp();

  komo->run_prepare(0.);
  cout <<"** OptBench_Skeleton: created path ";
  komo->pathConfig.report();
}

//===========================================================================

OptBench_Skeleton_Pick::OptBench_Skeleton_Pick(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //grasp
    { 1., 1., rai::SY_touch, {"R_endeff", "box3"} },
    { 1., 1., rai::SY_stable, {"R_endeff", "box3"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_Handover::OptBench_Skeleton_Handover(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //grasp
    { 1., 1., rai::SY_touch, {"R_endeff", "stick"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "stick"} },

    //handover
    { 2., 2., rai::SY_touch, {"L_endeff", "stick"} },
    { 2., -1., rai::SY_stable, {"L_endeff", "stick"} },

    //touch something
    { 3., -1., rai::SY_touch, {"stick", "ball"} },
  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

//===========================================================================

OptBench_Skeleton_StackAndBalance::OptBench_Skeleton_StackAndBalance(rai::ArgWord sequenceOrPath) {
  rai::Skeleton S = {
    //pick
    { 1., 1., rai::SY_touch, {"R_endeff", "box0"} },
    { 1., 2., rai::SY_stable, {"R_endeff", "box0"} },
    { .9, 1.1, rai::SY_downUp, {"R_endeff"} },

    //place
    { 2., 2., rai::SY_touch, {"table", "box0"} },
    { 2., -1., rai::SY_stable, {"table", "box0"} },
    { 1.9, 2.1, rai::SY_downUp, {"R_endeff"} },

    //pick
    { 1.5, 1.5, rai::SY_touch, {"L_endeff", "box1"} },
    { 1.5, 3., rai::SY_stable, {"L_endeff", "box1"} },
    { 1.4, 1.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 3., 3., rai::SY_touch, {"box0", "box1"} },
    { 3., -1., rai::SY_stable, {"box0", "box1"} },
    { 2.9, 3.1, rai::SY_downUp, {"L_endeff"} },

    { 3., 4., rai::SY_forceBalance, {"box1"} },
    { 3., 4., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"R_endeff", "box2"} },
    { 4., 5., rai::SY_stable, {"R_endeff", "box2"} },
    { 3.9, 4.5, rai::SY_downUp, {"R_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box2"} },
    { 5., -1., rai::SY_stable, {"box1", "box2"} },
    { 4.9, 5.1, rai::SY_downUp, {"R_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box2"} },
    { 5., 5., rai::SY_contact, {"box1", "box2"} },

    { 5., -1., rai::SY_forceBalance, {"box1"} },
    { 5., -1., rai::SY_contact, {"box0", "box1"} },

    //pick
    { 4., 4., rai::SY_touch, {"L_endeff", "box3"} },
    { 4., 5., rai::SY_stable, {"L_endeff", "box3"} },
    { 3.9, 4.5, rai::SY_downUp, {"L_endeff"} },

    //place
    { 5., 5., rai::SY_touch, {"box1", "box3"} },
    { 5., 5., rai::SY_touch, {"box2", "box3"} },
    { 5., 5.5, rai::SY_stable, {"box1", "box3"} },
    { 4.9, 5.1, rai::SY_downUp, {"L_endeff"} },

    { 5., 5., rai::SY_forceBalance, {"box3"} },
    { 5., 5., rai::SY_contact, {"box1", "box3"} },

  };
  create(rai::raiPath("test/KOMO/skeleton/model2.g"), S, sequenceOrPath);
}

