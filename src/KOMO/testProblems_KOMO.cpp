/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "testProblems_KOMO.h"
#include "../Optim/testProblems_Opt.h"
#include "../Kin/frame.h"
#include "../Kin/F_forces.h"
#include "../KOMO/komo.h"
#include "../Kin/F_geometrics.h"
#include "../Optim/utils.h"

shared_ptr<KOMO> problem_IK(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  rai::Frame* f = C.addFrame("target", "table");
  f->setRelativePosition({.3, .2, .2});
  f->setShape(rai::ST_sphere, {.02}) .setColor({1., 1., 0.});

  auto komo = make_shared<KOMO>();
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 0);

  komo->addControlObjective({}, 0, 1e-1);
  komo->add_jointLimits();
  komo->addObjective({}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1});
  return komo;
}

shared_ptr<KOMO> problem_IKobstacle(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  // #IK (table): {Q:"t(.1 .5 .3)  d(0 0 0 1) d(90 0 1 0)" shape:capsule, size:[.3, .02] }
  C.addFrame("dot", "table", "Q:\"t(.2 .5 .3)\", shape:sphere, size:[.02]" );
  C.addFrame("obstacle", "table", "Q:[.1 .2 .5], shape: capsule, size:[1. .1], color: [.2] " );

  auto manip = make_shared<ManipulationHelper>();
  manip->setup_inverse_kinematics(C, 1e-1, false);
  //    manip.grasp_cylinder(1., "l_gripper", "cylinder", "l_palm");
  manip->komo->addObjective({}, FS_positionDiff, {"l_gripper", "dot"}, OT_eq, {1e1});
  for(uint coll=3;coll<=7;coll++){
    manip->no_collisions({1.}, {STRING("l_panda_coll"<<coll), "obstacle"});
  }
  manip->no_collisions({1.}, {"l_palm", "obstacle"});

  return manip->komo;
}

shared_ptr<KOMO> problem_IKtorus(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  auto f = & C.addFrame("torus", "table", "Q:[.1, .2, .4]") ->setMesh2(rai::Mesh().readFile(rai::raiPath("../rai-robotModels/objects/torus.ply"))) .setColor({.9});
  rai::Mesh& M = f->getShape().mesh();
  rai::Mesh decomp = M.decompose();
  C.addFrame("coll_torus", "torus") ->setMesh2(decomp) .setColor({1.,.1}); //, "shape: ssCylinder, size:[.04 .23 .02], color:[1 .1] " );
  // C.addFrame("coll_torus", "torus", "shape: ssCylinder, size:[.04 .23 .02], color:[1 .1] " );
  // C.view(true);

  rai::Frame* torus = C.getFrame("torus");
  torus->set_X()->rot.setRandom();

  auto komo = make_shared<KOMO>();
  komo->setConfig(C, false);
  komo->setTiming(1., 1, 1., 0);
  komo->addControlObjective({}, 0, 1e-1);
  komo->add_jointLimits();
  komo->addObjective({}, make_shared<F_TorusGraspEq>(.2, .02), {"l_gripper", "torus"}, OT_eq, {1e1});
  // komo->addObjective({}, FS_negDistance, {"l_palm", "coll_torus"}, OT_ineq, {1e1});

  auto manip = make_shared<ManipulationHelper>(komo);
  manip->no_collisions({}, {"l_panda_coll1", "table",
                            "l_panda_coll2", "table",
                            "l_panda_coll3", "table",
                            "l_panda_coll4", "table",
                            "l_panda_coll5", "table",
                            "l_panda_coll6", "table",
                            "l_panda_coll7", "table",
                            "l_palm", "coll_torus"}, .0);

#if 0
  manip->no_collisions({}, {"l_panda_coll3", "coll_torus",
                            "l_panda_coll4", "coll_torus",
                            "l_panda_coll5", "coll_torus",
                            "l_panda_coll6", "coll_torus",
                            "l_panda_coll7", "coll_torus"}, .0, 1e-0);
#endif

  return komo;
}

shared_ptr<KOMO> problem_PushToReach(){
  rai::Configuration C;
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

  auto manip = make_shared<ManipulationHelper>(str{"push"});

  manip->setup_sequence(C, 4, 1e-1, 1e-1, false);
  manip->komo->addQuaternionNorms();

  //1,2: push
  // manip->komo->addModeSwitch({1., 2.}, rai::SY_stable, {gripper, obj}, true);
  // manip->komo->addModeSwitch({2., -1.}, rai::SY_stableOn, {table, obj}, false);
  // auto helper_frame =
  manip->komo->addFrameDof("obj_trans", table, rai::JT_transXY, false, obj); //a permanent moving(!) transXY joint table->trans, and a snap trans->obj
  // helper_frame->setAutoLimits();
  // helper_frame->joint->sampleUniform=1.;
  manip->komo->addRigidSwitch(1., {"obj_trans", obj});
  manip->straight_push({1.,2.}, obj, gripper, table);
  manip->no_collisions({2.}, {stick, obj}, .02);
  manip->freeze_joint({3., -1.}, {"obj_trans"});

  //3: pick
  manip->grasp_cylinder(3., gripper, stick, palm);
  manip->no_collisions({2.,4.}, {"l_panda_coll1", obj,
                                 "l_panda_coll2", obj,
                                 "l_panda_coll3", obj,
                                 "l_panda_coll4", obj,
                                 "l_panda_coll5", obj,
                                 "l_panda_coll6", obj,
                                 "l_panda_coll7", obj,
                                 "l_palm", obj}, .02);

  //3,4: carry
  manip->komo->addModeSwitch({3., -1.}, rai::SY_stable, {gripper, stick}, true);

  //4: touch
  manip->komo->addObjective({4.}, FS_negDistance, {stick, "dot"}, OT_eq, {1e1});
  manip->no_collisions({4.}, {stick, table,
                             palm, table}, .01);

  return manip->komo;
}

shared_ptr<KOMO> problem_StableSphere(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/ballFinger.g"), "l_");
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/ballFinger.g"), "r_");
  C.addFrame("world");
  C.addFrame("table", "world", "Q:[0 0 .1], shape: ssBox, size: [1 1 .1 .01], color: [.8], contact: 1" );
  C.addFrame("wall", "table", "Q:[-.45 0 .3], shape: ssBox, size: [.1 1 .5 .01], color: [.8], contact: 1" );
  C.addFrame("box", "table", "Q:[.05 -.35 .25], shape: ssBox, size: [.9 .3 .4 .01], color: [.8], contact: 1" );
  C.addFrame("obj", "world", "joint:trans3, limits:[-.5,-.5,0,.5,.5,1], Q:[0 0 .3], shape: sphere, size: [.1], color: [1 .5 .0 .5], mass: .2, sampleUniform: 1., contact: 1" );

  C["l_finger"]->setShape(rai::ST_sphere, {.05});
  C["l_jointX"]->joint->limits = {-.5,.5};
  C["l_jointY"]->joint->limits = {-.5,.5};
  C["l_jointZ"]->joint->limits = {-.5,.5};
  C["r_finger"]->setShape(rai::ST_sphere, {.05});
  C["r_jointX"]->joint->limits = {-.5,.5};
  C["r_jointY"]->joint->limits = {-.5,.5};
  C["r_jointZ"]->joint->limits = {-.5,.5};

  StringA supports = { "box", "wall", "l_finger", "r_finger" };

  auto komo = make_shared<KOMO>();
  komo->setConfig(C);
  komo->setTiming(1,1,1,0);
  komo->addControlObjective({}, 0, 1e-1);

  komo->addObjective({}, make_shared<F_TotalForce>(), {"obj"}, OT_eq, {1e1} );
  komo->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1} );

  // if(opt.verbose>0) komo->set_viewer(C.get_viewer());

  double frictionCone_mu = rai::getParameter<double>("RndStableConfigs/frictionCone_mu", .8);

  //-- discrete decisions:
  str supp="supports:";
  for(const str& thing:supports){
    if(rnd.uni()<.5){
      supp <<' ' <<thing;
      komo->addContact_stick(0.,-1., "obj", thing, frictionCone_mu);
    }
  }
  // if(opt.verbose>0){
  LOG(0) <<"\n======================\n" <<supp;

  return komo;
}

//===========================================================================

StringA rai::get_NLP_Problem_names(){
  static StringA names = {"quadratic", "RastriginSOS", "Modes", "Wedge", "HalfCircle", "LinearProgram", "IK", "IKobstacle", "IKtorus", "PushToReach", "StableSphere", "SpherePacking", "MinimalConvexCore"};
  return names;
}

std::shared_ptr<NLP> KOMO_wrap(std::shared_ptr<KOMO> komo){
  auto nlp = komo->nlp();
  nlp->obj = komo;
  return nlp;
}

std::shared_ptr<NLP> rai::make_NLP_Problem(str problem){
  if(!problem.N) problem = rai::getParameter<str>("problem");

  int s = problem.find('.', false);
  uint dim = 2;
  if(s>0){
    problem.getSubString(s+1,-1) >>dim;
    rai::setParameter<uint>("problem/dim", dim);
    problem = problem.getSubString(0,s-1);
  }

  std::shared_ptr<NLP> nlp;

  if(problem == "square") nlp = make_shared<NLP_Squared>(dim);
  else if(problem == "Rastrigin") nlp = make_shared<Conv_ScalarFunction2NLP>(make_shared<NLP_Rosenbrock>(dim));
  else if(problem == "Rosenbrock") nlp = make_shared<Conv_ScalarFunction2NLP>(make_shared<NLP_Rosenbrock>(dim));
  else if(problem == "RastriginSOS") nlp = make_shared<NLP_RastriginSOS>();

  else if(problem == "Box") nlp = make_shared<BoxNLP>();
  else if(problem == "Modes") nlp = make_shared<ModesNLP>();
  else if(problem == "Wedge") nlp = make_shared<NLP_Wedge>();
  else if(problem == "HalfCircle") nlp = make_shared<NLP_HalfCircle>();
  else if(problem == "LinearProgram") nlp = make_shared<NLP_RandomLP>();

  else if(problem == "IK") nlp = KOMO_wrap(problem_IK());
  else if(problem == "IKobstacle") nlp = KOMO_wrap(problem_IKobstacle());
  else if(problem == "IKtorus") nlp = KOMO_wrap(problem_IKtorus());
  else if(problem == "PushToReach") nlp = KOMO_wrap(problem_PushToReach());
  else if(problem == "StableSphere") nlp = KOMO_wrap(problem_StableSphere());

  else if(problem == "SpherePacking") nlp = make_shared<SpherePacking>();
  else if(problem == "MinimalConvexCore") nlp = make_shared<MinimalConvexCore>();

  else HALT("can't create problem '" <<problem <<"'");

  CHECK(nlp, "");
  //  NLP_Viewer(nlp).display();  rai::wait();

  return nlp;
}

//===========================================================================

SpherePacking::SpherePacking(uint _n, double _rad, bool _ineqAccum) : n(_n), rad(_rad), ineqAccum(_ineqAccum) {
  dimension = 3*n;

  bounds.resize(2, n, 3);
  for(uint i=0;i<n;i++){
    bounds(0, i, 0) = -1.+rad;
    bounds(1, i, 0) = +1.-rad;
    bounds(0, i, 1) = -1.+rad;
    bounds(1, i, 1) = +1.-rad;
    bounds(0, i, 2) = 0.+rad;
    bounds(1, i, 2) = +16.-rad;
  }
  bounds.reshape(2, dimension);

  featureTypes.clear();
  if(!ineqAccum){
    featureTypes.append(rai::consts<ObjectiveType>(OT_ineq, n*(n-1)/2));
  }else{
    featureTypes.append(OT_eq);
  }
  featureTypes.append(rai::consts<ObjectiveType>(OT_f, n));

  //bounds as constraints
  // featureTypes.append(rai::consts<ObjectiveType>(OT_ineq, 2*dimension));
}

void SpherePacking::ineqAccumulation(uint phiIdx, arr& phi, arr& J, arr& g, const arr& Jg){
  CHECK_EQ(g.N, Jg.d0, "");
  CHECK_EQ(J.d1, Jg.d1, "");
  CHECK_GE(phi.N, phiIdx, "");

  // g += .01;

  for(uint i=0; i<g.N; i++) if(g(i)>0.) {  //ReLu for g
#if 0
        phi.elem(phiIdx) += g(i);
        J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0);
#elif 1
      double delta=.1;
      if(g(i)>delta){
        phi.elem(phiIdx) += g(i)-0.5*delta;
        J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0);
      }else{
        phi.elem(phiIdx) += 0.5*g(i)*g(i)/delta ;
        J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, g(i)/delta);
      }
#elif 1
      double delta=.1, x = g.elem(i);
      phi.elem(phiIdx) += x*x/delta ;
      J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, 2.*x/delta);
#else
      double delta=.1;
      double x = g(i);
      phi.elem(phiIdx) += x * ::exp(-delta/x);
      J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, ::exp(-delta/x)* (1.+delta/x));
#endif
    }

  // phi.elem(phiIdx) -= .1;
}

void SpherePacking::evaluate(arr& phi, arr& J, const arr& _x){
  x = _x;
  x.reshape(n,3);

  uint dimphi = featureTypes.N;
  phi.resize(dimphi).setZero();
  // J.resize(dimphi, dimension).setZero();
  if(!!J) J.sparse().resize(dimphi, dimension, 0);

  uint m=0;

  //constraints: collisions
  arr collPhi, collJ;
  {
    collPhi.resize(n*(n-1)/2);
    collJ.sparse().resize(collPhi.N, dimension, 0);
    for(uint i=0;i<n;i++) for(uint j=i+1;j<n;j++){
        arr diff = x[i]-x[j];
        double dist = length(diff);
        if(true || dist<2.*rad+.2){ //ipopt needs constant J_structure
          collPhi(m) = (-dist + 2.*rad); //ineq
          if(!!J){
            for(uint k=0;k<3;k++){
              collJ.elem(m,3*i+k) = -1./dist*diff.elem(k);
              collJ.elem(m,3*j+k) = +1./dist*diff.elem(k);
            }
          }
        }else{
          collPhi(m) = (-.2);
        }
        m++;
      }
  }

  if(!ineqAccum){
    CHECK_EQ(collPhi.N, m, "");
    phi.setVectorBlock(collPhi, 0);
    if(!!J) J.sparse().add(collJ, 0, 0);
  }else{
    collJ.sparse().setupRowsCols();
    ineqAccumulation(0, phi, J, collPhi, collJ);
    m = 1;
  }

  //costs: z coordinates
  for(uint i=0;i<n;i++){
    phi(m) = x(i,2); //f
    if(!!J) J.elem(m,3*i+2) = 1.;
    m++;
  }

  //bounds as constraints
  // for(uint i=0;i<dimension;i++){
  //   phi(m) = bounds(0,i) - x.elem(i);
  //   if(!!J) J.elem(m, i) = -1.;
  //   m++;
  //   phi(m) = x.elem(i) - bounds(1,i);
  //   if(!!J) J.elem(m, i) = +1.;
  //   m++;
  // }

  // report(cout, 10, "inner");
  CHECK_EQ(m, dimphi, "");
}

void SpherePacking::report(std::ostream& os, int verbose, const char* msg){
  // NLP::report(os, verbose, msg);
  x.reshape(n, 3);
  os <<"SpherePacking problem" <<endl;
  if(!disp.frames.N){
    for(uint i=0;i<n;i++){
      rai::Frame *f = disp.addFrame(STRING("sphere"<<i));
      f->setShape(rai::ST_sphere, {rad});
    }
    disp.addFrame("box")->setShape(rai::ST_box, {2.,2.,16.}).setColor({1.,1.,0.,.2}).setPosition({0.,0.,8.});
  }
  for(uint i=0;i<n;i++){
    disp.frames(i)->setPosition(x[i]);
  }
  disp.view(verbose>5);
}

//===========================================================================

MinimalConvexCore::MinimalConvexCore(const arr& X, double radius) : radius(radius) {
  if(!X.N){
    // rai::Configuration C;
    // C.addFile(rai::raiPath("../rai-robotModels/panda/panda.g"));
    // m0 = C["panda_link3_0"]->shape->mesh();
    M.setSphere(1);
    // M.setBox();
    M.scale(.08,.2,.3);
    M.translate(0.,0.,.5);
  }else{
    M.V = X;
  }
  M.V.reshape(-1,3);
  M.makeConvexHull();
  M.C = {.5, .3, .3, 1.};

  cen = mean(M.V);

  dimension = M.V.N;
  featureTypes.resize(M.V.d0+1+M.V.N) = OT_ineq;
  featureTypes(0) = OT_f;
  for(uint i=0;i<M.V.N;i++) featureTypes(M.V.d0+1+i)= OT_sos;
  // bounds.setBlockVector(min(m0.V,0), max(m0.V,0));
  // bounds.reshape()
}

arr MinimalConvexCore::getInitializationSample(){
  arr x = M.V;
  x.reshape(-1);
  x += .01 * randn(x.N);
  return x;
}

double attractor(double x, double a, double& dydx_x){
  CHECK_GE(x, 0., "")
  double f = x/a;
  double lof = log(1.+f);
  double y = f/(1.+f) * lof;
  dydx_x = (lof/f + (1.-lof)/(1.+f)) / (1.+f);
  dydx_x /= a*a;
  double scale= 1e0;
  dydx_x *= scale;
  return scale * y;
}

void MinimalConvexCore::evaluate(arr& phi, arr& J, const arr& _x) {
  x = _x;
  x.reshape(-1, 3);

  uint n = x.d0;

  phi.resize(n+1+x.N).setZero();
  if(!!J) J.resize(n+1+x.N, x.N).setZero();

  //-- accumulated cost
  double cost = 0.;
  double l_a=.01, dxdl_l;
  arr Jcost = zeros(x.N);
  for(uint i=0; i<M.T.d0; i++) {
    int a=M.T(i, 0), b=M.T(i, 1), c=M.T(i, 2);
    {
      arr d = x[a]-x[b];
      double l = length(d);
      cost += attractor(l, l_a, dxdl_l);
      Jcost({3*a, 3*a+2+1}) += d*dxdl_l;
      Jcost({3*b, 3*b+2+1}) += -d*dxdl_l;
      //            if(!!H){
      //              for(uint k=0;k<3;k++) for(uint l=0;l<3;l++){
      //                H(3*a+k,3*a+l) += d(k)*d(l)/(l*l);
      //              }
      //            }
    }
    {
      arr d = x[c]-x[b];
      double l = length(d);
      cost += attractor(l, l_a, dxdl_l);
      Jcost({3*c, 3*c+2+1}) += d*dxdl_l;
      Jcost({3*b, 3*b+2+1}) += -d*dxdl_l;
    }
    {
      arr d = x[a]-x[c];
      double l = length(d);
      cost += attractor(l, l_a, dxdl_l);
      Jcost({3*a, 3*a+2+1}) += d*dxdl_l;
      Jcost({3*c, 3*c+2+1}) += -d*dxdl_l;
    }
  }

  phi(0) = cost;
  if(!!J) J[0] = Jcost;

  //-- radius inequalities
  for(uint i=0; i<n; i++) {
    arr d = M.V[i] - x[i];
    double l = length(d);
    phi(i+1) = l - radius;
    if(l>1e-6) {
      if(!!J) J(i+1, {3*i, 3*i+2+1}) += -d/l;
    }
  }

  //-- center sos
  for(uint i=0;i<x.d0;i++) x[i] -= cen;
  x.reshape(-1);
  double scale = 1e-1;
  phi.setVectorBlock(scale*x, M.V.d0+1);
  if(!!J) J.setMatrixBlock(scale*eye(x.N), M.V.d0+1, 0);

  x.reshape(-1, 3);
  for(uint i=0;i<x.d0;i++) x[i] += cen;
}

void MinimalConvexCore::report(ostream& os, int verbose, const char* msg){
  // gl.dataLock.lock(RAI_HERE);
  // m0.setSSCvx(_x, radius);
  // gl.dataLock.unlock();
  // gl.update();

  // NLP::report(os, verbose, msg);
  x.reshape(M.V.d0, 3);
  os <<"MinimalConvexCore problem" <<endl;
  disp.clear();
  if(!disp.frames.N){
    rai::Frame *pts = disp.addFrame("pts");
    pts->setPointCloud(M.V);
    rai::Frame *cvx = disp.addFrame("cvx");
    cvx->setConvexMesh(x, {}, radius);
    cvx->setColor({1., .2});
    rai::Frame *core = disp.addFrame("core");
    core->setPointCloud(x);
    core->setColor({1,0,0});
  }
  // for(uint i=0;i<n;i++){
  //   disp.frames(i)->setPosition(x[i]);
  // }
  disp.view(verbose>5);
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

