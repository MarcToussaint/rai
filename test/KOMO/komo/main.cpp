#include <KOMO/komo.h>
#include <Kin/TM_default.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/solver.h>

//===========================================================================

void TEST(Easy){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 100, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_sos, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reportProblem();

//  komo.setSpline(5);
  komo.optimize();
  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  komo.plotTrajectory();
//  komo.reportProxies();
  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.view(true);
}

//===========================================================================

void TEST(Align){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
//  komo.solver = rai::KS_sparseStructured; //set via rai.cfg!!
//  komo.verbose=1; //set via rai.cfg!!
  komo.setModel(C);
  komo.setTiming(1., 100, 5., 2);

  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.optimize();
//  komo.checkGradients();

//  komo.plotTrajectory();
  rai::ConfigurationViewer V;
  V.setConfiguration(C);
  V.setPath(komo.getPath_X(), "optimized motion", true);
  for(uint i=0;i<2;i++) V.playVideo();

//  komo.pathConfig.setJointState(komo.x);
//  V.setConfiguration(komo.pathConfig, "path", true);
}

//===========================================================================

struct MyFeature : Feature {
  virtual void phi2(arr& y, arr& J, const FrameL& F){
    CHECK_EQ(order, 1, "");

    auto V = F_PositionDiff().setOrder(1).eval(F);

    auto C = F_PairCollision(F_PairCollision::_normal, false)
             .eval(F[1]);

    auto D = F_PairCollision(F_PairCollision::_negScalar, false)
             .eval(F[1]);

    //penalizing velocity whenever close
    double range=.2;
    if(-D.y.scalar() > range){
      y = zeros(3);
      if(!!J) J = zeros(3, V.J.d1);
      return;
    }

    double weight = 1. + D.y.scalar()/range;
    double normalWeight = 1.;

    arr P = eye(3) + normalWeight*(C.y*~C.y);
    y = weight * P * V.y;
    if(!!J){
      J = weight * P * V.J;
      J += P * V.y.reshape(3,1) * (1./range)*D.J;
      J += (weight * 2. * normalWeight * scalarProduct(C.y,V.y)) * C.J;
    }

#if 0
    //penalizing normal velocity
    double normalVel = scalarProduct(V.y, C.y);
    if(normalVel>0.){
      y = 0.;
      if(!!J) J = zeros(1, V.J.d1);
      return;
    }

    double scale = 3.;
    double weight = ::exp(scale * D.y.scalar());
    weight = 1.; scale=0.;

    y.resize(1);
    y(0) = weight * normalVel;
    if(!!J){
      J = weight * ( ~V.y * C.J + ~C.y * V.J );
      J += (normalVel * weight * scale) * D.J;
    }

#if 0
    normalVel += 1.;

    y = D.y / normalVel;

    if(!!J){
      J = D.J / normalVel;
      J += (-D.y.scalar() / (normalVel*normalVel)) * ( ~V.y * C.J + ~C.y * V.J );
    }
#endif
#endif

  }

  virtual uint dim_phi2(const FrameL& F) {  return 3;  }
};

void TEST(Thin){
  rai::Configuration C("thin.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1., 60, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"ball", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({}, FS_distance, {"wall", "ball"}, OT_ineq, {1.});

  komo.addObjective({}, make_shared<MyFeature>(), {"ball", "wall"}, OT_sos, {1e1}, {}, 1);

  komo.reportProblem();

  komo.animateOptimization=1;
  //  komo.setSpline(5);
  komo.optimize(1e-2);
  komo.plotTrajectory();
//  komo.reportProxies();
//  komo.checkGradients();

//  while(komo.displayTrajectory());
  rai::ConfigurationViewer V;
  V.setConfiguration(komo.pathConfig, "result", true);
  while(V.playVideo(C.frames.N));
}

//===========================================================================

void TEST(PR2){
  rai::Configuration C("model.g");
  C.optimizeTree(true);
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  double rand = rai::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(rai::getParameter<uint>("rndSeed", 0));
    rndGauss(C.q,rand,true);
    C.setJointState(C.q);
  }

  KOMO komo;
//  komo.logFile = new ofstream("z.dat");
//  komo.denseOptimization=true;
//  komo.sparseOptimization=true;
  komo.setModel(C);
  komo.setTiming(1., 100, 10., 2);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

//  komo.setSpline(10);
  komo.optimize();
  komo.plotTrajectory();
//  komo.checkGradients();
  rai::ConfigurationViewer V;
  V.setPath(C, komo.x, "result", true);
  while(V.playVideo());
}

//===========================================================================

// void TEST(FinalPosePR2){
//   rai::Configuration K("model.g");
//   K.pruneRigidJoints();
//   K.optimizeTree();
//   makeConvexHulls(K.frames);
//   cout <<"configuration space dim=" <<K.getJointStateDimension() <<endl;
//   arr x = finalPoseTo(K, *K.getFrameByName("endeff"), *K.getFrameByName("target"));
//   K.setJointState(x.reshape(x.N));
//   K.watch(true);
// }

void testFactors(){
  rai::Configuration C("../switches/model2.g");

  rai::Frame* g = C["gripper"];
  g->ensure_X();
  rai::Frame* g2 = new rai::Frame(C, g);
  g2->name = "gripperDUP";
  g2->setShape(rai::ST_box, {.1,.05,.05}); //usually not!
  g2->setParent(C.frames.first(), true);
  g2->setJoint(rai::JT_free);

  //=== define the full problem

  KOMO komo;

  komo.setModel(C, false);
  komo.setTiming(2.5, 3, 5., 2);
  komo.add_qControlObjective({}, 2);
  komo.addSquaredQuaternionNorms();

  //consistency constraint
  komo.addObjective({}, FS_poseDiff, {"gripper", "gripperDUP"}, OT_eq, {1e2});
  //grasp
  komo.addSwitch_stable(1., 2., "table", "gripper", "box");
  komo.addObjective({1.}, FS_positionDiff, {"gripperDUP", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripperDUP", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripperDUP"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);

  //place
  komo.addSwitch_stable(2., -1., "gripper", "table", "box", false);
  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  //slow - down - up
  komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
  komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);


  //=== select only gripperDUB
//  komo.selectJointsBySubtrees({"gripperDUP"});


  JointL ajo = komo.pathConfig.activeJoints;
  ajo.permuteRandomly();
  komo.pathConfig.setActiveJoints(ajo);

  komo.verbose = 4;
//  komo.animateOptimization = 1;
//  komo.optimize();

  komo.run_prepare(0);
//  komo.reportProblem();

  arr x,phi,J, blo,bup;

  auto nlp1 = komo.nlp_Factored();
  x = komo.pathConfig.getJointState();
  nlp1->getBounds(blo,bup);
  nlp1->evaluate(phi, J, x);
//  cout <<blo <<endl <<bup <<endl;
  cout <<"*****************" <<endl <<endl;

  auto nlp2 = komo.nlp_SparseNonFactored();
  x = komo.pathConfig.getJointState();
  nlp2->getBounds(blo,bup);
  nlp2->evaluate(phi, J, x);
//  cout <<blo <<endl <<bup <<endl;
  cout <<"*****************" <<endl <<endl;

//  return;
  //  checkJacobianCP(*nlp1, x, 1e-4);

  NLP_Solver()
//      .setProblem(*nlp2)
      .setProblem(*nlp1)
      .solve();

  komo.view(true, "optimized motion");
  while(komo.view_play(true));


}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

//  testEasy();
//  testAlign();
//  testThin();
//  testPR2();

  testFactors();

  return 0;
}

