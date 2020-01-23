#include <KOMO/komo.h>

//===========================================================================

void TEST(Easy){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  KOMO komo;
  komo.setModel(C);
  komo.setPathOpt(1., 100, 5.);
  komo.setSquaredQAccVelHoming();

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_sos, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reportProblem();

  komo.reset();
//  komo.setSpline(5);
  komo.run();
  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  komo.plotTrajectory();
//  komo.reportProxies();
  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(Align){
  rai::Configuration C("arm.g");
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  KOMO komo;
  komo.setModel(C);
  komo.setPathOpt(1., 100, 5.);
  komo.setSquaredQAccVelHoming();

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reset();
  komo.run();
  komo.plotTrajectory();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
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
  komo.setPathOpt(1., 100, 10.);
  komo.setSquaredQAccVelHoming();
  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.reset();
//  komo.setSpline(10);
  komo.run();
  komo.plotTrajectory();
//  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
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

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

//  testEasy();
//  testAlign();
  testPR2();

  return 0;
}

