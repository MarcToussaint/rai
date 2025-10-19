#include <KOMO/komo.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>

#include <thread>
#include <memory>

//===========================================================================

void TEST(Easy){
  rai::Configuration C(rai::raiPath("../rai-robotModels/tests/arm.g"));
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  
  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 100, 5., 2);
  komo.addControlObjective({}, 2, 1.);
  komo.addQuaternionNorms({}, 1e1, false);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});

  cout <<komo.report(true) <<endl;

  komo.solve(0., 5); //2nd argument uses spline representation!!
//  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
//  komo.plotTrajectory();
//  komo.reportProxies();
//  komo.checkGradients();
  komo.report(false, true, true);

//  rai::ConfigurationViewer V;
//  V.setConfiguration(komo.pathConfig, "KOMO test solution");
//  V.sliceTexts.resize(komo.T);
//  arr err = komo.info_objectiveErrorTraces();
//  for(uint t=0;t<V.sliceTexts.N;t++){
//    V.sliceTexts(t) = komo.info_sliceObjectives(t,err);
//  }
//  while(V.playVideo(komo.timeSlices, true, 3.));

//  for(uint t=0;t<komo.T;t++){
//    V.view_slice(t);
//    rai::wait(.1);
//  }


  komo.view(true, "result");
  komo.view_play(true);
}

//===========================================================================

void TEST(Align){
  rai::Configuration C(rai::raiPath("../rai-robotModels/tests/arm.g"));
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 100, 5., 2);

  komo.addControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});

  komo.solve();
//  komo.checkGradients();

//  komo.plotTrajectory();
  komo.report(false, true, true);
  komo.view(true, "result");
  komo.view_play(true);

//  komo.pathConfig.setJointState(komo.x);
//  V.setConfiguration(komo.pathConfig, "path", true);
}

//===========================================================================

void TEST(Thin){
  rai::Configuration C(rai::raiPath("../rai-robotModels/tests/thin.g"));
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 60, 5., 2);
  komo.addControlObjective({}, 2, 1.);

  //-- set a time optim objective
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e2}, {}, 1); //smooth time evolution
//  komo.addObjective({}, make_shared<TM_Time>(), OT_sos, {1e1}, {komo.tau}, 0); //prior on timing

  komo.addObjective({1.}, FS_positionDiff, {"ball", "target"}, OT_eq, {1e1});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.addObjective({}, FS_distance, {"wall", "ball"}, OT_ineqB, {1.});
  komo.addObjective({}, make_shared<F_VelocityDistance>(.1), {"ball", "wall"}, OT_sos, {1e1}, {}, 1);

  cout <<komo.report(true) <<endl;

  komo.view(true, "init");

  komo.opt.animateOptimization=1;
  komo.solve(1e-2, 5);
  komo.plotTrajectory();
//  komo.reportProxies();
  komo.checkGradients();

  komo.view(true, "result");
  komo.view_play(true);
}

//===========================================================================

void TEST(PR2){
  rai::Configuration C(rai::raiPath("../rai-robotModels/tests/pr2Shelf.g"));
  C.processStructure();
  cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
  double rand = rai::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(rai::getParameter<uint>("rndSeed", 0));
    rndGauss(C.q,rand,true);
    C.setJointState(C.q);
  }

  C.view(true);

  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1., 30, 10., 2);
  komo.addControlObjective({}, 2, 1.);
  komo.addObjective({1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  komo.solve();
  komo.plotTrajectory();
//  komo.checkGradients();
  komo.view(true, "result");
  komo.view_play(true);
}

//===========================================================================

void TEST(Threading) {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/workshopTable.g"));

  //-- create KOMO problem, to be cloned later
  KOMO komo;
  komo.opt.verbose = 0;
  //komo.opt.animateOptimization=1;
  komo.setConfig(C, false);
  komo.setTiming(1, 10, 2, 2);
  komo.addControlObjective({}, 2);
  komo.addObjective({1, 1}, FS_positionDiff, {"r_gripper", "block1"}, OT_eq, {1e2});
  arr x0 = komo.pathConfig.getJointState();

  //-- how many threads?
  uint nThreads = 3;
  uint nIters = 120 / nThreads;

  //-- define worker routine
  auto routine = [&]() {
    KOMO komo2;
    komo2.clone(komo); //each worker clones original problem
    for (uint i=0; i<nIters; i++) {
      komo2.pathConfig.setJointState(x0);
      komo2.solve();
    }
    return 0;
  };

  //-- run them all
  double time = -rai::realTime();
  {
    rai::Array<std::shared_ptr<std::thread>> threads;
    for(uint t=0;t<nThreads;t++) threads.append(make_shared<std::thread>(routine));
    for(uint t=0;t<nThreads;t++) threads(t)->join();
  }
  time += rai::realTime();
  std::cout <<nThreads <<" threads, " <<nIters <<" runs each: " <<time <<" sec"<< std::endl;
}

//===========================================================================

void TEST(Mobile){
  rai::Configuration C;
  C.addFile("../../Algo/rrt/mobile.g");

  arr q0 = C.getJointState();
  C["ranger_transX"]->setPosition(C["goal"]->getPosition());
  C["ranger_transY"]->setPosition(C["goal"]->getPosition());
  C["ranger_rot"]->setPose(C["goal"]->getPose());
  arr q1 = C.getJointState();
  C.setJointState(q0);

  rai::RRT_PathFinder rrt;
  rrt.opt.set_maxIters(500000). set_verbose(2). set_stepsize(.2) .set_subsamples(0);
  rrt.setProblem(_shared(C));
  rrt.setStartGoal(q0, q1);
  rrt.solve();
  rrt.view(false, STRING(*rrt.ret));

  uint T=100;
  T = rrt.path.d0;
  C.setJointState(q0);
  KOMO komo;
  komo.setConfig(C);
  komo.setTiming(1, T, 10., 2);
  komo.addControlObjective({}, 1, 1e-2);
  komo.addControlObjective({}, 2, 1e-1);
  komo.addQuaternionNorms({}, 1e2, false);
  komo.add_collision(true, .0, 1e1);
  komo.addObjective({1.}, FS_poseDiff, {"goal", "ranger_rot"}, OT_eq, {1e1}, {});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);
  komo.initWithPath_qOrg(rrt.path);
  // komo.checkGradients();
  // komo.opt.animateOptimization=1;
  rai::NLP_Solver sol;
  sol.setProblem(komo.nlp());
  sol.opt->set_verbose(2);
  sol.solve();
  cout <<komo.report(false, true, true) <<endl;
  komo.view(true);
  komo.view_play(true, 0, 1.);
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.seed_random();

  testEasy();
  testAlign();
  testThin();
  testPR2();
  testThreading();
  testMobile();

  return 0;
}

