#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <Geo/fclInterface.h>

void run_rrt(rai::Configuration& C, const arr& q0, const arr& q1, int verbose) {
  bool useFcl = rai::getParameter<bool>("useFcl", true);
  double rrtStepsize = rai::getParameter<double>("rrtStepsize", .1);
  double rrtSubsamples = rai::getParameter<int>("rrtSubsamples", 4);

  ConfigurationProblem P(C, useFcl, 1e-3);
  if(!useFcl){
    StringA pairs = rai::getParameter<StringA>("collisionPairs", {});
    for(rai::String& s:pairs) P.collisionPairs.append(C[s]->ID);
    P.collisionPairs.reshape(-1,2);
  }

  RRT_PathFinder rrt(P, q0, q1, rrtStepsize, verbose, rrtSubsamples);
  double time = -rai::cpuTime();
  rrt.run();
  time += rai::cpuTime();

  cout <<"rrt time: " <<time <<"sec or " <<1000.*time/double(rrt.iters) <<"msec/iter (tree sizes: " <<rrt.rrt0->getNumberNodes() <<' ' <<rrt.rrtT->getNumberNodes() <<")" <<endl;
//  cout <<"used params:\n" <<rai::params()() <<endl;
}

void TEST(RRT){
  int verbose = rai::getParameter<int>("rrtVerbose", 3);
  rai::Configuration C;
  C.addFile("scene.g");
  C.optimizeTree();
  if(verbose) C.view();

  arr q0,q1;

  { //start pose
    KOMO komo(C, 1., 1, 0, false);
    komo.addControlObjective({}, 0, 1e-2);
    komo.addObjective({}, FS_positionDiff, {"l_gripper", "target1"}, OT_eq, {1e1});
    auto ret = NLP_Solver(komo.nlp(), 0).solve();
    q0 = ret->x;
    cout <<"start pose: " <<*ret <<endl;
  }

  { //end pose
    KOMO komo(C, 1., 1, 0, true);
    komo.addControlObjective({}, 0, 1e-2);
    komo.add_collision(true);
    komo.addObjective({}, FS_positionDiff, {"l_gripper", "target2"}, OT_eq, {1e1});
    auto ret = NLP_Solver(komo.nlp(), 0).solve();
    q1 = ret->x;
    cout <<"start pose: " <<*ret <<endl;
  }

  run_rrt(C, q0, q1, verbose);
}

// =============================================================================

void testKinematics(bool withCollisions, uint N=100000){
  rai::Configuration C;
  C.addFile("scene.g");
  C.optimizeTree();

  arr q = C.getJointState();

  if(withCollisions){
    C.fcl()->mode = rai::FclInterface::_binaryCollisionAll;
//    C.fcl()->mode = rai::FclInterface::_distanceCutoff;  C.fcl()->cutoff = .0;
  }

  double time = -rai::cpuTime();
  for(uint i=0;i<N;i++){
    rndGauss(q);
    C.setJointState(q);
    for(rai::Frame *f:C.frames) if(f->shape && f->shape->cont) f->ensure_X(); //compute pose of all relevant frames

    if(withCollisions){
      C.stepFcl();
    }
  }
  time += rai::cpuTime();
  cout <<"kinematics (collisions=" <<withCollisions <<") time: " <<time <<"sec or " <<1000.*time/double(N) <<"msec/query" <<endl;
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  cout <<"=== RRT test" <<endl;
  testRRT();

  cout <<"=== kinematics test" <<endl;
  testKinematics(false, 100000);

  cout <<"=== kinematics+FCL test" <<endl;
  testKinematics(true, 10000);

  return 0;
}
