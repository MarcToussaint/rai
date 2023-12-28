#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>

void run_rrt(rai::Configuration& C, const arr& q0, const arr& q1, int verbose) {
  ConfigurationProblem P(C, false, 1e-3);
  P.computeCollisionFeatures = false;
  {
    StringA pairs = rai::getParameter<StringA>("collisionPairs", {});
    for(rai::String& s:pairs) P.collisionPairs.append(C[s]->ID);
    P.collisionPairs.reshape(-1,2);
  }
  RRT_PathFinder rrt(P, q0, q1, .1, verbose);
  double time = -rai::clockTime();
  rrt.run();
  time += rai::clockTime();
  cout <<"rrt time: " <<time <<endl;
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

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  rai::clockTime();
  testRRT();

  return 0;
}
