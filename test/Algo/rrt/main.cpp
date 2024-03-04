#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <Geo/fclInterface.h>
#include <Kin/proxy.h>

// =============================================================================

void run_rrt(rai::Configuration& C, const arr& q0, const arr& q1) {
  int verbose = rai::getParameter<int>("rrt/verbose", 3);
  bool useFcl = rai::getParameter<bool>("useFcl", true);

  ConfigurationProblem P(C, useFcl, 1e-3, verbose-2);
  P.verbose=0;
  if(!useFcl){
    StringA pairs = rai::getParameter<StringA>("collisionPairs", {});
    for(rai::String& s:pairs) P.collisionPairs.append(C[s]->ID);
    P.collisionPairs.reshape(-1,2);
  }

  RRT_PathFinder rrt(P, q0, q1);
  double time = -rai::cpuTime();
  rrt.run();
  time += rai::cpuTime();

  cout <<"rrt time: " <<time <<"sec or " <<1000.*time/double(rrt.iters) <<"msec/iter (tree sizes: " <<rrt.rrt0->getNumberNodes() <<' ' <<rrt.rrtT->getNumberNodes() <<")" <<endl;
}

// =============================================================================

void test_minimalistic(){
  rai::Configuration C;
  C.addFrame("base") -> setPosition({0.,0.,.05});
  C.addFrame("ego", "base")-> setShape(rai::ST_ssBox, {.05, .5, .1, .01}) .setJoint(rai::JT_transXYPhi, {-1.,1.,-1.,1.,-3.,3.}) .setContact(1);
  C.addFrame("obstacle")-> setShape(rai::ST_ssBox, {.05, .5, .1, .01}) .setPosition({.0, .0, .05}) .setContact(1);
  C.addFrame("obstacle0")-> setShape(rai::ST_ssBox, {.05, .5, .1, .01}) .setPosition({.2, .0, .05}) .setContact(1);

  arr q0 = {-.5, .0, .0};
  arr qT = {.5, .0, .0};

  C.setJointState(q0);
  C.view(true);
  C.setJointState(qT);
  C.view(true);
  LOG(0) <<"collidable pairs:\n" <<rai::framesToNames(C.getCollidablePairs());

  run_rrt(C, q0, qT);

}

// =============================================================================

void TEST(RRT){
  rai::Configuration C;
  C.addFile("scene.g");
  C.optimizeTree();

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

  run_rrt(C, q0, q1);
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
  uint count=0;
  for(uint i=0;i<N;i++){
    rndGauss(q);
    C.setJointState(q);
    for(rai::Frame *f:C.frames) if(f->shape && f->shape->cont) f->ensure_X(); //compute pose of all relevant frames

    if(withCollisions){
#if 0
      C.stepFcl();
      bool feas = !C.proxies.N;
#else
      bool feas = C.getCollisionFree();
#endif
      if(!feas) count++;
    }
  }
  time += rai::cpuTime();
  cout <<"kinematics (collisions=" <<withCollisions <<") time: " <<time <<"sec or " <<1000.*time/double(N) <<"msec/query" <<" (#collisions: " <<double(count)/N <<")" <<endl;
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  test_minimalistic(); return 0;

  cout <<"=== RRT test" <<endl;
  testRRT();

  cout <<"=== kinematics test" <<endl;
  testKinematics(false, 100000);

  cout <<"=== kinematics+FCL test" <<endl;
  testKinematics(true, 10000);

  int verbose = rai::getParameter<int>("rrt/verbose", 3);
  if(verbose>2){
    cout <<"used params:\n" <<rai::params()() <<endl;
  }

  return 0;
}
