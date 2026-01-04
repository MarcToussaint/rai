#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <Geo/i_fcl.h>
#include <Kin/proxy.h>

// =============================================================================

void run_rrt(rai::Configuration& C, const arr& q0, const arr& q1) {

  rai::RRT_PathFinder rrt;
  rrt.setProblem(_shared(C));
  rrt.setStartGoal(q0, q1);
  if(!rrt.opt.useBroadCollisions){
    StringA pairs = rai::getParameter<StringA>("collisionPairs", {});
    rrt.setExplicitCollisionPairs(pairs);
  }

  double time = -rai::cpuTime();
  rrt.solve();
  time += rai::cpuTime();

  cout <<"rrt time: " <<time <<"sec or " <<1000.*time/double(rrt.iters) <<"msec/iter (tree sizes: " <<rrt.rrt0->getNumberNodes() <<' ' <<rrt.rrtT->getNumberNodes() <<")" <<endl;
  if(rrt.opt.verbose>0){
    rrt.view(true, STRING(*rrt.ret), true);
  }
}

// =============================================================================

void test_minimalistic(){
  rai::Configuration C;
  C.addFrame("base") -> setPosition({0.,0.,.05});
  C.addFrame("ego", "base")-> setShape(rai::ST_ssBox, {.05, .5, .1, .01}) .setJoint(rai::JT_transXYPhi, {-1.,-1.,-3.,1.,1.,3.}) .setContact(1);
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
  C.processStructure();

  arr q0,q1;

  { //start pose
    KOMO komo(C, 1., 1, 0, false);
    komo.addControlObjective({}, 0, 1e-2);
    komo.addObjective({}, FS_positionDiff, {"l_gripper", "target1"}, OT_eq, {1e1});
    auto ret = rai::NLP_Solver(komo.nlp(), 0).solve();
    q0 = ret->x;
    cout <<"start pose: " <<*ret <<endl;
  }

  { //end pose
    KOMO komo(C, 1., 1, 0, true);
    komo.addControlObjective({}, 0, 1e-2);
    komo.add_collision(true);
    komo.addObjective({}, FS_positionDiff, {"l_gripper", "target2"}, OT_eq, {1e1});
    auto ret = rai::NLP_Solver(komo.nlp(), 0).solve();
    q1 = ret->x;
    cout <<"start pose: " <<*ret <<endl;
  }

  run_rrt(C, q0, q1);
}

// =============================================================================

void TEST(Mobile){
  rai::Configuration C;
  C.addFile("mobile.g");

  arr q0 = C.getJointState();
  C["ranger_transX"]->setPosition(C["goal"]->getPosition());
  C["ranger_transY"]->setPosition(C["goal"]->getPosition());
  C["ranger_rot"]->setPose(C["goal"]->getPose());
  arr q1 = C.getJointState();
  C.setJointState(q0);

  run_rrt(C, q0, q1);
}

// =============================================================================

void testKinematics(bool fine_postprocess, rai::CollisionQueryMode mode, uint N=100000){
  rnd.seed(1);

  rai::Configuration C;
  C.addFile("scene.g");
  C.processStructure();

  arr q = C.getJointState();

  double time = -rai::cpuTime();
  uint count=0;
  for(uint i=0;i<N;i++){
    rndGauss(q);
    C.setJointState(q);
    for(rai::Frame *f:C.frames) if(f->shape && f->shape->cont) f->ensure_X(); //compute pose of all relevant frames

    if(mode!=rai::_none){
      C.ensure_proxies(fine_postprocess, mode);
      bool feas=true;
      for(const rai::Proxy& p:C.proxies) if(p.d<=0.) { feas=false; break; }
      if(!feas) count++;
    }
  }
  time += rai::cpuTime();
  cout <<"kinematics (fine_postprocess=" <<fine_postprocess <<" queryMode: " <<mode <<") time: " <<1000.*time/double(N) <<"msec/query" <<" (#collisions: " <<double(count)/N <<")" <<endl;
}

// =============================================================================

void Config_null_deleter(rai::Configuration*) {}

void testRRTwPCL(){
  rai::Configuration C;
    C.addFrame("base") ->setPosition({0,0,1});

    C.addFrame("ego", "base") \
        ->setJoint(rai::JT_transXY, {-10.,-10.,10.,10.})
        .setRelativePosition({-1, 0, 0})
        .setShape(rai::ST_sphere, {.05})
        .setColor({0, 1., 1.})
        .setContact(1);

    arr pcl = .3*randn(1000, 3);
    C.addFrame("obstacle", "base")
        ->setPointCloud(pcl)
        .setContact(1);

    C.addFrame("goal") ->setPosition({1, 0, 1}) .setShape(rai::ST_sphere, {.05}) .setColor({0, 1., 0, .9});

    C.view(true);

    arr q0 = C.getJointState();
    arr qT = C.getFrame("goal") ->getPosition();
    qT.resizeCopy(2);

    rai::RRT_PathFinder rrt;
    rrt.opt. set_stepsize(.01) .set_useBroadCollisions(false);
    rrt.setProblem(shared_ptr<rai::Configuration>(&C, &Config_null_deleter));
    rrt.setExplicitCollisionPairs({"ego", "obstacle"});
    rrt.setStartGoal(q0, qT);
    auto ret = rrt.solve();
    cout <<*ret <<endl;
    rrt.view(true);
}

// =============================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  rnd.seed_random();
  // test_minimalistic(); return 0;

  cout <<"=== RRT test" <<endl;
  testRRT();

  cout <<"=== kinematics test" <<endl;
  testKinematics(false, rai::_none, 100000);

  cout <<"=== kinematics test" <<endl;
  testKinematics(false, rai::_none, 100000);

  cout <<"=== kinematics+broadphase" <<endl;
  testKinematics(false, rai::_broadPhaseOnly, 10000);

  cout <<"=== kinematics+binary coll" <<endl;
  testKinematics(false, rai::_binaryCollisionAll, 10000);

  cout <<"=== kinematics+fine" <<endl;
  testKinematics(true, rai::_broadPhaseOnly, 10000);

  testRRTwPCL();

  testMobile();

  int verbose = rai::getParameter<int>("rrt/verbose", 3);
  if(verbose>2){
    cout <<"used params:\n" <<rai::params()() <<endl;
  }

  return 0;
}
