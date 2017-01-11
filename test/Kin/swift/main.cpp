#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Gui/opengl.h>

void TEST(Swift) {
  mlr::KinematicWorld G("swift_test.g");

  G.swift().setCutoff(2.);
  G.stepSwift();

  uint t;
  for(t=0;t<50;t++){
    G.bodies(0)->X.addRelativeTranslation(0,0,-.01);
    G.bodies(0)->X.addRelativeRotationDeg(10,1,0,0);
    G.calc_fwdPropagateFrames();

    G.stepSwift();

    G.reportProxies();

    G.watch(false);
    mlr::wait(.1);
  }
}

void TEST(CollisionTiming){
  mlr::KinematicWorld G("../../../data/configurations/schunk.ors");

  G.swift().setCutoff(1.);

  arr q0,q;
  G.getJointState(q0);
  mlr::timerStart();
  uint t;
  for(t=0;t<1000;t++){
    if(!(t%1)){ q = q0;  rndGauss(q,.1,true); }
    G.setJointState(q);
    G.stepSwift();
//    G.reportProxies();
//    G.watch(false);
//    G.watch(true);
  }
  double time = mlr::timerRead();
  cout <<t <<" collision queries: sec=" <<time <<endl;
  CHECK(time>0.01 && time<1.,"strange time for collision checking!");
}


int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testSwift();
  testCollisionTiming();

  return 0;
}

