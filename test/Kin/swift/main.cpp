#include <Kin/kin.h>
#include <Kin/kin_swift.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>

void TEST(Swift) {
  rai::Configuration K("swift_test.g");

  K.swift().setCutoff(2.);
  K.stepSwift();
  K.orsDrawProxies=true;

  uint t;
  for(t=0;t<50;t++){
    K.frames(0)->set_X()->addRelativeTranslation(0,0,-.01);
    K.frames(0)->set_X()->addRelativeRotationDeg(10,1,0,0);

    K.stepSwift();

    K.reportProxies();

    K.watch(true);
    rai::wait(.1);
  }
}

void TEST(CollisionTiming){
  rai::Configuration K("../../../../rai-robotModels/pr2/pr2.g");

  K.swift().setCutoff(1.);

  cout <<"# objects:" <<K.swift().countObjects() <<endl;

  arr q0,q;
  q0 = K.getJointState();
  rai::timerStart();
  uint t;
  for(t=0;t<1000;t++){
    if(!(t%1)){ q = q0;  rndGauss(q,.1,true); }
    K.setJointState(q);
    K.stepSwift();
//    G.reportProxies();
//    G.watch(false);
//    G.watch(true);
  }
  double time = rai::timerRead();
  cout <<t <<" collision queries: sec=" <<time <<endl;
  CHECK(time>0.01 && time<1.,"strange time for collision checking!");
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testSwift();
//  testCollisionTiming();

  return 0;
}

