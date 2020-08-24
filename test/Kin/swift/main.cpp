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

void TEST(FCL) {
  rai::Configuration C("../../../../rai-robotModels/pr2/pr2.g");
  C.watch(true);

  C.getCollisionExcludePairIDs(true);

  cout <<"** SWIFT: " <<endl;
  C.stepSwift();
  C.totalCollisionPenetration();
  C.reportProxies();

  cout <<"** FCL: " <<endl;
  C.stepFcl();
  C.totalCollisionPenetration();
  C.reportProxies();
}

void TEST(CollisionTiming){
  rai::Configuration C;
  uint n=1000;
  for(uint i=0;i<n;i++){
    rai::Frame *a = C.addFrame(STRING("obj_i"<<i));

    a->setPose(rai::Transformation().setRandom());
    a->set_X()->pos.z += 1.;
    a->set_X()->pos *= 5.;

    a->setConvexMesh(.2*rai::Mesh().setRandom().V, {}, .02 + .1*rnd.uni());
    a->setColor({.5,.5,.8,.6});
    a->setContact(1);
  }
  C.watch();

  rai::timerStart();
  C.swift().setCutoff(1.);
  cout <<" SWIFT initialization time: " <<rai::timerRead(true) <<endl;

  //  rai::timerStart();
  //  C.fcl();
  //  cout <<" FCL initialization time: " <<rai::timerRead(true) <<endl;

  arr q0,q;
  q0 = C.getJointState();
  rai::timerStart();
  for(uint t=0;t<3;t++){
    for(rai::Frame *a:C.frames){
      a->setPose(rai::Transformation().setRandom());
      a->set_X()->pos.z += 1.;
      a->set_X()->pos *= 5.;
    }

    C.stepSwift();
//    C.stepFcl();
    cout <<"#proxies: " <<C.proxies.N <<endl; //this also calls pair collisions!!
    cout <<"time: " <<rai::timerRead() <<endl;
    cout <<"total penetration: " <<C.totalCollisionPenetration() <<endl; //this also calls pair collisions!!
    cout <<"time: " <<rai::timerRead() <<endl;
     C.reportProxies(FILE("z.col"), 0.);
  }
  cout <<" query time: " <<rai::timerRead(true) <<"sec" <<endl;
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

//  testSwift();
//  testFCL();
  testCollisionTiming();

  return 0;
}

