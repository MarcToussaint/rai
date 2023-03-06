#include <Kin/kin.h>
//#include <Kin/kin_swift.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>

/*void TEST(Swift) {
  rai::Configuration C("swift_test.g");

  C.swift()->cutoff = 2.;
  C.stepSwift();
  C.gl()->ensure_gl().drawOptions.drawProxies=true;

  uint t;
  for(t=0;t<50;t++){
    C.frames(0)->set_X()->addRelativeTranslation(0,0,-.01);
    C.frames(0)->set_X()->addRelativeRotationDeg(10,1,0,0);

    C.stepSwift();

    C.reportProxies();

    C.view(true);
    rai::wait(.1);
  }
  }*/

void TEST(FCL) {
  rai::Configuration C("../../../../rai-robotModels/pr2/pr2.g");
  C.view(true);

  C.getCollisionExcludePairIDs(true);

  // cout <<"** SWIFT: " <<endl;
  // C.stepSwift();
  // C.getTotalPenetration();
  // C.reportProxies();

  cout <<"** FCL: " <<endl;
  C.stepFcl();
  C.getTotalPenetration();
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

  C.view();

  //rai::timerStart();
  //C.swift(); //.setCutoff(.);
  //cout <<" SWIFT initialization time: " <<rai::timerRead(true) <<endl;

  rai::timerStart();
  C.fcl();
  cout <<" FCL initialization time: " <<rai::timerRead(true) <<endl;

  arr q0,q;
  q0 = C.getJointState();
  rai::timerStart();
  for(uint t=0;t<3;t++){
    for(rai::Frame *a:C.frames){
      a->setPose(rai::Transformation().setRandom());
      a->set_X()->pos.z += 1.;
      a->set_X()->pos *= 5.;
    }

    rai::timerStart();

    cout <<"-------------------- t=" <<t <<" ---------" <<endl;
    
    // C.stepSwift();
    // cout <<"SWIFT:" <<endl;
    // cout <<"#proxies: " <<C.proxies.N <<endl; //this also calls pair collisions!!
    // cout <<"time: " <<rai::timerRead(true) <<endl;
    // cout <<"total penetration: " <<C.getTotalPenetration() <<endl; //this also calls pair collisions!!
    // cout <<"time: " <<rai::timerRead(true) <<endl;
    // C.reportProxies(FILE("z.col"), 0.);
    // V.setConfiguration(C, "SWIFT result", true);

    C.stepFcl();
    cout <<"FCL:" <<endl;
    cout <<"#proxies: " <<C.proxies.N <<endl; //this also calls pair collisions!!
    cout <<"time: " <<rai::timerRead(true) <<endl;
    cout <<"total penetration: " <<C.getTotalPenetration() <<endl; //this also calls pair collisions!!
    cout <<"time: " <<rai::timerRead(true) <<endl;
    C.reportProxies(FILE("z.col"), 0.);
    C.view(true, "FCL result");
  }
  cout <<" query time: " <<rai::timerRead(true) <<"sec" <<endl;
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  //  testSwift();
  testFCL();
  testCollisionTiming();

  return 0;
}

