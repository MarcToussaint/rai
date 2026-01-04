#include <Kin/kin.h>
//#include <Kin/kin_swift.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>

bool interactive=false;

void TEST(Engine) {
  rai::Configuration C("../../../../rai-robotModels/pr2/pr2.g");
  C.view(true);

  C.getCollisionExcludePairIDs(true);

  // cout <<"** SWIFT: " <<endl;
  // C.stepSwift();
  // C.getTotalPenetration();
  // C.reportProxies();

  cout <<"** FCL: " <<endl;
  C.coll_stepFcl(rai::_broadPhaseOnly);
  C.coll_totalViolation();
  C.coll_reportProxies();
}

void TEST(CollisionTiming){
  //-- create a random scene
  rai::Configuration C;
  uint n=1000;
  for(uint i=0;i<n;i++){
    rai::Frame *a = C.addFrame(STRING("obj_i"<<i));

    a->setPose(rai::Transformation().setRandom());
    a->set_X()->pos.z += 1.;
    a->set_X()->pos *= 5.;

    a->setConvexMesh(.3*rai::Mesh().setRandom().V, {}, .02 + .1*rnd.uni());
    a->setColor({.5,.5,.8,.6});
    a->setContact(1);
  }

  C.view();

  rai::timerStart();
  C.coll_engine();
  cout <<"engine initialization time: " <<rai::timerRead(true) <<endl;

  arr q0,q;
  q0 = C.getJointState();
  rai::timerStart();
  for(uint t=0;t<5;t++){
    for(rai::Frame *a:C.frames){
      a->setPose(rai::Transformation().setRandom());
      a->set_X()->pos.z += 1.;
      a->set_X()->pos *= 5.;
    }

    rai::timerStart();

    cout <<"-------------------- t=" <<t <<" ---------" <<endl;
    

    C.coll_stepFcl(rai::_broadPhaseOnly);
    cout <<"#proxies: " <<C.proxies.N <<endl;
    cout <<"time: " <<rai::timerRead(true) <<endl;
    // C.view(interactive, "FCL result");
    cout <<"total penetration: " <<C.coll_totalViolation() <<endl; //this also calls pair collisions!!
    cout <<"time: " <<rai::timerRead(true) <<endl;
    C.coll_reportProxies(FILE("z.col"), 0.);
    C.view(interactive, "FCL result");
  }
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testEngine();

  rnd.seed(0);
  testCollisionTiming();

  return 0;
}

