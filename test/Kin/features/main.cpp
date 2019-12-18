#include <Kin/F_qFeatures.h>
#include <Kin/F_PairCollision.h>
#include <Kin/TM_angVel.h>
#include <Kin/F_dynamics.h>
#include <Kin/F_contacts.h>
#include <Kin/contact.h>
#include <iomanip>

extern bool orsDrawWires;
extern bool rai_Kin_frame_ignoreQuatNormalizationWarning;

//===========================================================================

void testFeature() {
  rai::Configuration K;
  rai::Frame world(K), obj1(&world), obj2(&world);
  world.name = "world";
  obj1.name = "obj1";
  obj2.name = "obj2";
  obj1.set_Q()->setText("t(1 1 1)");
  obj2.set_Q()->setText("t(-1 -1 1)");

  rai::Joint j1(obj1), j2(obj2);
  j1.type = j2.type = rai::JT_free;

  obj1.setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj2.setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj1.setColor({.5,.8,.5,.4});
  obj2.setColor({.5,.5,.8,.4});

  rai::Inertia m1(obj1), m2(obj2);
  m1.defaultInertiaByShape();
  m2.defaultInertiaByShape();

  rai::Contact con(obj1, obj2);

  K.setTimes(.1);
  rai::Configuration K1(K), K2(K);
  ConfigurationL Ktuple = {&K, &K1, &K2};
  uint n=3*K.getJointStateDimension();

  rai::Array<Feature*> F;
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_negScalar));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_vector));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_normal));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_center));
  F.append(new TM_LinAngVel (K, "obj1"));
  F.append(new TM_LinAngVel (K, "obj1")) -> order=2;
  //  F.append(new TM_ZeroAcc (K, "obj1"));
//  F.append(new TM_Energy );
//  F.append(new TM_ContactConstraints (K, "obj1", "obj2"));
  F.append(new F_NewtonEuler (K, "obj1"));

  rai_Kin_frame_ignoreQuatNormalizationWarning=true;

  for(uint k=0;k<100;k++){
    arr x = 5.*(rand(n)-.5);

    bool succ=true;

    for(Feature* f: F){
      cout <<k <<std::setw(30) <<f->shortTag(K) <<' ';
      succ &= checkJacobian(f->vf(Ktuple), x, 1e-5);
    }

    arr y;
    F.first()->__phi(y, NoArr, Ktuple);

    if(!succ)
      K2.watch(true);
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testFeature();

  return 0;
}
