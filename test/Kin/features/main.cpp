#include <Kin/TM_QuaternionNorms.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_NewtonEuler.h>
#include <Kin/TM_energy.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_ContactConstraints.h>
#include <Kin/contact.h>
#include <iomanip>

extern bool orsDrawWires;
extern bool rai_Kin_frame_ignoreQuatNormalizationWarning;

//===========================================================================

void testFeature() {
  rai::KinematicWorld K;
  rai::Frame world(K), obj1(&world), obj2(&world);
  world.name = "world";
  obj1.name = "obj1";
  obj2.name = "obj2";
  obj1.Q = "t(1 1 1)";
  obj2.Q = "t(-1 -1 1)";

  rai::Joint j1(obj1), j2(obj2);
  j1.type = j2.type = rai::JT_free;

  rai::Shape s1(obj1), s2(obj2);
  s1.type() = s2.type() = rai::ST_ssBox; //rai::ST_ssCvx; //ST_mesh;
//  s1.sscCore().setRandom();      s2.sscCore().setRandom();
  s1.mesh().C = {.5,.8,.5,.4};   s2.mesh().C = {.5,.5,.8,.4};
  s1.size(3) = rnd.uni(.01, .3); s2.size(3) = rnd.uni(.01, .3);
//  if(rnd.uni()<.2) s1.sscCore().setDot(); else if(rnd.uni()<.2) s1.sscCore().setLine(.5);
//  if(rnd.uni()<.2) s2.sscCore().setDot(); else if(rnd.uni()<.2) s1.sscCore().setLine(.5);
  s1.createMeshes();
  s2.createMeshes();

  rai::Inertia m1(obj1), m2(obj2);
  m1.defaultInertiaByShape();
  m2.defaultInertiaByShape();

  rai::Contact con(obj1, obj2);

  K.calc_fwdPropagateFrames();

  K.setTimes(.1);
  rai::KinematicWorld K1(K), K2(K);
  WorldL Ktuple = {&K, &K1, &K2};
  uint n=3*K.getJointStateDimension();

  rai::Array<Feature*> F;
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_negScalar));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_vector));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_normal));
  F.append(new TM_PairCollision (K, "obj1", "obj2", TM_PairCollision::_center));
  F.append(new TM_LinAngVel (K, "obj1"));
  F.append(new TM_LinAngVel (K, "obj1")) -> order=2;
  F.append(new TM_ZeroAcc (K, "obj1"));
//  F.append(new TM_Energy );
//  F.append(new TM_ContactConstraints (K, "obj1", "obj2"));
  F.append(new TM_NewtonEuler (K, "obj1"));

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

    if(!succ) K2.watch(true);
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testFeature();

  return 0;
}
