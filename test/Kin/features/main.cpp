#include <Kin/F_qFeatures.h>
#include <Kin/F_PairCollision.h>
#include <Kin/TM_angVel.h>
#include <Kin/F_dynamics.h>
#include <Kin/F_pose.h>
#include <Kin/F_contacts.h>
#include <Kin/forceExchange.h>
#include <iomanip>

extern bool orsDrawWires;
extern bool rai_Kin_frame_ignoreQuatNormalizationWarning;

//===========================================================================

void testFeature() {
  rai::Configuration C;
  rai::Frame world(C), obj1(&world), obj2(&world);
  world.name = "world";
  obj1.name = "obj1";
  obj2.name = "obj2";
  obj1.setRelativePosition({1.,1.,1.});
  obj2.setRelativePosition({-1.,-1.,1.});
  obj1.setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj2.setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj1.setColor({.5,.8,.5,.4});
  obj2.setColor({.5,.5,.8,.4});

  rai::Joint j1(obj1, rai::JT_free);
  rai::Joint j2(obj2, rai::JT_transXYPhi);

  rai::Inertia m1(obj1), m2(obj2);
  m1.defaultInertiaByShape();
  m2.defaultInertiaByShape();

  rai::ForceExchange con(obj1, obj2);

  C.setTimes(.1);

  rai::Configuration C1(C), C2(C);
  ConfigurationL Ktuple = {&C, &C1, &C2};

  uint n=3*C.getJointStateDimension();

  rai::Array<ptr<Feature>> F;
  F.append(make_shared<F_PairCollision>(C, "obj1", "obj2", F_PairCollision::_negScalar));
  F.append(make_shared<F_PairCollision>(C, "obj1", "obj2", F_PairCollision::_vector));
  F.append(make_shared<F_PairCollision>(C, "obj1", "obj2", F_PairCollision::_center));
  F.append(make_shared<TM_LinAngVel>(C, "obj1"));
  F.append(make_shared<TM_LinAngVel>(C, "obj2")) -> order=2;
  F.append(make_shared<F_Pose>(C, "obj1"));
  F.append(make_shared<F_Pose>(C, "obj2")) -> order=1;
  F.append(make_shared<F_Pose>(C, "obj1")) -> order=2;
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) -> order=0;
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) -> order=1;
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) -> order=2;
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) -> order=0;
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) -> order=1;
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) -> order=2;
  F.append(make_shared<F_NewtonEuler>(C, "obj1"));

  rai_Kin_frame_ignoreQuatNormalizationWarning=true;

  for(uint k=0;k<100;k++){
    arr x = 5.*(rand(n)-.5);

    bool succ=true;

    for(ptr<Feature>& f: F){
      cout <<k <<std::setw(30) <<f->shortTag(C) <<' ';
      succ &= checkJacobian(f->vf(Ktuple), x, 1e-5);
    }

    arr y;
    F.first()->__phi(y, NoArr, Ktuple);

    if(!succ)
      C2.watch(true);
  }
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testFeature();

  return 0;
}
