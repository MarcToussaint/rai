#include <Kin/F_qFeatures.h>
#include <Kin/F_collisions.h>
#include <Kin/F_pose.h>
#include <Kin/F_forces.h>
#include <Kin/forceExchange.h>
#include <iomanip>
#include <Kin/featureSymbols.h>

extern bool orsDrawWires;
extern bool rai_Kin_frame_ignoreQuatNormalizationWarning;

//===========================================================================

void testFeature() {
  rai::Configuration C;
  C.addFrame("world");
  rai::Frame *obj1 = C.addFrame("obj1", "world");
  rai::Frame *obj2 = C.addFrame("obj2", "world");
  obj1->setRelativePosition({1.,1.,1.});
  obj2->setRelativePosition({-1.,-1.,1.});
  obj1->setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj2->setShape(rai::ST_ssBox, ARR(1.,1.,1.,rnd.uni(.01, .3)));
  obj1->setColor({.5,.8,.5,.4});
  obj2->setColor({.5,.5,.8,.4});

  obj1->setJoint(rai::JT_free);
  obj2->setJoint(rai::JT_transXYPhi);

  obj1->setMass(1.);
  obj2->setMass(1.);

  rai::ForceExchange con(*obj1, *obj2);

  C.setTimes(.1);

  rai::Configuration Ctuple;
  Ctuple.addFramesCopy(C.frames);
  Ctuple.addFramesCopy(C.frames);
  Ctuple.addFramesCopy(C.frames);
  Ctuple.jacMode = rai::Configuration::JM_rowShifted;

  uint n=Ctuple.getJointStateDimension();
  arr q=Ctuple.getJointState();
  Ctuple.setJointState(q);

  rai::Array<std::shared_ptr<Feature>> F;
  F.append(make_shared<F_PairCollision>(F_PairCollision::_negScalar)) ->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_PairCollision>(F_PairCollision::_vector)) ->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_PairCollision>(F_PairCollision::_center)) ->setFrameIDs({"obj1", "obj2"}, C);
  F.append(make_shared<F_LinAngVel>()) ->setFrameIDs({"obj1"}, C);
  F.append(make_shared<F_LinAngVel>()) ->setFrameIDs({"obj2"}, C) .setOrder(2);
  F.append(symbols2feature(FS_position, {"obj1"}, C));
  F.append(symbols2feature(FS_positionDiff, {"obj1", "obj2"}, C));
  F.append(symbols2feature(FS_pose, {"obj1"}, C)) ->setOrder(0);
  F.append(symbols2feature(FS_pose, {"obj2"}, C)) ->setOrder(1);
  F.append(symbols2feature(FS_pose, {"obj1"}, C)) ->setOrder(2);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) ->setOrder(0);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) ->setOrder(1);
  F.append(symbols2feature(FS_poseRel, {"obj1", "obj2"}, C)) ->setOrder(2);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) ->setOrder(0);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) ->setOrder(1);
  F.append(symbols2feature(FS_poseDiff, {"obj1", "obj2"}, C)) ->setOrder(2);
  F.append(make_shared<F_NewtonEuler>()) ->setFrameIDs({"obj1"}, C);

  rai_Kin_frame_ignoreQuatNormalizationWarning=true;

  rai::timerStart();
  for(uint k=0;k<100;k++){
    arr x = 5.*(rand(n)-.5);

    bool succ=true;

    for(ptr<Feature>& f: F){
      cout <<k <<std::setw(30) <<f->shortTag(C) <<' ';
      succ &= checkJacobian(f->vf(Ctuple), x, 1e-5);
    }

    arr y;
    F.first()->__phi(y, NoArr, Ctuple);

    if(!succ) Ctuple.watch(true);
  }
  cout <<"*** COMPUTE TIME: " <<rai::timerRead() <<"sec" <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  testFeature();

  return 0;
}
