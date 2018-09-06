#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <KOMO/komo-ext.h>
#include <Kin/TM_ContactConstraints.h>
#include <Kin/TM_time.h>
#include <Kin/TM_default.h>

//===========================================================================

void TEST(Easy){
  rai::KinematicWorld K("arm.g");
  cout <<"configuration space dim=" <<K.q.N <<endl;

  //-- add time DOFs
//  K.addTimeJoint();

  KOMO_ext komo;
  komo.setModel(K, true);
  komo.setPathOpt(1., 100, 5.);

  //-- set a time optim objective
  komo.addObjective(-1., -1., new TM_Time(), OT_sos, {}, 1e2, 1); //smooth time evolution
  komo.addObjective(-1., -1., new TM_Time(), OT_sos, {komo.tau}, 1e1, 0); //prior on timing

  komo.setPosition(1., 1., "endeff", "target", OT_sos);
  komo.setSlowAround(1., .05);
//  komo.setTask(.3, .7, new TM_Default(TMT_posDiff, komo.world, "endeff", NoVector, "target"), OT_sos, NoArr, 1e2, 2);
  komo.add_collision(false);
  komo.reportProblem();

  komo.reset();
//  komo.setSpline(5);
  komo.run();
  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  komo.plotTrajectory();
  cout <<komo.getReport(true) <<endl;
//  komo.reportProxies();
  komo.checkGradients();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
//  while(komo.displayTrajectory());
}

//===========================================================================

void TEST(Align){
  rai::KinematicWorld K("arm.g");
  cout <<"configuration space dim=" <<K.q.N <<endl;
  KOMO_ext komo;
  komo.setModel(K);
  komo.setPathOpt(1., 100, 5.);

  komo.setPosition(1., 1., "endeff", "target");
  komo.setOrientation(1., 1., "endeff", "target", OT_eq);
  komo.setSlowAround(1., .02);
  komo.add_collision(false);
//  komo.setTask(-1., -1., new TM_ContactConstraints(), OT_ineq, NoArr, 1e1);

  komo.reset();
  komo.run();
  komo.plotTrajectory();
  komo.getReport(true);
  for(uint i=0;i<2;i++) komo.displayTrajectory();
//  while(komo.displayTrajectory());
}

//===========================================================================

void TEST(PR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of thbe PR2
  rai::KinematicWorld K("model.g");
  K.pruneRigidJoints();
  K.optimizeTree();
  makeConvexHulls(K.frames);
  cout <<"configuration space dim=" <<K.getJointStateDimension() <<endl;
  double rand = rai::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(rai::getParameter<uint>("rndSeed", 0));
    rndGauss(K.q,rand,true);
    K.setJointState(K.q);
  }

  KOMO_ext komo;
  komo.setModel(K);
  komo.setPathOpt(1., 100, 10.);
  komo.setPosition(1., 1., "endeff", "target");
  komo.setSlowAround(1., .02);
  komo.add_collision(false);
//  komo.setTask(-1., -1., new TM_ContactConstraints(), OT_ineq);
//  komo.setTask(-1., -1., new TM_ContactConstraints(), OT_ineq, NoArr, 1e2);

  komo.reset();
//  komo.setSpline(10);
  komo.run();
  komo.plotTrajectory();
  komo.getReport(true);
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(FinalPosePR2){
  rai::KinematicWorld K("model.g");
  K.pruneRigidJoints();
  K.optimizeTree();
  makeConvexHulls(K.frames);
  cout <<"configuration space dim=" <<K.getJointStateDimension() <<endl;
  arr x = finalPoseTo(K, *K.getFrameByName("endeff"), *K.getFrameByName("target"));
  K.setJointState(x.reshape(x.N));
  K.gl().watch();
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testEasy();
//  testAlign();
//  testPR2();

  return 0;
}

