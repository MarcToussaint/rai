#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>

//===========================================================================

void TEST(Easy){
  mlr::KinematicWorld G("test.ors");
  cout <<"configuration space dim=" <<G.q.N <<endl;
  KOMO komo;
  komo.setMoveTo(G, *G.getFrameByName("endeff"), *G.getFrameByName("target"));
//  komo.checkGradients();
//  komo.setSpline(3);
//  komo.checkGradients();
  komo.run();
  komo.plotTrajectory();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(EasyPR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of thbe PR2
  mlr::KinematicWorld G("model.kvg");
  G.optimizeTree();
  makeConvexHulls(G.frames);
  G.calc_fwdPropagateFrames();
  cout <<"configuration space dim=" <<G.q.N <<endl;
  double rand = mlr::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(mlr::getParameter<uint>("rndSeed", 0));
    rndGauss(G.q,rand,true);
    G.setJointState(G.q);
  }
  KOMO komo;
  komo.setMoveTo(G, *G.getFrameByName("endeff"), *G.getFrameByName("target"));
//  komo.setSpline(10);
  komo.run();
  cout <<komo.getReport(false) <<endl;
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(FinalPosePR2){
  mlr::KinematicWorld K("model.kvg");
  K.optimizeTree();
  makeConvexHulls(K.frames);
  cout <<"configuration space dim=" <<K.q.N <<endl;
  arr x = finalPoseTo(K, *K.getFrameByName("endeff"), *K.getFrameByName("target"));
  K.setJointState(x.reshape(x.N));
  K.gl().watch();
}

//===========================================================================

void TEST(EasyAlign){
  mlr::KinematicWorld G("test.ors");
  KOMO komo;
  komo.setMoveTo(G, *G.getFrameByName("endeff"), *G.getFrameByName("target"), 7); //aligns all 3 axes
//  komo.setSpline(5);
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(EasyAlign2){
  mlr::KinematicWorld G("test.ors");
  G.optimizeTree();
  mlr::Frame *a = G.getFrameByName("target");
  a->X.addRelativeRotationDeg(90,1,0,0);
  KOMO komo;
  komo.setMoveTo(G, *G.getFrameByName("endeff"), *a, 7);
//  komo.setSpline(10);
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  testEasy();
  testEasyAlign();
  testEasyAlign2();
  testEasyPR2();

  return 0;
}


