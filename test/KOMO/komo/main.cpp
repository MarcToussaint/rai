#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>

//===========================================================================

void TEST(Easy){
  mlr::KinematicWorld G("test.ors");
  cout <<"configuration space dim=" <<G.q.N <<endl;
  KOMO komo;
  komo.setMoveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
//  komo.checkGradients();
//  komo.setSpline(3);
//  komo.checkGradients();
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(EasyPR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of the PR2
  mlr::KinematicWorld G("model.kvg");
  G.meldFixedJoints();
  G.removeUselessBodies();
//  mlr::KinematicWorld G2=G;
//  G2.meldFixedJoints();
//  G2.removeUselessBodies();
//  G2 >>FILE("z.ors");
  makeConvexHulls(G.shapes);
  for(mlr::Shape *s:G.shapes) s->cont=true;
  cout <<"configuration space dim=" <<G.q.N <<endl;
  double rand = mlr::getParameter<double>("KOMO/moveTo/randomizeInitialPose", .0);
  if(rand){
    rnd.seed(mlr::getParameter<uint>("rndSeed", 0));
    rndGauss(G.q,rand,true);
    G.setJointState(G.q);
  }
  KOMO komo;
  komo.setMoveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
//  komo.setSpline(10);
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(FinalPosePR2){
  mlr::KinematicWorld G("model.kvg");
  G.meldFixedJoints();
  G.removeUselessBodies();
  makeConvexHulls(G.shapes);
  for(mlr::Shape *s:G.shapes) s->cont=true;
  cout <<"configuration space dim=" <<G.q.N <<endl;
  arr x = finalPoseTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  G.setJointState(x.reshape(x.N));
  G.gl().watch();
}

//===========================================================================

void TEST(EasyAlign){
  mlr::KinematicWorld G("test.ors");
  KOMO komo;
  komo.setMoveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 7); //aligns all 3 axes
  komo.setSpline(5);
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

void TEST(EasyAlign2){
  mlr::KinematicWorld G("test.ors");
  mlr::Shape *s = G.getShapeByName("target");
  s->rel.addRelativeRotationDeg(90,1,0,0);
  KOMO komo;
  komo.setMoveTo(G, *G.getShapeByName("endeff"), *s, 7);
  komo.setSpline(10);
  komo.run();
  for(uint i=0;i<2;i++) komo.displayTrajectory();
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

//  testEasy();
//  testEasyAlign();
//  testEasyAlign2();
  testEasyPR2();
//  testFinalPosePR2();
  return 0;
}


