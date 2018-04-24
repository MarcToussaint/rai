#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::KinematicWorld K("model.g");
//  K.optimizeTree(false);
  K.checkConsistency();
  FILE("z.g") <<K;

  KOMO komo;
  komo.setModel(K);
  komo.setPathOpt(2.5, 10., 5.);
//  komo.setIKOpt();

//  komo.setPosition(1., -1., "endeff", "stick");
//  komo.setTouch(1., -1., "endeff", "stick");

#if 0
  komo.setGrasp(1., "endeff", "stick");
#else
//  komo.setKinematicSwitch(1., true, new rai::KinematicSwitch(rai::SW_effJoint, rai::JT_quatBall, "endeff", "stickTip", K));
//  setTask(time, time, new TM_InsideBox(world, endeffRef, NoVector, object), OT_ineq, NoArr, 1e2);
  komo.setTouch(1., 1., "endeff", "stickTip");
  komo.setKS_stable(1., "endeff", "stickTip");
  komo.setSlow(1.-.1, 1.+.1, 1e0);
#endif

  komo.setTouch(2., -1., "stick", "redBall");
  komo.setTouch(-1.,-1., "stick", "table1", OT_ineq, {}, 1e2);
  komo.setTouch(-1.,-1., "stick", "arm1", OT_ineq, {}, 1e2);

  komo.setSlow(2., -1.,1e0);

  komo.reset();
  komo.checkGradients();
  komo.run();
  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testGrasp();

  return 0;
}

