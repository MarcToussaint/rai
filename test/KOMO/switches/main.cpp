#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::KinematicWorld K("model.g");
  K.optimizeTree(false);
  K.checkConsistency();
  FILE("z.g") <<K;

  K.watch(true);

  KOMO komo;
  komo.setModel(K);
  komo.setPathOpt(2.5, 10., 5.);
  komo.setSquaredQAccelerations();
//  komo.setIKOpt();

//  komo.setPosition(1., -1., "endeff", "stick");
//  komo.add_touch(1., -1., "endeff", "stick");

#if 0
  komo.setGrasp(1., "endeff", "stick");
#else
  komo.add_touch(1., 1., "endeff", "stickTip");
  komo.addSwitch_stable(1., -1., "endeff", "stickTip");
#endif

  komo.add_collision(true);

  komo.add_touch(2., -1., "stick", "redBall");

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

