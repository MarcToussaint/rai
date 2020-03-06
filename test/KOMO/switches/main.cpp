#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::Configuration K("model.g");
  K.optimizeTree();
  K.checkConsistency();

  KOMO komo;
  komo.setModel(K);
  komo.setPathOpt(2.5, 10., 5.);
  komo.setSquaredQAccVelHoming();

#if 0
  komo.setGrasp(1., "endeff", "stick");
#else
  komo.add_touch(1., 1., "endeff", "stickTip");
  komo.addSwitch_stable(1., -1., "endeff", "stick");
#endif

  komo.add_collision(true);

  komo.add_touch(2., -1., "stick", "redBall");

  komo.setSlow(2., -1.,1e0);

  komo.reset();
  komo.run();
  komo.checkGradients();

  rai::Graph result = komo.getReport(true);

  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testGrasp();

  return 0;
}

