#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>

using namespace std;

//===========================================================================

void TEST(Grasp){
  rai::Configuration K("model.g");
//  K.optimizeTree();
  K.checkConsistency();

  KOMO komo;
  komo.setModel(K);
  komo.setTiming(2.5, 10., 5.);
  komo.add_qControlObjective({}, 2, 1.);

#if 0
  komo.setGrasp(1., "endeff", "stick");
#else
  komo.addObjective({1.}, FS_distance, {"endeff", "stickTip"}, OT_eq, {1e2});
  komo.addSwitch_stable(1., -1., "endeff", "stickTip");
#endif

  komo.add_collision(true);

  komo.addObjective({2.}, FS_distance, {"stick", "redBall"}, OT_eq, {1e2});

  komo.setSlow(2., -1.,1e0);

//  komo.animateOptimization = 2;
//  komo.verbose = 8;
  komo.optimize();
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

