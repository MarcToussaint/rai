#include <KOMO/komo-ext.h>
#include <string>
#include <map>
#include <Core/graph.h>

using namespace std;

//===========================================================================

void TEST(KomoSequence){

  rai::KinematicWorld K("model.g");
  K.optimizeTree(false);
  makeConvexHulls(K.frames);

  KOMO_ext komo;
  komo.setModel(K);
  komo.setPathOpt(2., 20, 10.);
//  komo.setConfigFromFile();

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredQAccelerations();

  komo.setGrasp(1., "humanR", "Long1");
  komo.setPlace(1.8, "humanR", "Long1", "tableL");

  komo.setGrasp(1., "humanL", "Long2");
  komo.setPlace(1.8, "humanL", "Long2", "tableR");

  komo.reset();
  komo.run();

  Graph result = komo.getReport(true);

  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testKomoSequence();

  return 0;
}

