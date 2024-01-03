#include <KOMO/komo-ext.h>
#include <string>
#include <map>
#include <Core/graph.h>

using namespace std;

//===========================================================================

void TEST(KomoSequence){

  rai::Configuration K("model.g");
  K.optimizeTree(false);
  makeConvexHulls(K.frames);

  KOMO_ext komo;
  komo.setConfig(K);
  komo.setTiming(2., 20, 10.);

  komo.addControlObjective({}, 2);// setSquaredQAccVelHoming();
  komo.addQuaternionNorms();

  komo.setGrasp(1., 1.8, "humanR", "Long1");
  komo.setPlace(1.8, "humanR", "Long1", "tableL");

  komo.setGrasp(1., 1.8, "humanL", "Long2");
  komo.setPlace(1.8, "humanL", "Long2", "tableR");

  komo.optimize();

  rai::Graph result = komo.report(false, true, true);

  for(uint i=0;i<2;i++) if(!komo.displayTrajectory(.1, true)) break;
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testKomoSequence();

  return 0;
}

