#include <KOMO/komo.h>
#include <string>
#include <map>

using namespace std;

//===========================================================================

void TEST(KomoSequence){
  
  KOMO komo;
  komo.setConfigFromFile();

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setFixEffectiveJoints();
  komo.setFixSwitchedObjects();
  komo.setSquaredQAccelerations();

  komo.setGrasp(1., "humanR", "Long1");
  komo.setPlace(1.8, "humanR", "Long1", "tableL");
  komo.setSlowAround(1., .1, 1e3);

  komo.setGrasp(1., "humanL", "Long2");
  komo.setPlace(1.8, "humanL", "Long2", "tableR");

  komo.reset();
  komo.run();

  Graph result = komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  testKomoSequence();

  return 0;
}

