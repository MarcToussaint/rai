#include <Logic/fol.h>
#include <Logic/folWorld.h>

//===========================================================================

void testFastDownward(){
  rai::String file = rai::raiPath("bin/src_lgpPlayer/pnp.g");
  if(rai::checkParameter<rai::String>("file")) file = rai::getParameter<rai::String>("file");
  if(rai::argc>1 && rai::argv[1][0]!='-') file = rai::argv[1];

  rai::FOL_World world(file);
  world.verbose = rai::getParameter<int>("verbose", 2);
  world.reset_state();

  rai::String plan = world.callPDDLsolver();

  world.addDecisionSequence(plan);
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

  testFastDownward();
}
