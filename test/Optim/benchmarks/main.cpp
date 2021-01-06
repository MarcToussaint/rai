#include <KOMO/opt-benchmarks.h>
#include <Optim/solver.h>

void TEST(KOMO_IK) {
  OptBench_InvKin_Endeff nlp("../../KOMO/switches/model2.g", true);

  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(*nlp.get());
  arr x = S.solve();

  nlp.get()->report(cout, 4);
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testKOMO_IK();

  return 0;
}
