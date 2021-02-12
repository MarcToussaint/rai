#include <KOMO/opt-benchmarks.h>
#include <Optim/solver.h>
#include <KOMO/komo.h>


//===========================================================================

void TEST(KOMO_IK) {
  OptBench_InvKin_Endeff nlp("../../KOMO/switches/model2.g", false);

  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(*nlp.get());

  arr x = S.solve();
  nlp.get()->report(cout, 10);
  S.gnuplot_costs();

  rai::wait();
}

//===========================================================================

void TEST(Skeleton_Handover) {
  OptBench_Skeleton_Pick nlp(rai::_path);
//  OptBench_Skeleton_Handover nlp(rai::_path);
//  OptBench_Skeleton_StackAndBalance nlp(rai::_sequence);

  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(*nlp.get());

//  nlp.get()->report(cout, 0);
  arr x = S.solve();
  nlp.get()->report(cout, 10);
  S.gnuplot_costs();

  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();
  rnd.seed(0);

//  testKOMO_IK();
  testSkeleton_Handover();

  return 0;
}
