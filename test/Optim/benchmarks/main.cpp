#include <KOMO/opt-benchmarks.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/komo.h>

//===========================================================================

void TEST(KOMO_IK) {
  OptBench_InvKin_Endeff nlp("../../KOMO/switches/model2.g", false);

  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp.get());

  auto ret = S.solve();
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
  S.setProblem(nlp.get());

//  nlp.get()->report(cout, 0);
  auto ret = S.solve();
  nlp.get()->report(cout, 10);
  S.gnuplot_costs();

  rai::wait();
}

//===========================================================================

void test(str problemName){
  Problem P;
  P.load(problemName);

  NLP_Solver S;
  S.setProblem(P.nlp);
  S.setSolver(NLPS_slackGN);
  for(uint k=0;k<10;k++){
    S.setInitialization(P.nlp->getUniformSample());
    P.komo->pathConfig.setJointState(S.x);
    // P.komo->initRandom(5);
    P.komo->view(true);
    // S.setInitialization(P.nlp->getInitializationSample());
    auto ret = S.solve();
    cout <<*ret <<endl;
    P.komo->view(true);
  }
  // P.nlp->report(cout, 10);
  S.gnuplot_costs();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();
  rnd.seed(0);

  // testKOMO_IK();
  // testSkeleton_Handover();

  test("push");

  return 0;
}
