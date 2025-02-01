#include <KOMO/opt-benchmarks.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>

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

void test(str problemName, bool fullyUniform=false){
  uint s=0;
  for(uint k=0;k<1;k++){
    Problem P;
    P.load(problemName);

    for(uint i=0;i<20;i++){
      NLP_Solver S;
      S.setProblem(P.nlp);
      S.setSolver(NLPS_slackGN);
      if(fullyUniform){
        S.setInitialization(P.nlp->getUniformSample());
        P.komo->pathConfig.setJointState(S.x);
      }else{
        P.komo->initRandom();
        S.setInitialization(P.nlp->getInitializationSample());
      }

      // P.komo->view(true);
      auto ret = S.solve();
      cout <<*ret <<endl;
      //cout <<P.komo->report() <<endl;
      P.komo->view(ret->feasible);
      if(ret->feasible){
        P.komo->get_viewer()->visualsOnly();
        P.komo->view_play(false, 0, .2, STRING("z."<<s++<<".vid/"));
      }
    }
  }
  // P.nlp->report(cout, 10);
  // S.gnuplot_costs();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();
  rnd.seed(0);

  // testKOMO_IK();
  // testSkeleton_Handover();

  // test("IK-obstacle", true);
  test("push");
  // test("stableSphere");

  return 0;
}
