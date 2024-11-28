#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/NLP_Solver.h>
#include <Optim/lagrangian.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  NLP_Viewer(nlp).display();

  rai::wait();
}

//===========================================================================

void TEST(Solver) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  if(sid==NLPS_logBarrier){
    NLP_Viewer(nlp).display(1., 1.);
  }else{
    NLP_Viewer(nlp).display(1., -1.);
  }
  rai::wait(.5, true);

//  arr x = nlp->getInitializationSample();
//  checkJacobianCP(*nlp, x, 1e-4);

  arr x_init = rai::getParameter<arr>("x_init", {});
  NLP_Solver S;

  S.setSolver(sid);
  S.setProblem(nlp);
  if(x_init.N) S.setInitialization(x_init);
  if(sid==NLPS_augmentedLag || sid==NLPS_squaredPenalty || sid==NLPS_logBarrier){
    while(!S.step()){
      if(sid==NLPS_logBarrier){
        NLP_Viewer(nlp, S.P). display(S.optCon->L.mu, S.optCon->L.muLB);
      }else{
        NLP_Viewer(nlp, S.P). display(S.optCon->L.mu);
      }
      if(S.opt.verbose>2) rai::wait(.2, true);
    }
  }else{
    S.solve();
  }

  arr path = catCol(S.getTrace_x(), S.getTrace_costs());
  FILE("z.path") <<path.modRaw();

  cout <<"\nRESULT:\n" <<*S.ret <<"\nx: " <<S.ret->x <<"\ndual: " <<S.ret->dual <<endl;

  NLP_Viewer(nlp, S.P). display();
  // displayNLP(nlp, S.getTrace_x(), S.getTrace_costs());
//  gnuplot("load 'plt'", false, false);
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

//  testDisplay();
  testSolver();

  return 0;
}
