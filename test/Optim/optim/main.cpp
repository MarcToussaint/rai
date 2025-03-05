#include <Optim/testProblems_Opt.h>
#include <functional>
#include <Optim/NLP_Solver.h>
// #include <Optim/NLP_Sampler.h>
#include <Optim/SlackGaussNewton.h>
#include <Optim/lagrangian.h>
#include <Optim/constrained.h>
#include <Core/arrayDouble.h>
#include <Optim/opt-ipopt.h>
#include <KOMO/testProblems_KOMO.h>

// #include <Kin/kin.h>
// #include <Kin/frame.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  NLP_Viewer(nlp).display();

  rai::wait();
}

//===========================================================================

void TEST(Solver) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  rai::Enum<rai::OptMethod> sid (rai::getParameter<rai::String>("solver"));
  if(sid==rai::M_logBarrier){
    NLP_Viewer(nlp).display(1., 1.);
  }else{
    NLP_Viewer(nlp).display(1., -1.);
  }
  rai::wait(.5, true);

  //  arr x = nlp->getInitializationSample();
  //  checkJacobianCP(*nlp, x, 1e-4);

  arr x_init = rai::getParameter<arr>("x_init", {});
  rai::NLP_Solver S;

  S.setSolver(sid);
  S.setProblem(nlp);
  if(x_init.N) S.setInitialization(x_init);
  if(sid==rai::M_augmentedLag || sid==rai::M_squaredPenalty || sid==rai::M_logBarrier){
    while(!S.step()){
      if(sid==rai::M_logBarrier){
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

void testSpherePacking(){
  auto P =   std::make_shared<SpherePacking>(50, .21, false);
  std::cout <<P->reportSignature() <<std::endl;

  //-- sample a feasible (=no collision) solution
  rai::SlackGaussNewton sam(P);
  sam.opt.set_stopEvals(100) .set_interiorPadding(1e-2) .set_stopGTolerance(1e-4);
  auto retSam = sam.solve();
  cout <<*retSam <<endl;
  // P->report(std::cout, 10);


  //-- optimize
  rai::NLP_Solver S;
  S.setProblem(P);
  S.setInitialization(retSam->x);
  // S.getProblem()->checkJacobian(S.x, 1e-6);
  // S.setSolver(NLPS_Ipopt);
  S.setSolver(rai::M_logBarrier);
  // S.setSolver(NLPS_augmentedLag);
  S.opt.set_muMax(1e6) .set_stopEvals(5000) .set_muLBInit(1e-3) .set_muInit(1e3);
  auto ret = S.solve(-1, 2);
  // S.getProblem()->checkJacobian(ret->x, 1e-5);
  std::cout <<*ret <<std::endl;
  P->report(std::cout, 10);

}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  // rnd.clockSeed();
  rnd.seed(0);

  // testDisplay();
  // testSolver();

  testSpherePacking();
  return 0;
}
