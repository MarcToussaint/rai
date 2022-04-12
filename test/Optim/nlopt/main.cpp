#include <Optim/opt-nlopt.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(NLOpt){
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();
  auto traced = make_shared<NLP_Traced>(nlp);
  arr x_init = nlp->getInitializationSample();

  {
    NLoptInterface nlo(traced);
    nlo.solve(x_init);
  }

  NLP_Viewer(nlp, traced).display();
  rai::wait();
  NLP_Viewer(nlp, traced).plotCostTrace();
  rai::wait();

  //---

  traced->clear();
  {
    arr x = x_init;
    OptConstrained opt(x, NoArr, traced);
    opt.run();
  }

  NLP_Viewer(nlp, traced).display();
  rai::wait();
  NLP_Viewer(nlp, traced).plotCostTrace();
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testNLOpt();

  return 0;
}
