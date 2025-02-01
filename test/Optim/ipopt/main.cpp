#include <Optim/opt-ipopt.h>

#include <Optim/testProblems_Opt.h>.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(Ipopt){
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();
  auto traced = make_shared<NLP_Traced>(nlp);
  arr x_init = nlp->getInitializationSample();

  {
    IpoptInterface ipo(traced);
    ipo.solve(x_init);
    cout <<"#evals: " <<traced->evals <<endl;
  }

  NLP_Viewer(nlp, traced).display();
  rai::wait();
  NLP_Viewer(nlp, traced).plotCostTrace();
  rai::wait();

  //---

  traced->clear();
  {
    arr x = x_init;
    ConstrainedSolver opt(x, NoArr, traced);
    opt.run();
  }

  NLP_Viewer(nlp, traced).display();
  rai::wait();
  NLP_Viewer(nlp, traced).plotCostTrace();
  rai::wait();
}

//===========================================================================

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

  testIpopt();

  return 0;
}
