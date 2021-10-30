#include <Optim/opt-nlopt.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(NLOpt){
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();
  auto traced = make_shared<MP_Traced>(mp);
  arr x_init = mp->getInitializationSample();

  {
    NLoptInterface nlo(traced);
    nlo.solve(x_init);
  }

  MP_Viewer(mp, traced).display();
  rai::wait();
  MP_Viewer(mp, traced).plotCostTrace();
  rai::wait();

  //---

  traced->clear();
  {
    arr x = x_init;
    OptConstrained opt(x, NoArr, traced);
    opt.run();
  }

  MP_Viewer(mp, traced).display();
  rai::wait();
  MP_Viewer(mp, traced).plotCostTrace();
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testNLOpt();

  return 0;
}
