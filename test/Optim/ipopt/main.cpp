#include <Optim/opt-ipopt.h>

#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(Ipopt){
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();
  auto traced = make_shared<MP_Traced>(mp);
  arr x_init = mp->getInitializationSample();

  {
    IpoptInterface ipo(traced);
    ipo.solve(x_init);
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

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);
  rnd.clockSeed();

  testIpopt();

  return 0;
}
