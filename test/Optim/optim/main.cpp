#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/MP_Solver.h>
#include <Optim/lagrangian.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();

  MP_Viewer(mp).display();

  rai::wait();
}

//===========================================================================

void TEST(Solver) {
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();

//  displayMathematicalProgram(mp);

  arr x = mp->getInitializationSample();
  checkJacobianCP(*mp, x, 1e-4);

  MP_Solver S;

  rai::Enum<MP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setVerbose(rai::getParameter<int>("opt/verbose"));
  S.setSolver(sid);
  S.setProblem(mp);
//  S.setInitialization(ones(x.N)); //{2., 0.});
  S.solve();

  arr path = catCol(S.getTrace_x(), S.getTrace_costs());
  path.writeRaw(FILE("z.path"));

  MP_Viewer(mp, S.P). display();
  // displayMathematicalProgram(mp, S.getTrace_x(), S.getTrace_costs());
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
