#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/solver.h>

void TEST(SqrProblem) {
  const ScalarFunction& _f = ChoiceFunction();

  uint dim=rai::getParameter<double>("dim");

  Conv_ScalarProblem_MathematicalProgram _nlp(_f, dim);
  _nlp.setBounds(-2., 2.);
  MathematicalProgram_Traced nlp(_nlp);
  Conv_MathematicalProgram_ScalarProblem f(nlp);

  displayFunction(f, true);

  arr x(dim),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  checkGradient(f, x, 1e-3);
  checkHessian (f, x, 1e-3);

  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp);
  S.setInitialization({1., 1.});
  S.solve();

  arr path = catCol(S.getTrace_x(), S.getTrace_costs());
  path.writeRaw(FILE("z.path"));
  gnuplot("load 'plt'", false, true);
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testSqrProblem();

  return 0;
}
