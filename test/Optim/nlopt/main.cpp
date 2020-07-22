#include <Optim/opt-nlopt.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(NLOpt){
//  MP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  NLOptInterface nlo(P);
  nlo.solve();

  arr x, phi;
  x = P.getInitializationSample();

  P.evaluate(phi, NoArr, x);
  cout <<x <<endl <<phi;

  checkJacobianCP(P, x, 1e-4);

  OptConstrained opt(x, NoArr, P, 6);
  P.getBounds(opt.newton.bound_lo, opt.newton.bound_up);
  opt.run();

  cout <<"optimum: " <<x <<endl;
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  testSqrProblem();

//  ChoiceConstraintFunction F;
  testNLOpt();

  return 0;
}
