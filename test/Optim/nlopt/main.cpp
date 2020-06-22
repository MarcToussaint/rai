#include <Optim/opt-nlopt.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(SqrProblem) {
  const ScalarFunction& f = ChoiceFunction();

  displayFunction(f, true);

  Conv_ScalarProblem_MathematicalProgram P(f, 10);
  NLOptInterface nlo(P);
  nlo.solve();

  arr x(10),x0;
  rndUniform(x,1.,1.,false);
  x0=x;

  checkGradient(f, x, 1e-3);
  checkHessian (f, x, 1e-3);

  optRprop(x, f, OPT(initStep=.01, stopTolerance=1e-5, stopEvals=1000, verbose=2));
  system("cp z.opt z.rprop");

  x=x0;
  optGrad(x, f, OPT(stopEvals=10000));
  system("cp z.opt z.grad");

  x=x0;
  optNewton(x, f, OPT(stopEvals=1000, initStep=1., stopTolerance=1e-5, verbose=2, damping=.1));
  system("cp z.opt z.newton");

  gnuplot("set log y; plot 'z.newton' us 1:3 w l,'z.grad' us 1:3 w l,'z.rprop' us 1:3 w l",false,true);
}


//===========================================================================


void TEST(MathematicalProgram){
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
  testMathematicalProgram();

  return 0;
}
