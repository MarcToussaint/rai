#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/MP_Solver.h>
#include <Optim/lagrangian.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();

  uint d = mp->getDimension();
  CHECK_EQ(d, 2, "can only display 2D problems for now");

  //-- get bounds
  arr lo, up;
  mp->getBounds(lo, up);
  if(!lo.N) lo = -ones(2);
  if(!up.N) up = ones(2);

  //-- make grid
  arr X, Y, phi;
  X.setGrid(2, 0., 1., 100);
  for(uint i=0;i<X.d0;i++){ X[i] = lo + (up-lo)%X[i]; }
  Y.resize(X.d0);

  //-- transform constrained problem to AugLag scalar function
  mp->evaluate(phi, NoArr, X[0]);
  std::shared_ptr<LagrangianProblem> lag;
  std::shared_ptr<MathematicalProgram> mp_save;
  if(phi.N>1){
    lag = make_shared<LagrangianProblem>(mp);
    lag->mu = lag->nu = 1e3;
    mp_save = mp;
    mp.reset();
    mp = make_shared<Conv_ScalarProblem_MathematicalProgram>(*lag, d);
  }

  //-- evaluate over the grid
  for(uint i=0; i<X.d0; i++) {
    mp->evaluate(phi, NoArr, X[i]);
    CHECK_EQ(phi.N, 1, "only 1 feature for now");
    double fx=phi.scalar();
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101, 101);

  //-- plot
  //  plot()->Gnuplot();  plot()->Surface(Y);  plot()->update(true);
  write(LIST<arr>(Y), "z.fct");
  gnuplot("reset; set contour; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", false, false);

  rai::wait();
}

//===========================================================================

void TEST(SqrProblem) {
  const ScalarFunction& _f = ChoiceFunction();

  uint dim=rai::getParameter<double>("dim");

  auto _nlp = make_shared<Conv_ScalarProblem_MathematicalProgram>(_f, dim);
  _nlp->setBounds(-2., 2.);
  auto nlp = make_shared<MathematicalProgram_Traced>(_nlp);
  Conv_MathematicalProgram_ScalarProblem f(nlp);

  displayFunction(f, true);

  arr x(dim),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  checkGradient(f, x, 1e-3);
  checkHessian (f, x, 1e-3);

  MP_Solver S;

  rai::Enum<MP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp);
  S.setInitialization({2., 0.});
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

  testDisplay();
//  testSqrProblem();

  return 0;
}
