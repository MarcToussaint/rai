#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/MP_Solver.h>
#include <Optim/lagrangian.h>

//===========================================================================

void displayMathematicalProgram(std::shared_ptr<MathematicalProgram> mp, const arr& trace_x={}, const arr& trace_cost={}){
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

  rai::String cmd;
  cmd <<"reset; set contour; set cntrparam linear; set cntrparam levels incremental 0,.1,10; set xlabel 'x'; set ylabel 'y';";
  if(!trace_x.N){
    cmd <<"splot [-1:1][-1:1] 'z.fct' matrix us ($2/50-1):($1/50-1):3 w l;";
  }else{
    if(trace_cost.N){
      catCol(trace_x, trace_cost.col(0)).writeRaw(FILE("z.trace"));
      cmd <<"splot [-1:1][-1:1] 'z.fct' matrix us ($2/50-1):($1/50-1):3 w l, 'z.trace' us 1:2:3 w lp;";
    }else{
      trace_x.writeRaw(FILE("z.trace"));
      cmd <<"unset surface; set table 'z.table';";
      cmd <<"splot [-1:1][-1:1] 'z.fct' matrix us ($2/50-1):($1/50-1):3 w l;";
      cmd <<"unset table;";
      cmd <<"plot 'z.table' w l, 'z.trace' us 1:2 w lp lw 2;";
    }
  }
  gnuplot(cmd, false, false);
}

void TEST(Display) {
  std::shared_ptr<MathematicalProgram> mp = getBenchmarkFromCfg();

  displayMathematicalProgram(mp);

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

  displayMathematicalProgram(mp, S.getTrace_x());
  // displayMathematicalProgram(mp, S.getTrace_x(), S.getTrace_costs());
//  gnuplot("load 'plt'", false, false);
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testDisplay();
  testSolver();

  return 0;
}
