#include <Optim/opt-nlopt.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(NLOpt){
//  MP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  {
    NLoptInterface nlo(P);
    nlo.solve();
    ofstream fil2("z.opt2");
    //    nlo.P.xLog.writeRaw(fil2);
  }

  arr x, phi;
  x = P.getInitializationSample();

  checkJacobianCP(P, x, 1e-4);

  OptConstrained opt(x, NoArr, P, OptOptions().set_verbose(6));
  {
    P.getBounds(opt.newton.bounds_lo, opt.newton.bounds_up);
    ofstream fil("z.opt");
    opt.newton.simpleLog = &fil;
    opt.run();
  }

  if(x.N==2){
    displayFunction(opt.L);
    rai::wait();
    gnuplot("load 'plt'");
    rai::wait();
  }

  cout <<"optimum: " <<x <<endl;
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  testNLOpt();

  return 0;
}
