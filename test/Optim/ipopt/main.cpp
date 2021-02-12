#include <Optim/opt-ipopt.h>

#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void TEST(Ipopt){
//  MP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  {
    IpoptInterface ipo(P);
    ipo.solve();
    //ofstream fil2("z.opt2");
    //ipo.P.xLog.writeRaw(fil2);
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

int main(int argn, char** argv){
  rai::initCmdLine(argn, argv);

  testIpopt();

  return 0;
}
