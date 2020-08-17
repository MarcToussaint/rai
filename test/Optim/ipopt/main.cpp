
#include "IpoptConstrainedProblem.h"
#include "prob.h"
#include <coin/IpIpoptApplication.hpp>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//===========================================================================

void orig(){
  // Create a new instance of your nlp 
  //  (use a SmartPtr, not raw)
  SmartPtr<TNLP> mynlp = new MyNLP();

  // Create a new instance of IpoptApplication
  //  (use a SmartPtr, not raw)
  // We are using the factory, since this allows us to compile this
  // example with an Ipopt Windows DLL
  SmartPtr<IpoptApplication> app = new IpoptApplication(); //IpoptApplicationFactory();

  // Change some options
  // Note: The following choices are only examples, they might not be
  //       suitable for your optimization problem.
  app->Options()->SetNumericValue("tol", 1e-9);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");

  // Intialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    printf("\n\n*** Error during initialization!\n");
    return;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);

  if (status == Solve_Succeeded) {
    printf("\n\n*** The problem solved!\n");
  }
  else {
    printf("\n\n*** The problem FAILED!\n");
  }

  // As the SmartPtrs go out of scope, the reference count
  // will be decremented and the objects will automatically 
  // be deleted.
}

//===========================================================================

void TEST(Ipopt){
//  MP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  {
    IpoptInterface ipo(P);
    ipo.solve();
    ofstream fil2("z.opt2");
    ipo.P.xLog.writeRaw(fil2);
  }

  arr x, phi;
  x = P.getInitializationSample();

  checkJacobianCP(P, x, 1e-4);

  OptConstrained opt(x, NoArr, P, 6);
  {
    P.getBounds(opt.newton.bound_lo, opt.newton.bound_up);
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

//  orig();

  testIpopt();

  return 0;
}
