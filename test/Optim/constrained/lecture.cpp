#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

#include <iomanip>

//this code is only for demo in the lecture -- a bit messy!

//void displayFunction(const ScalarFunction& f){
//  arr phi;
//  arr X, Y;
//  X.setGrid(2,-1.2,1.2,100);
//  Y.resize(X.d0);
//  for(uint i=0;i<X.d0;i++) Y(i) = f(NoArr, NoArr, X[i]);
//  Y.reshape(101,101);
//  write(LIST<arr>(Y),"z.fct");
//  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us (1.2*($1/50-1)):(1.2*($2/50-1)):3 w l", false, true);
//}

//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint(ConstrainedProblem& p, uint dim_x, arr& x_start=NoArr, uint iters=20){

  OptOptions options;
  LagrangianProblem UCP(p, options);

  //-- choose constrained method
  switch(options.constrainedMethod){
  case squaredPenalty: UCP.mu=10.; UCP.nu=10.;  break;
  case augmentedLag:   UCP.mu=1.;  UCP.nu=1.;   break;
  case logBarrier:     UCP.muLB=1.;  UCP.nu=1.;   break;
  default: NIY;
  }

  double muInc = rai::getParameter<double>("opt/aulaMuInc", 1.5);

  //-- initial x
  arr x(dim_x);
  if(!!x_start) x=x_start;
  else{
    x.setZero();
    if(options.constrainedMethod==logBarrier){ } //log barrier needs a feasible starting point
    else rndUniform(x, -1., 1.);
  }
  //  cout <<std::setprecision(2);
  cout <<"x0=" <<x <<endl;

  rnd.seed(0);

  system("rm -f z.grad_all");

  uint evals=0;
  for(uint k=0;k<iters;k++){
    checkJacobianCP(p, x, 1e-4);
    checkGradient(UCP, x, 1e-4);
    checkHessian (UCP, x, 1e-4); //will throw errors: no Hessians for g!

    UCP.lagrangian(NoArr, NoArr, x);

    if(x.N==2){
      displayFunction(UCP);
      rai::wait();
      gnuplot("load 'plt'", false, true);
      rai::wait();
    }

    //optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    OptNewton newton(x, UCP, OPT(verbose=2, damping=.1, stopTolerance=1e-2));
    newton.run();
    evals+=newton.evals;

    //upate unconstraint problem parameters
    switch(newton.o.constrainedMethod){
    case squaredPenalty: UCP.mu *= 2.;  UCP.nu *= 2.;  break;
    case augmentedLag:   UCP.aulaUpdate(false, 1., muInc, &newton.fx, newton.gx, newton.Hx);  break;
    case anyTimeAula:    UCP.aulaUpdate(true,  1., muInc, &newton.fx, newton.gx, newton.Hx);  break;
    case logBarrier:     UCP.muLB /= 2.;  UCP.nu *= 10;  break;
    default: NIY;
    }

    system("cat z.grad >>z.grad_all");
    cout <<k <<' ' <<evals <<" f(x)=" <<UCP.get_costs()
	 <<" \tg_compl=" <<UCP.get_sumOfGviolations()
	 <<" \th_compl=" <<UCP.get_sumOfHviolations()
      <<" \tmu=" <<UCP.mu <<" \tnu=" <<UCP.nu <<" \tmuLB=" <<UCP.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda /*<<" \tg=" <<UCP.g_x <<" \th=" <<UCP.h_x*/;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<UCP.get_costs() <<"\nx_opt=" <<x <<"\nlambda=" <<UCP.lambda <<endl;

  system("mv z.grad_all z.grad");
  if(x.N==2) gnuplot("load 'plt'", false, true);

  if(!!x_start) x_start = x;
}
