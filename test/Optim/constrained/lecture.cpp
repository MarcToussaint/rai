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

void testConstraint(MathematicalProgram& p, uint dim_x, arr& x_start=NoArr, uint iters=20){

  OptOptions options;
  LagrangianProblem lag(p, options);

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

  system("rm -f z.opt_all");

  uint evals=0;
  for(uint k=0;k<iters;k++){
    checkJacobianCP(p, x, 1e-4);
    checkGradient(lag, x, 1e-4);
    checkHessian (lag, x, 1e-4); //will throw errors: no Hessians for g!

    lag.lagrangian(NoArr, NoArr, x);

    if(x.N==2){
      displayFunction(lag);
      rai::wait();
      gnuplot("load 'plt'", false, true);
      rai::wait();
    }

//    optRprop(x, UCP, options);
//    optGrad(x, lag, options);
//    system("cat z.opt >> z.opt_all");

    OptNewton newton(x, lag, options);
    lag.getBounds(newton.bounds_lo, newton.bounds_up);
    newton.reinit(x);
    ofstream fil("z.opt");
    newton.simpleLog = &fil;
    newton.run();
    evals += newton.evals;

    system("cat z.opt >> z.opt_all");
    if(x.N==2){
      gnuplot("load 'plt'", false, true);
      rai::wait();
    }

    //upate unconstraint problem parameters
    lag.autoUpdate(options, &newton.fx, newton.gx, newton.Hx);
//    switch(options.constrainedMethod){
//    case squaredPenalty: lag.mu *= 2.;  lag.nu *= 2.;  break;
//    case augmentedLag:   lag.aulaUpdate(false, 1., muInc, &newton.fx, newton.gx, newton.Hx);  break;
//    case anyTimeAula:    lag.aulaUpdate(true,  1., muInc, &newton.fx, newton.gx, newton.Hx);  break;
//    case logBarrier:     lag.muLB /= 2.;  lag.nu *= 10;  break;
//    default: NIY;
//    }

    cout <<k <<' ' <<evals <<" f(x)=" <<lag.get_costs()
         <<" \tg_compl=" <<lag.get_sumOfGviolations()
         <<" \th_compl=" <<lag.get_sumOfHviolations()
      <<" \tmu=" <<lag.mu <<" \tnu=" <<lag.nu <<" \tmuLB=" <<lag.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<lag.lambda /*<<" \tg=" <<UCP.g_x <<" \th=" <<UCP.h_x*/;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<lag.get_costs() <<"\nx_opt=" <<x <<"\nlambda=" <<lag.lambda <<endl;

  system("mv z.opt_all z.opt");
  if(x.N==2) gnuplot("load 'plt'", false, true);

  if(!!x_start) x_start = x;
}
