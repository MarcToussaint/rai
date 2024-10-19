#include <Optim/benchmarks.h>
#include <Optim/constrained.h>
#include <Optim/utils.h>

#include <iomanip>

//this code is only for demo in the lecture -- a bit messy!

//==============================================================================
//
// test standard constrained optimizers
//

void lectureDemo(const shared_ptr<NLP>& P, const arr& x_start, uint iters){
  rai::OptOptions options;
  LagrangianProblem lag(P, options);

  //-- initial x
  arr x = P->getInitializationSample();
  if(!!x_start) x=x_start;

  //  cout <<std::setprecision(2);
  cout <<"x0=" <<x <<endl;

  rai::system("rm -f z.opt_all");

  uint evals=0;
  for(uint k=0;k<iters;k++){
    P->checkJacobian(x, 1e-4);
//    checkGradient(lag, x, 1e-4);
//    checkHessian (lag, x, 1e-4); //will throw errors: no Hessians for g!

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
    newton.bounds = lag.bounds;
    ofstream fil("z.opt");
    newton.simpleLog = &fil;
    newton.reinit(x);
    newton.run();
    evals += newton.evals;

    rai::system("cat z.opt >> z.opt_all");
    if(x.N==2){
      gnuplot("load 'plt'", false, true);
      rai::wait();
    }

    //upate unconstraint problem parameters
    lag.autoUpdate(options, &newton.fx, newton.gx, newton.Hx);

    cout <<k <<' ' <<evals <<" f(x)=" <<lag.get_costs()
         <<" \tg_compl=" <<lag.get_sumOfGviolations()
         <<" \th_compl=" <<lag.get_sumOfHviolations()
      <<" \tmu=" <<lag.mu <<" \tmuLB=" <<lag.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<lag.lambda /*<<" \tg=" <<UCP.g_x <<" \th=" <<UCP.h_x*/;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<lag.get_costs() <<"\nx_opt=" <<x <<"\nlambda=" <<lag.lambda <<endl;

  rai::system("mv z.opt_all z.opt");
  if(x.N==2) gnuplot("load 'plt'", false, true);

//  if(!!x_start) x_start = x;
}
