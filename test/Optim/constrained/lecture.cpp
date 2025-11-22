#include <Optim/testProblems_Opt.h>
#include <Optim/constrained.h>
#include <Optim/utils.h>

#include <iomanip>

//this code is only for demo in the lecture -- a bit messy!

//==============================================================================
//
// test standard constrained optimizers
//

void lectureDemo(const shared_ptr<NLP>& P, const arr& x_start, uint iters){
  auto opt = make_shared<rai::OptOptions>();
  auto traced = make_shared<NLP_Traced>(P);
  auto lag = make_shared<rai::LagrangianProblem>(traced, opt);

  { //initial display
    auto lag = make_shared<rai::LagrangianProblem>(P, opt);
    lag->useLB = false;
    lag->mu = 1e5;
    displayFunction(lag->f_scalar());
    rai::wait();
  }
  { //initial display
    auto lag = make_shared<rai::LagrangianProblem>(P, opt);
    displayFunction(lag->f_scalar());
    rai::wait();
  }

  //-- initial x
  arr x = P->getInitializationSample();
  if(!!x_start) x=x_start;

  //  cout <<std::setprecision(2);
  cout <<"x0:" <<x <<endl;

  rai::system("rm -f z.opt_all");

  uint evals=0;
  arr err;
  for(uint k=0;k<iters;k++){
    P->checkJacobian(x, 1e-4);
//    checkGradient(lag, x, 1e-4);
//    checkHessian (lag, x, 1e-4); //will throw errors: no Hessians for g!

    lag->eval_scalar(NoArr, NoArr, x);

//    optRprop(x, UCP, options);
//    optGrad(x, lag, options);
//    system("cat z.opt >> z.opt_all");

    rai::OptNewton newton(x, lag->f_scalar(), opt);
    newton.bounds = lag->bounds;
    newton.reinit(x);
    newton.run();
    evals += newton.evals;

    traced->setTracing(false, false, false, false);
    NLP_Viewer(lag, traced).display();
    traced->setTracing(true, true, false, false);
    rai::wait();
    // rai::system("cat z.opt >> z.opt_all");
    // if(x.N==2){
    //   gnuplot("load 'plt'", false, true);
    //   rai::wait();
    // }

    //upate unconstraint problem parameters
    lag->autoUpdate(&newton.fx, newton.gx, newton.Hx);

    err = P->summarizeErrors(lag->phi_x);
    cout <<k <<' ' <<evals <<" f:" <<err(OT_f)+err(OT_sos)
         <<" \tg:" <<err(OT_ineq)
         <<" \th:" <<err(OT_eq)
      <<" \tmu:" <<lag->mu <<" \tmuLB:" <<lag->muLB;
    if(x.N<5) cout <<" \tx:" <<x <<" \tlambda:" <<lag->lambda /*<<" \tg:" <<UCP.g_x <<" \th:" <<UCP.h_x*/;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x):" <<sum(err) <<"\nx_opt:" <<x <<"\nlambda:" <<lag->lambda <<endl;

  rai::system("mv z.opt_all z.opt");
  if(x.N==2) gnuplot("load 'plt'", true);

//  if(!!x_start) x_start = x;
}
