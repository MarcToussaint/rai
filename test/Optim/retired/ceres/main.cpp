#ifndef RAI_CERES
#error *** this only compiles with the ceres dependency ***
#else

#include <Optim/opt-ceres.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>
#include <Optim/utils.h>

//#include <Kin/kin.h>
//#include <Gui/opengl.h>
#include <KOMO/komo.h>

//#include <Optim/Graph_Problem.h>
//#include <Optim/newOptim.h>
//#include <Optim/benchmarks.h>

#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_LE
#undef CHECK_GE
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>

//class Conv_CostFunction : public ceres::CostFunction {
//  std::shared_ptr<NLP> MP;
//  uint feature_id;
//  uint featureDim;
//  intA varIds;
//  uintA varDims;
//  uint varTotalDim;

//public:
//  Conv_CostFunction(const shared_ptr<NLP>& _MP,
//                    uint _feature_id,
//                    const uintA& variableDimensions,
//                    const uintA& featureDimensions,
//                    const intAA& featureVariables)
//    : MP(_MP),
//      feature_id(_feature_id) {
//    featureDim = featureDimensions(feature_id);
//    varIds = featureVariables(feature_id);
//    varDims.resize(varIds.N);
//    for(uint i=0;i<varIds.N;i++){
//      varDims(i) = variableDimensions(varIds(i));
//    }
//    varTotalDim = sum(varDims);
//    //ceres internal:
//    mutable_parameter_block_sizes()->resize(varDims.N);
//    for(uint i=0;i<varDims.N;i++) (*mutable_parameter_block_sizes())[i] = varDims(i);
//    set_num_residuals(featureDim);
//  }

//  virtual bool Evaluate(double const* const* parameters,
//                        double* residuals,
//                        double** jacobians) const{
//    //set variables individually
//    {
//      arr x;
//      for(uint i=0;i<varIds.N;i++){
//        x.referTo(parameters[i], varDims(i));
//        MP.setSingleVariable(varIds(i), x);
//      }
//    }
//    {
//      arr phi, J;
//      phi.referTo(residuals, featureDim);
//      if(jacobians){
//        J.referTo(jacobians[0], featureDim*varTotalDim);
//        MP.evaluateSingleFeature(feature_id, phi, J, NoArr);
//      }else{
//        MP.evaluateSingleFeature(feature_id, phi, NoArr, NoArr);
//      }
//    }
//    return true;
//  }
//};

////===========================================================================

//struct Conv_MatematicalProgram_CeresProblem {
//  std::shared_ptr<NLP> MP;

//  uintA variableDimensions, featureDimensions, variableDimIntegral;
//  intAA featureVariables;
//  ObjectiveTypeA featureTypes;

//  arr x_base;
//  arr bounds_lo, bounds_up;
//  arrA x;
//  arrA phi;
//  arrA J;

//  ceres::Problem cs;

//  Conv_MatematicalProgram_CeresProblem(const shared_ptr<NLP>& _MP) : MP(_MP) {

//    //you must never ever resize these arrays, as ceres takes pointers directly into these fixed memory buffers!
//    x_base.resize(MP.getDimension());
//    MP.getBounds(bounds_lo, bounds_up);
//    MP.getFeatureTypes(featureTypes);
//    MP.getStructure(variableDimensions, featureDimensions, featureVariables);
//    variableDimIntegral = integral(variableDimensions);

//    x.resize(variableDimensions.N);
//    uint n=0;
//    for(uint i=0;i<x.N;i++){
//      uint d = variableDimensions(i);
//      x(i).referTo(x_base.p+n, d);
//      n += d;
//    }
//    //CHECK_EQ(n, x_base.N, "");

//    phi.resize(featureDimensions.N);
//    for(uint i=0;i<phi.N;i++) phi(i).resize(featureDimensions(i));

//    J.resize(phi.N);
//    for(uint i=0;i<J.N;i++){
//      uint n=0;
//      for(uint j : featureVariables(i)) n += variableDimensions(j);
//      J(i).resize(n);
//    }

//    for(arr& xi: x){
//      cs.AddParameterBlock(xi.p, xi.N);
//      for(uint i=0;i<xi.N;i++){
//        uint i_all = i + xi.p-x_base.p;
//        cs.SetParameterLowerBound(xi.p, i, bounds_lo(i_all));
//        cs.SetParameterUpperBound(xi.p, i, bounds_up(i_all));
//      }
//    }

//    for(uint i=0;i<phi.N;i++){
//      rai::Array<double*> parameter_blocks(featureVariables(i).N);
//      for(uint k=0;k<featureVariables(i).N;k++){
//        parameter_blocks(k) = x(featureVariables(i)(k)).p;
//      }
//      auto fct = new Conv_CostFunction(MP, i, variableDimensions, featureDimensions, featureVariables);
//      cs.AddResidualBlock(fct, nullptr, parameter_blocks);
//    }
//  }
//};

//===========================================================================

void tutorialBasics(){
  rai::Configuration C("model.g");

  KOMO komo;
  komo.setConfig(C, false);
  komo.setTiming(1, 1, 5., 1);
  komo.addControlObjective({}, 1, 1e0);
//  komo.addQuaternionNorms(-1., -1., 1e1); //when the kinematics includes quaternion joints, keep them roughly regularized

  komo.addObjective({1.,-1.}, FS_positionDiff, {"endeff", "target"}, OT_sos, {1e1});

  komo.addObjective({1., -1.}, FS_quaternionDiff, {"endeff", "target"}, OT_sos, {1e1});

//  komo.setSlow(1., -1., 1e1);

  komo.run_prepare(.01);

#if 1
//  komo.solver=rai::KS_dense;
  auto P1 = komo.nlp();
  auto P = make_shared<Conv_NLP_TrivialFactoreded>(P1);

  P->checkJacobian(komo.x, 1e-4);

  Conv_NLP_CeresProblem cer(P);
  cer.x_full = P->getInitializationSample();

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; //DENSE_QR;
  options.use_nonmonotonic_steps = false;
  options.initial_trust_region_radius = 1e-2;
  options.max_trust_region_radius = 1e-0;
//      options.minimizer_type = ceres::LINE_SEARCH;
//      options.line_search_direction_type = ceres::STEEPEST_DESCENT;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options,  cer.ceresProblem.get(), &summary);

  // Run the solver!
//  ceres::Solver::Options options;
//  options.linear_solver_type = ceres::DENSE_QR;
//  options.minimizer_progress_to_stdout = true;
//  ceres::Solver::Summary summary;
//  ceres::Solve(options, cer.ceresProblem.get(), &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "solution : " <<cer.x_full << "\n";

  komo.set_x(cer.x_full);
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
  return;
#endif

  //-- call the optimizer
  komo.optimize();
  komo.report(false, false, true); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
}

//===========================================================================

void testCeres2(){
  NLP_TrivialSquareFunction P(20, 1., 2.);
//  auto P = make_shared<ChoiceConstraintFunction>();

  auto P2 = make_shared<Conv_NLP_TrivialFactoreded>(P.ptr());
  Conv_NLP_CeresProblem cer(P2);

  cer.x_full = P.getInitializationSample();

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, cer.ceresProblem.get(), &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "solution : " <<cer.x_full << "\n";
}

//===========================================================================

void TEST(Ceres){
//  NLP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  {
    NLP_Traced P2(P.ptr());
    LagrangianProblem L(P2.ptr());
    auto P3 = make_shared<Conv_NLP_TrivialFactoreded>(L.ptr());

    CeresInterface opt(P3);
    opt.solve();
    ofstream fil2("z.opt2");
    fil2 <<P2.xTrace.modRaw();
  }

  arr x, phi;
  x = P.getInitializationSample();

  P.checkJacobian(x, 1e-4);

  OptConstrained opt(x, NoArr, P.ptr());
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

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  tutorialBasics();
//  testCeres2();

  return 0;
}

#endif
