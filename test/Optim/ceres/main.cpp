#ifndef RAI_CERES
#error *** this only compiles with the ceres dependency ***
#else

#include <Optim/opt-ceres.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

//#include <Kin/kin.h>
//#include <Gui/opengl.h>
#include <KOMO/komo.h>
//#include <Kin/TM_default.h>

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
//  std::shared_ptr<MathematicalProgram> MP;
//  uint feature_id;
//  uint featureDim;
//  intA varIds;
//  uintA varDims;
//  uint varTotalDim;

//public:
//  Conv_CostFunction(const ptr<MathematicalProgram>& _MP,
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
//  std::shared_ptr<MathematicalProgram> MP;

//  uintA variableDimensions, featureDimensions, variableDimIntegral;
//  intAA featureVariables;
//  ObjectiveTypeA featureTypes;

//  arr x_base;
//  arr bounds_lo, bounds_up;
//  arrA x;
//  arrA phi;
//  arrA J;

//  ceres::Problem cs;

//  Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram>& _MP) : MP(_MP) {

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
  komo.solver = rai::KS_sparse; //sparseOptimization=true;
  komo.setModel(C, false);
  komo.setTiming(1, 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e0);
//  komo.addSquaredQuaternionNorms(-1., -1., 1e1); //when the kinematics includes quaternion joints, keep them roughly regularized

  komo.addObjective({1.,-1.}, FS_positionDiff, {"endeff", "target"}, OT_sos, {1e1});

  komo.addObjective({1., -1.}, FS_quaternionDiff, {"endeff", "target"}, OT_sos, {1e1});

//  komo.setSlow(1., -1., 1e1);

  komo.run_prepare(.01);

#if 1
  KOMO::Conv_KOMO_FactoredNLP P(komo);
//  auto P1 = make_shared<KOMO::Conv_KOMO_SparseUnstructured>(komo, false);
//  auto P = make_shared<Conv_MathematicalProgram_TrivialStructured>(*P1);

  checkJacobianCP(P, komo.x, 1e-4);

  Conv_MathematicalProgram_CeresProblem cer(P);
  cer.x_full = P.getInitializationSample();

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

  komo.set_x2(cer.x_full);
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
  return;
#endif

  //-- call the optimizer
  komo.optimize();
  komo.getReport(true); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
}

//===========================================================================

void testCeres2(){
  MP_TrivialSquareFunction P(20, 1., 2.);
//  auto P = make_shared<ChoiceConstraintFunction>();

  Conv_MathematicalProgram_TrivialFactoreded P2(P);
  Conv_MathematicalProgram_CeresProblem cer(P2);

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
//  MP_TrivialSquareFunction P(2, 1., 2.);
  ChoiceConstraintFunction P;

  {
    MathematicalProgram_Traced P2(P);
    LagrangianProblem L(P2);
    Conv_MathematicalProgram_TrivialFactoreded P3(L);

    CeresInterface opt(P3);
    opt.solve();
    ofstream fil2("z.opt2");
    P2.xTrace.writeRaw(fil2);
  }

  arr x, phi;
  x = P.getInitializationSample();

  checkJacobianCP(P, x, 1e-4);

  OptConstrained opt(x, NoArr, P);
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
