#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/TM_default.h>

#include <Optim/Graph_Problem.h>
#include <Optim/newOptim.h>
#include <Optim/benchmarks.h>

#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_LE
#undef CHECK_GE
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>

class Conv_CostFunction : public ceres::CostFunction {
  std::shared_ptr<MathematicalProgram> MP;
  uint feature_id;
  uint featureDim;
  intA varIds;
  uintA varDims;
  uint varTotalDim;

public:
  Conv_CostFunction(const ptr<MathematicalProgram>& _MP,
                    uint _feature_id,
                    const uintA& variableDimensions,
                    const uintA& featureDimensions,
                    const intAA& featureVariables)
    : MP(_MP),
      feature_id(_feature_id) {
    featureDim = featureDimensions(feature_id);
    varIds = featureVariables(feature_id);
    varDims.resize(varIds.N);
    for(uint i=0;i<varIds.N;i++){
      varDims(i) = variableDimensions(varIds(i));
    }
    varTotalDim = sum(varDims);
    //ceres internal:
    mutable_parameter_block_sizes()->resize(varDims.N);
    for(uint i=0;i<varDims.N;i++) (*mutable_parameter_block_sizes())[i] = varDims(i);
    set_num_residuals(featureDim);
  }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const{
    //set variables individually
    {
      arr x;
      for(uint i=0;i<varIds.N;i++){
        x.referTo(parameters[i], varDims(i));
        MP->setSingleVariable(varIds(i), x);
      }
    }
    {
      arr phi, J;
      phi.referTo(residuals, featureDim);
      if(jacobians){
        J.referTo(jacobians[0], featureDim*varTotalDim);
        MP->evaluateSingleFeature(feature_id, phi, J, NoArr);
      }else{
        MP->evaluateSingleFeature(feature_id, phi, NoArr, NoArr);
      }
    }
    return true;
  }
};


//===========================================================================

struct Conv_MatematicalProgram_CeresProblem {
  std::shared_ptr<MathematicalProgram> MP;

  uintA variableDimensions, featureDimensions, variableDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;

  arr x_base;
  arr bounds_lo, bounds_up;
  arrA x;
  arrA phi;
  arrA J;

  ceres::Problem cs;

  Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram>& _MP) : MP(_MP) {

    //you must never ever resize these arrays, as ceres takes pointers directly into these fixed memory buffers!
    x_base.resize(MP->getDimension());
    MP->getBounds(bounds_lo, bounds_up);
    MP->getFeatureTypes(featureTypes);
    MP->getStructure(variableDimensions, featureDimensions, featureVariables);
    variableDimIntegral = integral(variableDimensions);

    x.resize(variableDimensions.N);
    uint n=0;
    for(uint i=0;i<x.N;i++){
      uint d = variableDimensions(i);
      x(i).referTo(x_base.p+n, d);
      n += d;
    }
    //CHECK_EQ(n, x_base.N, "");

    phi.resize(featureDimensions.N);
    for(uint i=0;i<phi.N;i++) phi(i).resize(featureDimensions(i));

    J.resize(phi.N);
    for(uint i=0;i<J.N;i++){
      uint n=0;
      for(uint j : featureVariables(i)) n += variableDimensions(j);
      J(i).resize(n);
    }

    for(arr& xi: x){
      cs.AddParameterBlock(xi.p, xi.N);
      for(uint i=0;i<xi.N;i++){
        uint i_all = i + xi.p-x_base.p;
        cs.SetParameterLowerBound(xi.p, i, bounds_lo(i_all));
        cs.SetParameterUpperBound(xi.p, i, bounds_up(i_all));
      }
    }

    for(uint i=0;i<phi.N;i++){
      rai::Array<double*> parameter_blocks(featureVariables(i).N);
      for(uint k=0;k<featureVariables(i).N;k++){
        parameter_blocks(k) = x(featureVariables(i)(k)).p;
      }
      auto fct = new Conv_CostFunction(MP, i, variableDimensions, featureDimensions, featureVariables);
      cs.AddResidualBlock(fct, nullptr, parameter_blocks);
    }
  }
};

//===========================================================================

void tutorialBasics(){
  rai::Configuration C("model.g");

  KOMO komo;
  komo.sparseOptimization=true;
  komo.setModel(C, false);
  komo.setTiming(2, 20, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addSquaredQuaternionNorms(-1., -1., 1e1); //when the kinematics includes quaternion joints, keep them roughly regularized

  komo.addObjective({1.,-1.}, FS_positionDiff, {"endeff", "target"}, OT_eq, {1e0});

  komo.addObjective({1., -1.}, FS_quaternionDiff, {"endeff", "target"}, OT_eq, {1e1});

  komo.setSlow(1., -1., 1e1);

  komo.setupRepresentations();

  //-- call the optimizer
  komo.optimize();
  komo.getReport(true); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory
}

//===========================================================================

void testCeres(){
  auto P = make_shared<MP_TrivialSquareFunction>(2, 1., 2.);

  Conv_MatematicalProgram_CeresProblem cer(P);

  cer.x_base = P->getInitializationSample();

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &cer.cs, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "solution : " <<cer.x_base << "\n";
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  tutorialBasics();
  testCeres();


  return 0;
}

//===========================================================================
//OLD:

struct Conv_GraphProblem_CeresProblem {
  GraphProblem& G;
  uintA variableDimensions, varDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;

  arr x;
  arr phi;
  arrA J;

  ceres::Problem cs;

  Conv_GraphProblem_CeresProblem(GraphProblem& _G) : G(_G) {
    G.getStructure(variableDimensions, featureVariables, featureTypes);
    varDimIntegral = integral(variableDimensions);

    //you must never ever resize these arrays, as ceres takes pointers directly into these fixed memory buffers!
    x.resize(sum(variableDimensions));
    phi.resize(featureTypes.N);
    J.resize(phi.N);
    for(uint i=0;i<phi.N;i++){
      uint n_Ji=0;
      for(int v:featureVariables(i)) n_Ji += variableDimensions(v);
      J(i).resize(n_Ji);
    }

    for(uint v=0;v<variableDimensions.N;v++){
      cs.AddParameterBlock(&x(varDimIntegral(v)), variableDimensions(v));
    }

    for(uint i=0;i<phi.N;i++){
      rai::Array<double*> parameter_blocks(featureVariables(i).N);
      for(uint k=0;k<featureVariables(i).N;k++){
        int v = featureVariables(i)(k);
        parameter_blocks(k) = &x(varDimIntegral(v));
      }
//      cs.AddResidualBlock(&phi(i), nullptr, parameter_blocks);
    }
  }

  Conv_GraphProblem_CeresProblem(GraphProblem& _G, ostream *_log=0);
};
