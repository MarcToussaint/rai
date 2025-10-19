/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "i_Ceres.h"

#ifdef RAI_CERES

#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_LE
#undef CHECK_GE
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>

#include <assert.h>

//===========================================================================

arr CeresInterface::solve() {
  Conv_NLP2CeresProblem cer(P);
  cer.x_full = P->getInitializationSample();

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; //DENSE_QR;
//  options.initial_trust_region_radius = 1e-0;
//  options.max_trust_region_radius = 1e-0;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = true;
  options.parameter_tolerance = 1e-4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, cer.ceresProblem.get(), &summary);

  std::cout << summary.FullReport() << "\n";
  std::cout << "solution : " <<cer.x_full << "\n";

  return cer.x_full;
}

//===========================================================================

class Conv_Feature2CostFunction : public ceres::CostFunction {
  Conv_NLP2CeresProblem& P;
  uint feature_id;
  uint featureDim;
  uintA varIds;
  uintA varDims;
  uint varTotalDim;

 public:
  Conv_Feature2CostFunction(Conv_NLP2CeresProblem& _P,
                            uint _feature_id,
                            const uintA& variableDimensions,
                            const uintA& featureDimensions,
                            const uintAA& featureVariables);

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const;
};

Conv_Feature2CostFunction::Conv_Feature2CostFunction(Conv_NLP2CeresProblem& _P, uint _feature_id, const uintA& variableDimensions, const uintA& featureDimensions, const uintAA& featureVariables)
  : P(_P), feature_id(_feature_id) {
  featureDim = featureDimensions(feature_id);
  varIds = featureVariables(feature_id);
  varDims.resize(varIds.N);
  for(uint i=0; i<varIds.N; i++) {
    varDims(i) = variableDimensions(varIds(i));
  }
  varTotalDim = 0;
  mutable_parameter_block_sizes()->clear();
  for(uint i=0; i<varDims.N; i++) if(varIds(i)>=0) {
      mutable_parameter_block_sizes()->push_back(varDims(i));
      varTotalDim += varDims(i);
    }
  set_num_residuals(featureDim);
}

bool Conv_Feature2CostFunction::Evaluate(const double* const* parameters, double* residuals, double** jacobians) const {
  //set variables individually
  {
    arr x;
    uint parameters_count=0;
    for(uint i=0; i<varIds.N; i++) if(varIds(i)>=0) {
        x.referTo(parameters[parameters_count++], varDims(i));
        P.P->setSingleVariable(varIds(i), x);
      }
  }
  {
    arr phi, J;
    phi.referTo(residuals, featureDim);
    if(jacobians) {
      J.referTo(jacobians[0], featureDim*varTotalDim);
      J.reshape(featureDim, varTotalDim);
      P.P->evaluateSingleFeature(feature_id, phi, J, NoArr);
    } else {
      P.P->evaluateSingleFeature(feature_id, phi, NoArr, NoArr);
    }
  }
  return true;
}

Conv_NLP2CeresProblem::Conv_NLP2CeresProblem(const shared_ptr<NLP_Factored>& _P) : P(_P) {
  uintA variableDimIntegral, featureDimIntegral;
  arr bounds_lo=bounds[0], bounds_up=bounds[1];
  uint n = P->dimension;
  for(uint i=0; i<bounds_lo.N; i++) {
    if(bounds_lo.elem(i)>=bounds_up.elem(i)) { bounds_lo.elem(i) = -10.;  bounds_up.elem(i) = 10.; }
  }
  variableDimIntegral = integral(P->variableDimensions).prepend(0);
  featureDimIntegral = integral(P->featureDimensions).prepend(0);

  if(n!=variableDimIntegral.last()) throw("");
  if(P->featureTypes.N!=featureDimIntegral.last()) throw("");

  //you must never ever resize these arrays, as ceres takes pointers directly into these fixed memory buffers!
  x_full.resize(variableDimIntegral.last());
  arrA x(P->variableDimensions.N);
  for(uint i=0; i<x.N; i++) {
    x(i).referTo(x_full.p+variableDimIntegral(i), P->variableDimensions(i));
  }

//  phi_full.resize(featureDimIntegral.last());
//  phi.resize(featureDimensions.N);
//  for(uint i=0;i<phi.N;i++){
//    phi(i).referTo(phi_full.p+featureDimIntegral(i), featureDimensions(i));
//  }

//  //we don't have a full Jacobian, as it wouldn't be dense
//  J.resize(phi.N);
//  for(uint i=0;i<phi.N;i++){
//    uint n=0;
//    for(uint j : featureVariables(i)) n += variableDimensions(j);
//    J(i).resize(featureDimensions(i), n);
//  }

  ceresProblem = make_shared<ceres::Problem>();

  for(arr& xi: x) {
    if(xi.N) {
      ceresProblem->AddParameterBlock(xi.p, xi.N);
      assert(bounds_lo.N == x_full.N);
      assert(bounds_up.N == x_full.N);
      for(uint i=0; i<xi.N; i++) {
        uint i_all = i + xi.p-x_full.p;
        ceresProblem->SetParameterLowerBound(xi.p, i, bounds_lo(i_all));
        ceresProblem->SetParameterUpperBound(xi.p, i, bounds_up(i_all));
      }
    }
  }

  for(uint i=0; i<P->featureDimensions.N; i++) {
    if(P->featureDimensions(i)) {
      assert(P->featureTypes(i) == OT_sos);
      std::vector<double*> parameter_blocks;
      for(uint k=0; k<P->featureVariables(i).N; k++) {
        int var = P->featureVariables(i)(k);
        if(var>=0) parameter_blocks.push_back(x(var).p);
//        parameter_blocks(k) = x().p;
      }
      auto fct = new Conv_Feature2CostFunction(*this, i, P->variableDimensions, P->featureDimensions, P->featureVariables);
      ceresProblem->AddResidualBlock(fct, nullptr, parameter_blocks);
    }
  }
}

#else //RAI_CERES

arr CeresInterface::solve() {
  NICO
}

Conv_NLP2Ceres::Conv_NLP2Ceres(const shared_ptr<NLP_Factored>& _P) : P(_P) {
  NICO
}

#endif

