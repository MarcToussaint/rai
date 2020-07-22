#include "opt-ceres.h"

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

//#undef CHECK
//#undef CHECK_EQ
//#undef CHECK_LE
//#undef CHECK_GE
//#define CHECK(cond, msg) \
//  if(!(cond)){ LOG(-2) <<"CHECK failed: '" <<#cond <<"' -- " <<msg;  throw std::runtime_error(rai::errString.p); }
//#define CHECK_EQ(A, B, msg) \
//  if(!(A==B)){ LOG(-2) <<"CHECK_EQ failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); }
//#define CHECK_GE(A, B, msg) \
//  if(!(A>=B)){ LOG(-2) <<"CHECK_GE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); }
//#define CHECK_LE(A, B, msg) \
//  if(!(A<=B)){ LOG(-2) <<"CHECK_LE failed: '" <<#A<<"'=" <<A <<" '" <<#B <<"'=" <<B <<" -- " <<msg; throw std::runtime_error(rai::errString.p); }

class Conv_CostFunction : public ceres::CostFunction {
  std::shared_ptr<MathematicalProgram_Structured> MP;
  uint feature_id;
  uint featureDim;
  intA varIds;
  uintA varDims;
  uint varTotalDim;

public:
  Conv_CostFunction(const ptr<MathematicalProgram_Structured>& _MP,
                    uint _feature_id,
                    const uintA& variableDimensions,
                    const uintA& featureDimensions,
                    const intAA& featureVariables);

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const;
};

Conv_CostFunction::Conv_CostFunction(const ptr<MathematicalProgram_Structured>& _MP, uint _feature_id, const uintA& variableDimensions, const uintA& featureDimensions, const intAA& featureVariables)
  : MP(_MP), feature_id(_feature_id) {
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

bool Conv_CostFunction::Evaluate(const double* const * parameters, double* residuals, double** jacobians) const{
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
      J.reshape(featureDim, varTotalDim);
      MP->evaluateSingleFeature(feature_id, phi, J, NoArr);
    }else{
      MP->evaluateSingleFeature(feature_id, phi, NoArr, NoArr);
    }
  }
  return true;
}

Conv_MatematicalProgram_CeresProblem::Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram_Structured>& _MP) : MP(_MP) {

  //you must never ever resize these arrays, as ceres takes pointers directly into these fixed memory buffers!
  uint n = MP->getDimension();
  MP->getBounds(bounds_lo, bounds_up);
  MP->getFeatureTypes(featureTypes);
  MP->getStructure(variableDimensions, featureDimensions, featureVariables);
  variableDimIntegral = integral(variableDimensions).prepend(0);
  featureDimIntegral = integral(featureDimensions).prepend(0);

  if(n!=variableDimIntegral.last()) throw("");
  if(featureTypes.N!=featureDimIntegral.last()) throw("");

  x_full.resize(variableDimIntegral.last());
  x.resize(variableDimensions.N);
  for(uint i=0;i<x.N;i++){
    x(i).referTo(x_full.p+variableDimIntegral(i), variableDimensions(i));
  }

  phi_full.resize(featureDimIntegral.last());
  phi.resize(featureDimensions.N);
  for(uint i=0;i<phi.N;i++){
    phi(i).referTo(phi_full.p+featureDimIntegral(i), featureDimensions(i));
  }

  //we don't have a full Jacobian, as it wouldn't be dense
  J.resize(phi.N);
  for(uint i=0;i<phi.N;i++){
    uint n=0;
    for(uint j : featureVariables(i)) n += variableDimensions(j);
    J(i).resize(featureDimensions(i), n);
  }

  ceresProblem = make_shared<ceres::Problem>();
  
  for(arr& xi: x){
    if(xi.N){
      ceresProblem->AddParameterBlock(xi.p, xi.N);
#if 1 //bounds
      for(uint i=0;i<xi.N;i++){
        uint i_all = i + xi.p-x_full.p;
        if(bounds_lo(i_all)<bounds_up(i_all)){
          ceresProblem->SetParameterLowerBound(xi.p, i, bounds_lo(i_all));
          ceresProblem->SetParameterUpperBound(xi.p, i, bounds_up(i_all));
        }else{
          ceresProblem->SetParameterLowerBound(xi.p, i, -10.);
          ceresProblem->SetParameterUpperBound(xi.p, i, 10.);
        }
      }
#endif
    }
  }

  for(uint i=0;i<phi.N;i++){
    if(featureDimensions(i)){
      rai::Array<double*> parameter_blocks(featureVariables(i).N);
      for(uint k=0;k<featureVariables(i).N;k++){
        parameter_blocks(k) = x(featureVariables(i)(k)).p;
      }
      auto fct = new Conv_CostFunction(MP, i, variableDimensions, featureDimensions, featureVariables);
      ceresProblem->AddResidualBlock(fct, nullptr, parameter_blocks);
    }
  }
}

#else //RAI_CERES

Conv_MatematicalProgram_CeresProblem::Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram>& _MP) {
  NICO
}

#endif
