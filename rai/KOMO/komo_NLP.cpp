#include "komo_NLP.h"

#include "../Kin/frame.h"
#include "../Kin/proxy.h"
#include "../Kin/forceExchange.h"

namespace rai{

//===========================================================================

void reportAfterPhiComputation(KOMO& komo) {
  if(komo.opt.verbose>6 || komo.opt.animateOptimization>2) {
    //  komo.reportProxies();
    cout <<komo.getReport(true) <<endl;
  }
  if(komo.opt.animateOptimization>0) {
    komo.view(komo.opt.animateOptimization>1, "optAnim");
    if(komo.opt.animateOptimization>2){
      komo.view_play(komo.opt.animateOptimization>3);
    }
    //  komo.plotPhaseTrajectory();
    //  wait();
    //  reportProxies();
  }
}

//===========================================================================

void Conv_KOMO_NLP::evaluate(arr& phi, arr& J, const arr& x) {
  //-- set the trajectory
  komo.set_x(x);
  if(sparse){
    komo.pathConfig.jacMode = Configuration::JM_sparse;
  }else {
    komo.pathConfig.jacMode = Configuration::JM_dense;
  }

  phi.resize(featureTypes.N);
  if(!!J) {
    if(sparse) {
      J.sparse().resize(phi.N, x.N, 0);
    } else {
      J.resize(phi.N, x.N).setZero();
    }
  }

  komo.sos=komo.ineq=komo.eq=0.;

  komo.timeFeatures -= cpuTime();

  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
      //query the task map and check dimensionalities of returns
      arr y = ob->feat->eval(ob->frames);
//      cout <<"EVAL '" <<ob->name() <<"' phi:" <<y <<endl <<y.J() <<endl<<endl;
      if(!y.N) continue;
      checkNan(y);
      if(!!J){
        CHECK(y.jac, "Jacobian needed but missing");
        CHECK_EQ(y.J().nd, 2, "");
        CHECK_EQ(y.J().d0, y.N, "");
        CHECK_EQ(y.J().d1, komo.pathConfig.getJointStateDimension(), "");
      }
//      uint d = ob->feat->dim(ob->frames);
//      if(d!=y.N){
//        d  = ob->feat->dim(ob->frames);
//        ob->feat->eval(y, y.J(), ob->frames);
//      }
//      CHECK_EQ(d, y.N, "");
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      arr yJ = y.J_reset();
      phi.setVectorBlock(y, M);

      double scale=1.;
      if(komo.opt.unscaleEqIneqReport && ob->feat->scale.N) scale = absMax(ob->feat->scale);
      CHECK_GE(scale, 1e-4, "");

      if(ob->type==OT_sos) komo.sos+=sumOfSqr(y); // / max(ob->feat->scale);
      else if(ob->type==OT_ineq) komo.ineq += sumOfPos(y) / scale;
      else if(ob->type==OT_eq) komo.eq += sumOfAbs(y) / scale;

      if(!!J) {
        if(sparse){
          yJ.sparse().reshape(J.d0, J.d1);
          yJ.sparse().colShift(M);
          J += yJ;
        }else{
          J.setMatrixBlock(yJ, M, 0);
        }
      }

      //counter for features phi
      M += y.N;
  }

  komo.timeFeatures += cpuTime();

  CHECK_EQ(M, phi.N, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;

  reportAfterPhiComputation(komo);

  if(quadraticPotentialLinear.N) {
    phi.append((~x * quadraticPotentialHessian * x).scalar() + scalarProduct(quadraticPotentialLinear, x));
    J.append(quadraticPotentialLinear);
  }
}

void Conv_KOMO_NLP::getFHessian(arr& H, const arr& x) {
  if(quadraticPotentialLinear.N) {
    H = quadraticPotentialHessian;
  } else {
    H.clear();
  }
}

void Conv_KOMO_NLP::report(std::ostream& os, int verbose, const char* msg) {
  komo.reportProblem(os);
  if(verbose>1) os <<komo.getReport(verbose>3);
  if(verbose>2) komo.view(verbose>3, "Conv_KOMO_SparseNonfactored - report");
  if(verbose>4) while(komo.view_play(true));
  if(verbose>6){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
  }
}

Conv_KOMO_NLP::Conv_KOMO_NLP(KOMO& _komo, bool sparse) : komo(_komo), sparse(sparse) {
  dimension = komo.pathConfig.getJointStateDimension();

  komo.getBounds(bounds_lo, bounds_up);

  //-- feature types
  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) M += ob->feat->dim(ob->frames);

  featureTypes.resize(M);
  komo.featureNames.clear();
  M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint j=0; j<m; j++) komo.featureNames.append(ob->feat->shortTag(komo.pathConfig));
    M += m;
  }
  if(quadraticPotentialLinear.N) {
    featureTypes.append(OT_f);
  }
  komo.featureTypes = featureTypes;
}

arr Conv_KOMO_NLP::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

//===========================================================================

Conv_KOMO_FactoredNLP::Conv_KOMO_FactoredNLP(KOMO& _komo, const rai::Array<DofL>& varDofs) : komo(_komo) {
  komo.pathConfig.jacMode = rai::Configuration::JM_sparse;
  komo.run_prepare(0.);

  //NLP_Factored signature: variables
  uint xDim=0;
  DofL activeDofs;
  uintA xIndex2varIndex;
  __variableIndex.resize(varDofs.N);
  for(uint i=0;i<varDofs.N;i++) {
    DofL& dofs = varDofs(i);
    __variableIndex(i).dofs = dofs;
    activeDofs.append(dofs);

    //variable dimension
    uint varDim=0;
    for(Dof* d:dofs){
      varDim += d->dim;
      xIndex2varIndex.append(consts<uint>(i, d->dim));
    }
    __variableIndex(i).dim = varDim;
    xDim += varDim;

    //variable name
    String name;
    String A; A <<dofs(0)->frame->name <<'.' <<(dofs(0)->frame->ID/komo.timeSlices.d1 - komo.k_order);
    String B; B <<dofs(-1)->frame->name <<'.' <<(dofs(-1)->frame->ID/komo.timeSlices.d1 - komo.k_order);
    if(dofs.N>1){
      name <<A <<"--" <<B;
    }else if(dofs(0)->fex()){
      name <<"F--" <<A <<"--" <<dofs(0)->fex()->a.name <<"--" <<dofs(0)->fex()->b.name;
    }else if(dofs(0)->mimic){
      name <<"M--" <<A;
    }else{
      name <<A;
    }
    __variableIndex(i).name = name;
  }
  CHECK_EQ(xDim, komo.pathConfig.getJointStateDimension(), "");
  CHECK_EQ(xDim, xIndex2varIndex.N, "");
//  cout <<"xIndex2varIndex" <<xIndex2varIndex <<endl;

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveDofs(activeDofs);
  komo.pathConfig.ensure_q();
  komo.pathConfig.checkConsistency();

  //NLP_Factored signature: features
  __featureIndex.resize(komo.objs.N);
  for(uint f=0;f<featsN();f++){
    std::shared_ptr<GroundedObjective>& ob = komo.objs(f);
    __featureIndex(f).ob = ob;
//    __featureIndex(f).dim = ob->feat->dim(ob->frames);
  }

  //get variable dependance from querying the sparse Jacobian!
  for(uint f=0;f<featsN();f++){
    std::shared_ptr<GroundedObjective>& ob = komo.objs(f);
    arr y = ob->feat->eval(ob->frames);
    if(y.N){
      CHECK(isSparse(y.J()), "");
      SparseMatrix& S = y.J().sparse();
      for(uint i=0;i<S.elems.d0;i++){
        uint xIndex = S.elems(i,1);
        uint var = xIndex2varIndex(xIndex);
        __featureIndex(f).vars.setAppendInSorted(var);
      }
    }
  }

  //this also creates the NLP_Factored signature
  subSelect({}, {});
}

void Conv_KOMO_FactoredNLP::subSelect(const uintA& activeVariables, const uintA& conditionalVariables){
  uintA subVarsInv(__variableIndex.N);
  subVarsInv = UINT_MAX;
  DofL activeDofs;

  if(!activeVariables.N){ //select all

    subVars.clear();
    subFeats.clear();
    subVarsInv.setStraightPerm();
    for(uint i=0;i<__variableIndex.N;i++) activeDofs.append(__variableIndex(i).dofs);

  }else{ //really sub-select

    subVars = activeVariables;

    uintA allVars;
    for(uint i:activeVariables) allVars.setAppendInSorted(i);
    for(uint i:conditionalVariables) allVars.setAppendInSorted(i);

    for(uint v:activeVariables) activeDofs.append(__variableIndex(v).dofs);

    subFeats.clear();
    for(uint f=0;f<__featureIndex.N;f++){
      bool active=true;
      bool oneActiveVar=false;
      for(int j:__featureIndex(f).vars){
        if(!allVars.containsInSorted(j)) { //only objectives that link only to X \cup Y
          active=false;
          break;
        }
        if(activeVariables.contains(j)) oneActiveVar=true;
      }
      if(active && oneActiveVar) subFeats.append(f);
    }
    if(!subFeats.N){
      LOG(-1) <<"THIS SUBPROBLEM HAS NO FEATURES!";
    }

    for(uint i=0;i<subVars.N;i++) subVarsInv(subVars(i)) = i;
  }

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveDofs(activeDofs);
  komo.run_prepare(0.);

  //NLP signature
  dimension = komo.pathConfig.getJointStateDimension();
  komo.getBounds(bounds_lo, bounds_up);

  //create NLP_Factored signature
  variableDimensions.resize(varsN());
  for(uint i=0;i<varsN();i++) variableDimensions(i) = vars(i).dim;
  featureDimensions.resize(featsN());
  featureVariables.resize(featsN());
  featureTypes.clear();
  for(uint i=0;i<featsN();i++){
    featureDimensions(i) = feats(i).ob->feat->dim(feats(i).ob->frames); //__featureIndex(f).dim = ob->feat->dim(ob->frames);
    featureVariables(i) = subVarsInv.sub(feats(i).vars);
    featureTypes.append(consts<ObjectiveType>(feats(i).ob->type, featureDimensions(i)));
  }
}

arr Conv_KOMO_FactoredNLP::getInitializationSample(const arr& previousOptima) {
#if 0
  komo.run_prepare(0.01);
  return komo.x;
#else
  for(Dof *d:komo.pathConfig.activeDofs){
    if(false && d->limits.N && d->dim!=1){ //HACK!!
      arr q(d->dim);
      for(uint k=0; k<d->dim; k++) { //in case joint has multiple dimensions
        double lo = d->limits.elem(2*k+0); //lo
        double up = d->limits.elem(2*k+1); //up
        q(k) = rnd.uni(lo,up);
      }
      d->setDofs(q);
    }else{
      arr q = d->calcDofsFromConfig();
      rndGauss(q, 0.01, true);
      d->setDofs(q);
    }
  }
  komo.pathConfig._state_q_isGood=false;
//  komo.run_prepare(.0);
//  komo.initRandom();
  komo.x = komo.pathConfig.getJointState();
  {
    arr lo, up;
    komo.getBounds(lo, up);
    boundClip(komo.x, lo, up);
  }
  komo.set_x(komo.x);
//  komo.view(true, "randomInit");
  return komo.x;
#endif
}

void Conv_KOMO_FactoredNLP::randomizeSingleVariable(uint var_id){
  for(Dof *d:__variableIndex(var_id).dofs){
    if(d->limits.N && d->dim!=1){ //HACK!!
      arr q(d->dim);
      for(uint k=0; k<d->dim; k++) { //in case joint has multiple dimensions
        double lo = d->limits.elem(2*k+0); //lo
        double up = d->limits.elem(2*k+1); //up
        q(k) = rnd.uni(lo,up);
      }
      LOG(0) <<"### initializing " <<d->frame->name <<" with " <<q;
      d->setDofs(q);
    }else{
      arr q = d->calcDofsFromConfig();
      rndGauss(q, 0.01, true);
      d->setDofs(q);
    }
  }
}

arr Conv_KOMO_FactoredNLP::getSingleVariableInitSample(uint var_id){
  arr z;
  for(Dof *d:__variableIndex(var_id).dofs){
    //if joint, find previous dof:
    if(d->frame->ID >= komo.timeSlices.d1){ //is joint and prev time slice exists
      Frame *prev = komo.pathConfig.frames.elem(d->frame->ID - komo.timeSlices.d1); //grab frame from prev time slice
      CHECK(prev, "");
      //init from relative pose (as in applySwitch)
      d->frame->set_X() = prev->ensure_X(); //copy the relative pose (switch joint initialization) from the first application
      arr q = d->calcDofsFromConfig();
#if 1
      d->setDofs(q); //also sets it for all mimicers
#else
      for(Dof *m: d->mimicers) m->frame->set_Q() = d->frame->get_Q();
#endif
      z.append(q);
    }else{//otherwise???
      z.append(d->calcDofsFromConfig());
    }
  }
  return z;
}

void Conv_KOMO_FactoredNLP::setSingleVariable(uint var_id, const arr& x) {
  CHECK_EQ(vars(var_id).dim, x.N, "");
  komo.pathConfig.setDofState(x, vars(var_id).dofs);
}

void Conv_KOMO_FactoredNLP::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
  std::shared_ptr<GroundedObjective>& ob = feats(feat_id).ob;
  phi = ob->feat->eval(ob->frames);
  J = phi.J();
}

void Conv_KOMO_FactoredNLP::evaluate(arr& phi, arr& J, const arr& x) {
  NLP_Factored::evaluate(phi, J, x);
  reportAfterPhiComputation(komo);
}

void Conv_KOMO_FactoredNLP::report(std::ostream& os, int verbose, const char* msg) {
//  komo.reportProblem(os);
  komo.pathConfig.ensure_q();

  if(verbose>4){
    for(uint i=0; i<varsN(); i++) {
      os <<"Variable " <<i;
      if(subVars.N) os <<"[" <<subVars(i) <<"]";
      os <<" '" <<vars(i).name <<"' dim:" <<vars(i).dim  <<" dofs:" <<vars(i).dofs.N;
      os <<" {";
      if(vars(i).dofs.N<=2){
        for(Dof *d:vars(i).dofs){
          os <<" qIdx:" <<d->qIndex;
          if(d->limits.N) os <<" limits:" <<d->limits;
          if(d->isStable) os <<" STABLE";
        }
      }else{ os <<" ..."; }
      os <<" }" <<endl;
    }

    arr y, J;
    for(uint f=0; f<featsN(); f++) {
      std::shared_ptr<GroundedObjective>& ob = feats(f).ob;
      os <<"Feature " <<f;
      if(subVars.N) os <<"[" <<subFeats(f) <<"]";
      os <<" '" <<ob->feat->shortTag(komo.pathConfig) <<"' dim:" <<feats(f).ob->feat->dim(feats(f).ob->frames) <<" vars: " <<featureVariables(f) <<'=' <<feats(f).vars <<"=[ ";
      for(uint& i:featureVariables(f)) if(i!=UINT_MAX) os <<vars(i).name <<' '; else os <<"% ";
      os <<"]" ;
      evaluateSingleFeature(f, y, J, NoArr);
      os <<" y:" <<y.noJ() <<endl;
//      os <<"J:" <<J <<endl;
    }

    os <<"NLP_Factored signature:"
      <<"\n  variableDimensions: " <<variableDimensions
     <<"\n  featureDimensions: " <<featureDimensions
    <<"\n  featureVariables: " <<featureVariables <<endl;
  }

  if(msg) os <<" *** " <<msg <<" ***"<<endl;

  if(verbose>3) komo.view(verbose>4, STRING("Conv_KOMO_FineStructuredProblem - " <<msg));
  if(verbose>5) komo.view_play(true);
  if(verbose>6){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
    if(verbose>3) komo.view(true, "Conv_KOMO_SparseNonfactored - video saved in z.vid/");
  }
}

}//namespace
