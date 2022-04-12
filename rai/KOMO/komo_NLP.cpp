#include "komo_NLP.h"

#include "../Kin/frame.h"
#include "../Kin/proxy.h"

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

void Conv_KOMO_NLP::report(std::ostream& os, int verbose) {
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

Conv_KOMO_FactoredNLP::Conv_KOMO_FactoredNLP(KOMO& _komo) : komo(_komo) {
  komo.run_prepare(0.);
  komo.pathConfig.jacMode = Configuration::JM_sparse;

  //create variable index
  uint xDim=0;
  uint varId=0;
  FrameL roots = komo.pathConfig.getRoots();
  DofL activeJoints;
  //each frame varies only with a single variable (THAT'S A LIMITING ASSUMPTION)
  uintA frameID2VarId;
  frameID2VarId.resize(komo.pathConfig.frames.N) = UINT_MAX;
  for(Frame *f:roots) {
    if(f->ID < komo.timeSlices(komo.k_order,0)->ID) continue; //ignore prefixes!
    DofL varDofs;
    FrameL branch;
    f->getSubtree(branch);
    uint varDim=0;
    for(Frame* b:branch){
      frameID2VarId(b->ID) = varId; //all frames in the branch depend on this variable
      if(b->joint && b->joint->active && b->joint->type!=JT_rigid){ //and we collect all branch active dofs into this variable
        activeJoints.append(b->joint);
        if(!b->joint->mimic && b->joint->dim){ //this is different to Configuration::calc_indexActiveJoints
          //           cout <<b->name <<", ";
          varDofs.append(b->joint);
          varDim += b->joint->dim;
        }
      }
    }
//    cout <<"--" <<endl;
    CHECK_EQ(varId, getNumVariables(), "");
    __variableIndex.append( VariableIndexEntry{ varDofs, varDim } );
    xDim += varDim;
    varId++;
  }
  CHECK_EQ(xDim, komo.pathConfig.getJointStateDimension(), "");

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveJoints(activeJoints);

  //create feature index
  uint featId=0;
  for(shared_ptr<GroundedObjective>& ob:komo.objs) {
    uint featDim = ob->feat->dim(ob->frames);
    uintA featVars;
    for(Frame *f:ob->frames){ //which frames does the objective depend on?
      uint varId = frameID2VarId(f->ID); //which variable parameterized that frame
      if(varId!=UINT_MAX) featVars.setAppendInSorted(varId);
    }
    __featureIndex.append( FeatureIndexEntry{ ob, featDim, featVars } );
    featId++;
  }

  //define signature
  dimension = komo.pathConfig.getJointStateDimension();
  komo.getBounds(bounds_lo, bounds_up);
  featureTypes.clear();
  for(uint f=0; f<getNumFeatures(); f++) {
    featureTypes.append(consts<ObjectiveType>(featureIndex(f).ob->type, featureIndex(f).dim));
  }

  //define factorization
  variableDimensions.resize(getNumVariables());
  for(uint i=0; i<getNumVariables(); i++) {
    variableDimensions(i) = variableIndex(i).dim;
  }

  featureDimensions.resize(getNumFeatures());
  featureVariables.resize(getNumFeatures());
  arr y, J;

  for(uint f=0; f<getNumFeatures(); f++) {
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    if(!F.dim) continue;
    featureVariables(f) = F.varIds;
  }
}

void Conv_KOMO_FactoredNLP::subSelect(const uintA& activeVariables, const uintA& conditionalVariables){
  uintA allVars;
  for(uint i:activeVariables) allVars.setAppendInSorted(i);
  for(uint i:conditionalVariables) allVars.setAppendInSorted(i);

  DofL activeJoints;
  for(uint v:activeVariables){
    VariableIndexEntry& V = __variableIndex(v);
    for(Dof *d:V.dofs) activeJoints.append(dynamic_cast<Joint*>(d));
  }
  subVars = activeVariables;

  //ensure that komo.pathConfig uses the same indexing -- that its activeJoint set is indexed exactly as consecutive variables
  komo.pathConfig.setActiveJoints(activeJoints);
  komo.run_prepare(0.);

  subFeats.clear();
  for(uint f=0;f<__featureIndex.N;f++){
    FeatureIndexEntry& F = __featureIndex(f);
    bool active=true;
    for(int j:F.varIds){
      if(!allVars.containsInSorted(j)) { //only objectives that link only to X \cup Y
        active=false;
      }
    }
    if(active) subFeats.append(f);
  }
}

String Conv_KOMO_FactoredNLP::getVariableName(uint var_id){
  Dof* d = variableIndex(var_id).dofs.first();
  Joint *j = dynamic_cast<Joint*>(d);
  CHECK(j, "");
  return STRING(j->frame->name <<'.' <<j->frame->ID);
}

void Conv_KOMO_FactoredNLP::setSingleVariable(uint var_id, const arr& x) {
  //komo.pathConfig.ensure_q();

  VariableIndexEntry& v = variableIndex(var_id);
  CHECK_EQ(v.dim, x.N, "");
//  uint dofIdx = v.xIndex;
  uint xIdx=0;
  for(Dof *dof:v.dofs){
    dof->setDofs(x, xIdx);
    //      if(!j->mimic){
    if(dof->active){
      for(uint ii=0; ii<dof->dim; ii++) komo.pathConfig.q.elem(dof->qIndex+ii) = x(xIdx+ii);
    }else{
      HALT("shoudn't be here..?");
      for(uint ii=0; ii<dof->dim; ii++) komo.pathConfig.qInactive.elem(dof->qIndex+ii) = x(xIdx+ii);
    }
    xIdx += dof->dim;
  }
  CHECK_EQ(xIdx, v.dim, "");

  komo.pathConfig.proxies.clear();
  komo.pathConfig._state_q_isGood=true;
  komo.pathConfig._state_proxies_isGood=false;
}

void Conv_KOMO_FactoredNLP::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
  FeatureIndexEntry& F = featureIndex(feat_id);

  phi = F.ob->feat->eval(F.ob->frames);
  J = phi.J();
  //  cout <<"EVAL '" <<F.ob->name() <<"' phi:" <<phi <<endl;
}

void Conv_KOMO_FactoredNLP::report(std::ostream& os, int verbose) {
  komo.reportProblem(os);
  komo.pathConfig.ensure_q();

  if(verbose>1){
    for(uint v=0; v<getNumVariables(); v++) {
      VariableIndexEntry& V = variableIndex(v);
      os <<"Variable " <<v <<" dim:" <<V.dim <<" dofs:{ ";
      for(Dof* d:V.dofs){
        Joint *j = dynamic_cast<Joint*>(d);
        CHECK(j, "");
        os <<j->frame->name <<'.' <<j->frame->ID <<' ';
      }
      os <<'}' <<endl;
    }


    arr y, J;
    for(uint f=0; f<getNumFeatures(); f++) {
      FeatureIndexEntry& F = featureIndex(f);
      os <<"Feature " <<f <<" dim:" <<F.dim <<" vars:( ";
      for(uint& i:F.varIds) os <<i <<' ';
      os <<") desc:" <<F.ob->feat->shortTag(komo.pathConfig);
      evaluateSingleFeature(f, y, J, NoArr);
      os <<" y:" <<y <<endl;
//      os <<"J:" <<J <<endl;
    }
  }

  if(verbose>3) komo.view(true, "Conv_KOMO_FineStructuredProblem - report");
  if(verbose>4) komo.view_play(true);
  if(verbose>5){
    rai::system("mkdir -p z.vid");
    komo.view_play(false, .1, "z.vid/");
    if(verbose>3) komo.view(true, "Conv_KOMO_SparseNonfactored - video saved in z.vid/");
  }
}

//===========================================================================

Conv_KOMO_TimeFactoredNLP::Conv_KOMO_TimeFactoredNLP(KOMO& _komo) : komo(_komo) {
  //count variables
  uint xDim = getDimension();

  //create variable index
  xIndex2VarId.resize(xDim);
  variableIndex.resize(komo.T);
  uint count=0;
  for(uint t=0; t<komo.T; t++) {
    VariableIndexEntry& V = variableIndex(t);
    V.t = t;
    V.dim = komo.getConfiguration_qAll(t).N; //.configurations(t+komo.k_order)->getJointStateDimension();
    V.xIndex = count;
    for(uint i=0; i<V.dim; i++) xIndex2VarId(count++) = t;
  }

  //count features
  uint F=0;
  NIY;
  //  for(shared_ptr<Objective>& ob:komo.objectives) if(ob->timeSlices.N) {
  //      CHECK_EQ(ob->timeSlices.nd, 2, "in sparse mode, vars need to be tuples of variables");
//      F += ob->timeSlices.d0;
//    }
  featureIndex.resize(F);

  //create feature index
  uint f=0;
  uint fDim = 0;
  for(shared_ptr<GroundedObjective>& ob:komo.objs) {
    FeatureIndexEntry& F = featureIndex(f);
    F.ob2 = ob;
//    F.Ctuple = komo.configurations.sub(convert<uint, int>(ob->timeSlices+(int)komo.k_order));
//    F.t = l;
    copy(F.varIds, ob->timeSlices);
    F.dim = ob->feat->dim(ob->frames); //dimensionality of this task
    F.phiIndex = fDim;
    fDim += F.dim;
    f++;
  }
  CHECK_EQ(f, featureIndex.N, "");

  featuresDim = fDim;

  //define signature and factorization
  dimension = komo.pathConfig.getJointStateDimension();

  featureTypes.resize(featuresDim);
  komo.featureNames.resize(featuresDim);
  uint M=0;
  for(shared_ptr<GroundedObjective>& ob : komo.objs) {
    uint m = ob->feat->dim(ob->frames);
    for(uint i=0; i<m; i++) featureTypes(M+i) = ob->type;
    for(uint i=0; i<m; i++) komo.featureNames(M+i) = "TODO";
    M += m;
  }

  komo.featureTypes = featureTypes;

  komo.getBounds(bounds_lo, bounds_up);

  variableDimensions.resize(variableIndex.N);
  variableDimensions.resize(variableIndex.N);
  for(uint i=0; i<variableIndex.N; i++) variableDimensions(i) = variableIndex(i).dim;

  featureDimensions.resize(featureIndex.N);
  featureVariables.resize(featureIndex.N);
  for(uint f=0; f<featureIndex.N; f++) {
    FeatureIndexEntry& F = featureIndex(f);
    featureDimensions(f) = F.dim;
    featureVariables(f) = F.varIds;
  }
}

arr Conv_KOMO_TimeFactoredNLP::getInitializationSample(const arr& previousOptima) {
  komo.run_prepare(.01);
  return komo.x;
}

void Conv_KOMO_TimeFactoredNLP::setAllVariables(const arr& x) {
  komo.set_x(x);
}

void Conv_KOMO_TimeFactoredNLP::setSingleVariable(uint var_id, const arr& x) {
  komo.set_x(x, {var_id});
}

void Conv_KOMO_TimeFactoredNLP::report(){
  reportAfterPhiComputation(komo);
}

void Conv_KOMO_TimeFactoredNLP::evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H) {
#if 1
  if(!komo.featureValues.N) {
    FeatureIndexEntry& Flast = featureIndex.last();
    komo.featureValues.resize(Flast.phiIndex+Flast.dim).setZero();
  }

  FeatureIndexEntry& F = featureIndex(feat_id);

  phi = F.ob2->feat->eval(F.ob2->frames);
  CHECK_EQ(phi.N, F.dim, "");

  komo.featureValues.setVectorBlock(phi, F.phiIndex);

  if(!J) return;

  CHECK_EQ(phi.N, phi.J().d0, "");
  CHECK_EQ(phi.J().nd, 2, "");
  if(absMax(phi)>1e10) RAI_MSG("WARNING phi=" <<phi);

  if(isSparseMatrix(phi.J())) {
    auto& S = phi.J().sparse();

    uint n=0;
    for(uint v:F.varIds) n += variableIndex(v).dim;
    J.resize(phi.N, n).setZero();

    for(uint k=0; k<phi.J().N; k++) {
      uint i = S.elems(k, 0);
      uint j = S.elems(k, 1);
      double x = phi.J().elem(k);
      uint var = xIndex2VarId(j);
      VariableIndexEntry& V = variableIndex(var);
      uint var_j = j - V.xIndex;
      CHECK(var_j < V.dim, "");
      if(V.dim == J.d1) {
        J(i, var_j) += x;
      } else {
        bool good=false;
        uint offset=0;
        for(uint v:F.varIds) {
          if(v==var) {
            J(i, offset+var_j) += x;
            good=true;
            break;
          }
          offset += variableIndex(v).dim;
        }
        CHECK(good, "Jacobian is non-zero on variable " <<var <<", but indices say that feature depends on " <<F.varIds);
      }
    }
  } else if(isRowShifted(phi.J())){
    J = phi.J();
  } else {
    HALT("??");
#if 0 //ndef KOMO_PATH_CONFIG
    intA vars = F.ob->configs[F.t];
    uintA kdim = getKtupleDim(F.Ctuple).prepend(0);
    for(uint j=vars.N; j--;) {
      if(vars(j)<0) {
        Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
      }
    }
    J = Jy;
#endif
  }

#else
  //count to the feat_id;

  uint count=0;
  for(shared_ptr<Objective>& ob:komo.objectives) {
    for(uint l=0; l<ob->configs.d0; l++) {
      if(count==feat_id) { //this is the feature we want!
        ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
        uintA kdim = getKtupleDim(Ktuple);
        kdim.prepend(0);

        //query the task map and check dimensionalities of returns
        arr Jy;
        ob->feat->eval(phi, Jy, Ktuple);
        if(!!J && isSpecial(Jy)) Jy = unpack(Jy);

        if(!!J) CHECK_EQ(phi.N, Jy.d0, "");
        if(!!J) CHECK_EQ(Jy.nd, 2, "");
        if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
        if(!phi.N) continue;
        if(absMax(phi)>1e10) RAI_MSG("WARNING y=" <<phi);

        if(!!J) {
          for(uint j=ob->configs.d1; j--;) {
            if(ob->configs(l, j)<0) {
              Jy.delColumns(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
            }
          }
          J = Jy;
        }

        if(!!H) NIY;

        if(komo.featureValues.N!=featDimIntegral.last()) {
          komo.featureValues.resize(featDimIntegral.last());
        }
        komo.featureValues.setVectorBlock(phi, featDimIntegral(feat_id));

        return;
      }
      count++;
    }
  }
#endif
}


//===========================================================================

#if 0
void KOMO::TimeSliceProblem::getDimPhi() {
  CHECK_EQ(komo.configurations.N, komo.k_order+komo.T, "configurations are not setup yet: use komo.reset()");
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      M += ob->feat->__dim_phi(Ktuple); //dimensionality of this task
    }
  }
  dimPhi = M;
}

void KOMO::TimeSliceProblem::getFeatureTypes(ObjectiveTypeA& ft) {
  if(!dimPhi) getDimPhi();
  ft.resize(dimPhi);

  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uint m = ob->feat->__dim_phi(Ktuple);
      if(!!ft) for(uint i=0; i<m; i++) ft(M+i) = ob->type;
      M += m;
    }
  }
  komo.featureTypes = ft;
}

void KOMO::TimeSliceProblem::evaluate(arr& phi, arr& J, const arr& x) {
  komo.set_x2(x, TUP(slice));

  if(!dimPhi) getDimPhi();

  phi.resize(dimPhi);
  if(!!J) J.resize(dimPhi, x.N).setZero();

  uintA x_index = getKtupleDim(komo.configurations({komo.k_order, -1}));
  x_index.prepend(0);

  arr y, Jy;
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    ptr<Objective> ob = komo.objectives.elem(i);
    CHECK_EQ(ob->configs.nd, 2, "only in sparse mode!");
    if(ob->configs.d1!=1) continue; //ONLY USE order 0 objectives!!!!!
    for(uint l=0; l<ob->configs.d0; l++) {
      if(ob->configs(l, 0)!=slice) continue; //ONLY USE objectives for this slice
      ConfigurationL Ktuple = komo.configurations.sub(convert<uint, int>(ob->configs[l]+(int)komo.k_order));
      uintA kdim = getKtupleDim(Ktuple);
      kdim.prepend(0);

      //query the task map and check dimensionalities of returns
      ob->feat->eval(y, Jy, Ktuple);
      if(!!J) CHECK_EQ(y.N, Jy.d0, "");
      if(!!J) CHECK_EQ(Jy.nd, 2, "");
      if(!!J) CHECK_EQ(Jy.d1, kdim.last(), "");
      if(!y.N) continue;
      if(absMax(y)>1e10) RAI_MSG("WARNING y=" <<y);

      //write into phi and J
      phi.setVectorBlock(y, M);

      if(!!J) {
        if(isSpecial(Jy)) Jy = unpack(Jy);
        J.setMatrixBlock(Jy, M, 0);
      }

      //counter for features phi
      M += y.N;
    }
  }

  CHECK_EQ(M, dimPhi, "");
  komo.featureValues = phi;
  if(!!J) komo.featureJacobians.resize(1).scalar() = J;

  reportAfterPhiComputation(komo);
}

#endif


}//namespace
