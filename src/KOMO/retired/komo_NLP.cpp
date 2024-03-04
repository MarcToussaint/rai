/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//this treats each time slice as its own variable
struct Conv_KOMO_TimeFactoredNLP : NLP_Factored {
  KOMO& komo;

  struct VariableIndexEntry { uint t; uint dim; uint xIndex; };
  rai::Array<VariableIndexEntry> variableIndex;

  struct FeatureIndexEntry { shared_ptr<Objective> ob; shared_ptr<GroundedObjective> ob2; uint t; uintA varIds; uint dim; uint phiIndex; };
  rai::Array<FeatureIndexEntry> featureIndex;

  uintA xIndex2VarId;
  uint featuresDim;

  Conv_KOMO_TimeFactoredNLP(KOMO& _komo);

  virtual arr getInitializationSample(const arr& previousOptima= {});

  virtual void setAllVariables(const arr& x);
  virtual void setSingleVariable(uint var_id, const arr& x); //set a single variable block
  virtual void evaluateSingleFeature(uint feat_id, arr& phi, arr& J, arr& H); //get a single feature block
  virtual void report();
};

struct Conv_KOMO_TimeSliceProblem : NLP {
  KOMO& komo;
  int slice;

  Conv_KOMO_TimeSliceProblem(KOMO& _komo, int _slice) : komo(_komo), slice(_slice) {
    dimension = komo.pathConfig.getJointStateDimension();
  }

  void getDimPhi();

  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

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

void Conv_KOMO_TimeFactoredNLP::report() {
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
  } else if(isRowShifted(phi.J())) {
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
    shared_ptr<Objective> ob = komo.objectives.elem(i);
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
    shared_ptr<Objective> ob = komo.objectives.elem(i);
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
  komo.set_x2(x, uintA{slice});

  if(!dimPhi) getDimPhi();

  phi.resize(dimPhi);
  if(!!J) J.resize(dimPhi, x.N).setZero();

  uintA x_index = getKtupleDim(komo.configurations({komo.k_order, -1}));
  x_index.prepend(0);

  arr y, Jy;
  uint M=0;
  for(uint i=0; i<komo.objectives.N; i++) {
    shared_ptr<Objective> ob = komo.objectives.elem(i);
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
