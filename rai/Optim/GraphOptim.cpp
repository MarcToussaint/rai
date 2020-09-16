/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "GraphOptim.h"

//===========================================================================

GraphProblem_Structure::GraphProblem_Structure(GraphProblem& _G) : G(_G) {
  //get signature
  uintA variableDimensions, varDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;
  _G.getStructure(variableDimensions, featureVariables, featureTypes);
  varDimIntegral = integral(variableDimensions);

  StringA varNames, phiNames;
  _G.getSemantics(varNames, phiNames);

  //setup variable nodes
  V.resize(variableDimensions.N);
  for(uint i=0; i<V.N; i++) {
    V(i).x_index = i;
    V(i).x_dim = variableDimensions(i);
    V(i).description = varNames(i);
  }

  //setup objective nodes
  O.resize(featureVariables.N);
  for(uint k=0; k<O.N; k++) {
    O(k).phi_index = k;
    O(k).vars = featureVariables(k);
    for(int i=O(k).vars.N; i--;) if(O(k).vars(i)<0) O(k).vars.remove(i); //remove variables of negative index
    O(k).type = featureTypes(k);
    for(int i:O(k).vars) if(i>=0) V(i).objs.setAppendInSorted(k);
    O(k).description = phiNames(k);
  }

//  for(uint l=0;l<O.N;l++){
//    for(int i:O(l).vars) if(i>=0) CHECK(V(i).objs.contains(l), "");
//  }

//  for(uint j=0;j<V.N;j++){
//    for(uint k:V(j).objs) CHECK(O(k).vars.contains(j), "");
//  }
}

//===========================================================================

SubGraphProblem::SubGraphProblem(GraphProblem_Structure& G, const uintA& _X, const uintA& _Y) : G(G) {
  reset(_X, _Y);
}

void SubGraphProblem::reset(const uintA& _X, const uintA& _Y) {
  X = _X;
  Y = _Y;
  for(uint i:Y) CHECK(!X.containsInSorted((i)), "");

  Gindex2SubIndex = consts<int>(-1, G.V.N);
  for(uint i=0; i<X.N; i++) Gindex2SubIndex(_X(i)) = i;

  uintA allVars;
  for(uint i:X) allVars.setAppendInSorted(i);
  for(uint i:Y) allVars.setAppendInSorted(i);

  Phi.clear();
  for(uint i:X) { //include only objectives linked to X!
    for(uint k:G.V(i).objs) {
      bool contains=true;
      for(int j:G.O(k).vars) {
        if(j>=0 && !allVars.containsInSorted(j)) { //only objectives that link only to X \cup Y
          contains=false;
          break;
        }
      }

      if(contains) {
        Phi.setAppendInSorted(k);
      }
    }
  }

  if(false) {
    cout <<" ** subproblem - VARIABLES:  #=" <<X.N <<endl;
    if(X.N<30) for(uint i:X) cout <<"    " <<i <<": " <<G.V(i).description <<endl;
    cout <<" ** subproblem - CONSTANTS:  #=" <<Y.N <<endl;
    if(Y.N<30) for(uint i:Y) cout <<"    " <<i <<": " <<G.V(i).description <<endl;
    cout <<" ** subproblem - OBJECTIVES:  #=" <<Phi.N <<endl;
    if(Phi.N<30) for(uint k:Phi) cout <<"    " <<k <<": " <<G.O(k).description <<"(" <<G.O(k).vars <<")" <<endl;
  }
}

ofstream logFile;

void SubGraphProblem::optim(int verbose) {
  uint m=0;
  for(uint i:X) m += G.V(i).x_dim;
  arr x(m);
  m=0;
  for(uint i:X) {
    uint d = G.V(i).x_dim;
    if(d) x({m, m+d-1}) = G.V(i).value;
    m+=d;
  }
  CHECK_EQ(m, x.N, "");

  checkStructure(x);

  //    Configuration::setJointStateCount=0;
  rai::timerStart();
#if 0
  ModGraphProblem Gsel(*this);
  Conv_Graph_MathematicalProgram C(Gsel);
#else
  Conv_Graph_MathematicalProgram C(*this);
#endif
  C.reportProblem(logFile);
  arr dual;
  OptConstrained opt(x, dual, C); //rai::MAX(verbose-2, 0));
  opt.L.logFile = &logFile;
  opt.run();
  //    opt.newton.evals;
  double runTime = rai::timerRead();
  if(verbose>0) {
    cout <<"** optimization time=" <<runTime <<endl;
  }

  //read out return values
  f = sos = eq = ineq = 0.;
  conflictSet.clear();
  for(uint k:Phi) {
    ObjectiveNode& o = G.O(k);
    if(o.type==OT_sos) sos += rai::sqr(o.value);
    else if(o.type==OT_eq) eq += fabs(o.value);
    else if(o.type==OT_ineq && o.value>0.) ineq += o.value;

    if(o.type==OT_eq && fabs(o.value)>.5) conflictSet.append(k);
    if(o.type==OT_ineq && o.value>.5) conflictSet.append(k);
  }
  if(eq+ineq>1.) {
    feasible = false;
    f = NAN;
  } else {
    feasible = true;
    f = sos;
  }

  if(verbose>0) {
    cout <<"   sos=" <<sos <<" ineq=" <<ineq <<" eq=" <<eq <<" #conflicts=" <<conflictSet.N <<endl;
  }

//  checkJacobianCP(C, x, 1e-4);
//  rai::wait();
}

void SubGraphProblem::getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes) {
  if(!!variableDimensions) {
    variableDimensions.resize(X.N);
    for(uint i=0; i<X.N; i++) variableDimensions(i) = G.V(X(i)).x_dim;
  }

  if(!!featureVariables) featureVariables.resize(Phi.N);
  if(!!featureTypes) featureTypes.resize(Phi.N);
  for(uint k=0; k<Phi.N; k++) {
    if(!!featureVariables) {
      featureVariables(k) = G.O(Phi(k)).vars;
      for(int& j:featureVariables(k)) j = Gindex2SubIndex(j);
    }
    if(!!featureTypes) featureTypes(k) = G.O(Phi(k)).type;
  }
}

void SubGraphProblem::getSemantics(StringA& varNames, StringA& phiNames) {
  if(!!varNames) {
    varNames.resize(X.N);
    for(uint i=0; i<X.N; i++) varNames(i) = G.V(X(i)).description;
  }

  if(!!phiNames) phiNames.resize(Phi.N);
  for(uint k=0; k<Phi.N; k++) {
    if(!!phiNames) phiNames(k) = G.O(Phi(k)).description;
  }
}

void SubGraphProblem::phi(arr& phi, arrA& J, arrA& H, const arr& x) {
  G.G.setPartialX(X, x);
  G.G.getPartialPhi(phi, J, H, Phi);

  //kill columns of J that refer to non-variables
  if(!!J) {
    CHECK_EQ(J.N, Phi.N, "");
    for(uint Jk=0; Jk<J.N; Jk++) {
      uint k = Phi(Jk);
      //what are the original variables that k refers to?
      intA Gvars = G.O(k).vars;
      //what are the dimensions of these original variables?
      uintA kdim = consts<uint>(0, Gvars.N+1);
      for(uint j=0; j<Gvars.N; j++) {
        kdim(j+1) = kdim(j);
        if(Gvars(j)>=0) kdim(j+1) += G.V(Gvars(j)).x_dim;
      }
      //check if some of the original variables map to not-included-here -> remove columns from J
      for(uint j=Gvars.N; j--;) {
        if(Gvars(j)>=0 && Gindex2SubIndex(Gvars(j))<0) {
          J(Jk).remove(kdim(j), kdim(j+1)-kdim(j)); //delete the columns that correspond to the prefix!!
        }
      }
    }
  }

  //store this query and evaluation
  uint m=0;
  for(uint i:X) {
    uint d = G.V(i).x_dim;
    if(d) G.V(i).value = x({m, m+d-1});
    m += d;
  }
  CHECK_EQ(m, x.N, "");
  if(!!phi) {
    CHECK_EQ(phi.N, Phi.N, "");
    for(uint k=0; k<Phi.N; k++) {
      G.O(Phi(k)).value = phi(k);
    }
  }
}

//===========================================================================

BacktrackingGraphOptimization::BacktrackingGraphOptimization(GraphProblem& _G) : G(_G) {
}

int BacktrackingGraphOptimization::chooseNextVariableToAssign(const uintA& Y) {
  uint n = G.V.N;
  for(uint i=n; i--;) {
//  for(uint i=0;i<n;i++){
    if(!Y.contains(i)) return i;
  }
  return -1;
}

uintA BacktrackingGraphOptimization::getVariablesForObjectives(uintA& O) {
  uintA X;
  for(uint k:O) for(int i:G.O(k).vars) if(i>=0) X.setAppendInSorted(i);
  return X;
}

void BacktrackingGraphOptimization::evaluate(const arr& x) {
  uintA X;
  X.setStraightPerm(G.V.N);
  SubGraphProblem G_X(G, X, {});
  G_X.phi(NoArr, NoArrA, NoArrA, x);
}

bool BacktrackingGraphOptimization::run() {
  uintA Y; //assigned variables
  uintA X; //active variables
  f_low=0.;

  logFile.open("z.optim");

  for(;;) {
    int next = chooseNextVariableToAssign(Y);
    if(next<0) return true; //success: no next variable
    X = {uint(next)};
    cout <<"** BGO: solve (" <<X <<"|" <<Y <<")" <<endl;
    SubGraphProblem G_XY(G, X, Y);
    G_XY.optim();
    while(!G_XY.feasible) {
      if(!Y.N) {
        infeasibleSubset = X;
        conflictSet = G_XY.conflictSet;
        return false;
      }
      /*
      SubGraphProblem G_X(G, X, {});
      G_X.optim();
      f_low = rai::MAX(f_low, G_X.f);
      if(!G_X.feasible){
        infeasibleSubset = X;
        conflictSet = G_X.conflictSet;
        return false;
      }
      */
      uintA pi_C = getVariablesForObjectives(G_XY.conflictSet);
      setMinusSorted(pi_C, X);
      if(!pi_C.N) {
        uint prev = Y.popLast();
        X.setAppendInSorted(prev);
      } else {
        for(uint i:pi_C) {
          Y.removeValue(i, false);
          X.setAppendInSorted(i);
        }
      }
      CHECK(X.isSorted(), "");
      for(uint i:Y) CHECK(!X.containsInSorted((i)), "");
      G_XY.reset(X, Y);
      G_XY.optim();
    }
    Y.append(X);
  }

  logFile.close();

  return true;
}

bool BacktrackingGraphOptimization::runFull() {
  uintA X;
  X.setStraightPerm(G.V.N);
  SubGraphProblem G_X(G, X, {});
  G_X.optim();
  return G_X.feasible;
}
