/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "optimization.h"
#include "Graph_Problem.h"
#include "convert.h"
#include "constrained.h"

struct VariableNode {
  uint x_index;
  uint x_dim;
  uintA objs;
  rai::String description;
  arr value;
};

struct ObjectiveNode {
  ObjectiveType type;
  uint phi_index;
  intA vars;
  rai::String description;
  double value;
};

struct GraphProblem_Structure {
  GraphProblem& G;
  rai::Array<VariableNode> V;
  rai::Array<ObjectiveNode> O;

  GraphProblem_Structure(GraphProblem& _G);
};

//typedef rai::Array<VariableNode*> VarL;
//typedef rai::Array<ObjectiveNode*> ObjL;

struct SubGraphProblem : GraphProblem {
  GraphProblem_Structure& G;

  //subselection of indices
  uintA X, Y, Phi;
  //mapping of indices
  intA Gindex2SubIndex;

  //return values
  arr x;
  bool feasible;
  double f, sos, eq, ineq;
  uintA conflictSet;

  SubGraphProblem(GraphProblem_Structure& G, const uintA& _X, const uintA& _Y);

  void reset(const uintA& _X, const uintA& _Y);

  void optim(int verbose=1);

  //-- graph problem of the subgraph
  virtual void getStructure(uintA& variableDimensions, intAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);
  virtual void getSemantics(StringA& varNames, StringA& phiNames);
};

struct BacktrackingGraphOptimization {
  GraphProblem_Structure G;

  //return values
  uintA infeasibleSubset;
  uintA conflictSet;
  double f_low;

  BacktrackingGraphOptimization(GraphProblem& _G);

  int chooseNextVariableToAssign(const uintA& Y);

  uintA getVariablesForObjectives(uintA& O);

  void evaluate(const arr& x);

  bool run();

  bool runFull();
};
