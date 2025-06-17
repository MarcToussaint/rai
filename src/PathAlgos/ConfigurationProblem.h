/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Kin/kin.h"
#include "../KOMO/objective.h"
#include "../Optim/NLP.h"

#include <unordered_map>

struct ConfigurationProblem;

struct QueryResult {
  //collision features
  uintA collisions;
  double totalCollision=0.;
  bool isFeasible=true;

  //optional a 3D coordinate for display
  arr disp3d;

  void write(ostream& os) const;
  void writeDetails(ostream& os, const ConfigurationProblem& P, double margin=0.) const;
};
stdOutPipe(QueryResult)

struct ConfigurationProblem {
  shared_ptr<rai::Configuration> C;
  arr limits;
  uintA sphericalCoordinates;

  //what collisions are evaluated?
  bool useBroadCollisions;
  uintA collisionPairs;
  double collisionTolerance;

  //user info
  int verbose=0;
  uint evals=0;
  double queryTime=0.;

  ConfigurationProblem(shared_ptr<rai::Configuration> _C, bool _useBroadCollisions=true, double _collisionTolerance=1e-3, int _verbose=0);

  void setExplicitCollisionPairs(const StringA& _collisionPairs);

  shared_ptr<QueryResult> query(const arr& x);
};
