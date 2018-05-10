/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "cma.h"
#include "algos.h"

CMA::CMA() {
}

CMA::~CMA() {
}

void CMA::run() {
  //initialize
  arr pc,ps;
  m.resize(dim);
  pc.resize(dim);
  ps.resize(dim);
  m.setZero();
  pc.setZero();
  ps.setZero();
  C = eye(dim);
}

