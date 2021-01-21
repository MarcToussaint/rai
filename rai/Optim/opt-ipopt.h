/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "MathematicalProgram.h"

struct IpoptInterface {
//  MathematicalProgram_Traced P;
  MathematicalProgram& P;

  IpoptInterface(MathematicalProgram& P) : P(P) {}

  arr solve(const arr& x_init=NoArr);
};

