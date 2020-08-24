/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kalman.h"

void Kalman::stepPredict(const arr& A, const arr& a, const arr& Q) {
  b_mean = A*b_mean + a;
  b_var  = Q + A*b_var*~A;
}

void Kalman::stepObserve(const arr& y, const arr& C, const arr& c, const arr& W) {
  arr Winv, Sinv, Ct;
  inverse_SymPosDef(Winv, W);
  inverse_SymPosDef(Sinv, b_var);
  transpose(Ct, C);
  b_var  = inverse_SymPosDef(Ct * Winv * C + Sinv);
  b_mean = b_var * (Ct * (Winv * (y-c)) + Sinv*b_mean);
}

void Kalman::step(const arr& A, const arr& a, const arr& Q, const arr& y, const arr& C, const arr& c, const arr& W) {
  stepPredict(A, a, Q);
  stepObserve(y, C, c, W);
}

