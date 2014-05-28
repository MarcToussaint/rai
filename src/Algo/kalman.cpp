/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "kalman.h"

void Kalman::stepPredict(const arr& A, const arr& a, const arr& Q){
  b_mean = A*b_mean + a;
  b_var  = Q + A*b_var*~A;
}

void Kalman::stepObserve(const arr& y, const arr& C, const arr& c, const arr& W){
  arr Winv, Sinv, Ct;
  inverse_SymPosDef(Winv, W);
  inverse_SymPosDef(Sinv, b_var);
  transpose(Ct, C);
  b_var  = inverse_SymPosDef(Ct * Winv * C + Sinv);
  b_mean = b_var * (Ct * (Winv * (y-c)) + Sinv*b_mean);
}

void Kalman::step(const arr& A, const arr& a, const arr& Q, const arr& y, const arr& C, const arr& c, const arr& W){
  stepPredict(A, a, Q);
  stepObserve(y, C, c, W);
}

