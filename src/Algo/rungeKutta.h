/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

namespace rai {

//----- Runge-Kutta
/// standard Runge-Kutta 4
void rk4(arr& x, const arr& x0, const fct& f, double dt);
/// same for second order diff equation
//void rk4dd(arr& x1, arr& v1, const arr& x0, const arr& v0,
//           void (*ddf)(arr& xdd, const arr& x, const arr& v),
//           double dt);
arr rk4_2ndOrder(const arr& x0, const std::function<arr (const arr&, const arr&)>& f, double dt);

/** RK with discrete event localization (zero-crossing detection): the
    function sf computes some double-valued indicators. If one of
    these indicators crosses zero this is interpreted as a
    discontinuity in the dynamics. The algorithm iteratively tries to
    find the zero-crossing point up to a tolerance tol (measured in
    time). The routine returns false for no-switching and true and the
    executed time step dt in the case of zero-crossing */
bool rk4_switch(arr& x1, arr& s1, const arr& x0, const arr& s0,
                void (*df)(arr& xd, const arr& x),
                void (*sf)(arr& s, const arr& x),
                double& dt, double tol);
/// same for 2nd order DEs
bool rk4dd_switch(arr& x1, arr& v1, arr& s1, const arr& x0, const arr& v0, const arr& s0,
                  void (*ddf)(arr& xdd, const arr& x, const arr& v),
                  void (*sf)(arr& s, const arr& x, const arr& v),
                  double& dt, double tol);

} //end namespace rai
