/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

//-- a basic message type for communication with the PR2 controller
struct CtrlMsg {
  arr q, qdot, fL, fR, J_ft_invL, J_ft_invR;
  arr P_compliance;
  arr Kp, Kd, Ki, u_bias, KiFTL, KiFTR, fL_offset, fR_offset, fL_err, fR_err;
  double velLimitRatio, effLimitRatio, intLimitRatio, fL_gamma, fR_gamma, qd_filt;
  CtrlMsg():Kp(arr{1.}), Kd(arr{1.}), Ki(arr{0.}), u_bias(arr{0.}), fL_offset(zeros(6)), fR_offset(zeros(6)), velLimitRatio(1.), effLimitRatio(1.), intLimitRatio(0.1),
    fL_gamma(0.), fR_gamma(0.), qd_filt(0.97) {}
  CtrlMsg(const arr& q, const arr& qdot,
          const arr& fL, const arr& fR, const arr& u_bias, const arr& fL_err, const arr& fR_err)
    :q(q), qdot(qdot), fL(fL), fR(fR), u_bias(u_bias), fL_err(fL_err), fR_err(fR_err) {}
};
