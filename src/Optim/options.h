/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/util.h"

namespace rai {

enum OptMethod { M_none=0,
		 M_gradientDescent, M_rprop, M_LBFGS, M_newton,
		 M_augmentedLag, M_squaredPenalty, M_logBarrier, M_singleSquaredPenalty,
		 M_slackGN,
		 M_NLopt, M_Ipopt, M_Ceres };


struct OptOptions {
  RAI_PARAM("opt/", int, verbose, 1)
  RAI_PARAM("opt/", double, stopTolerance, 1e-2)
  RAI_PARAM("opt/", double, stopFTolerance, -1.)
  RAI_PARAM("opt/", double, stopGTolerance, -1.)
  RAI_PARAM("opt/", int,    stopEvals, 1000)
  RAI_PARAM("opt/", int,    stopInners, 1000)
  RAI_PARAM("opt/", int,    stopOuters, 1000)
  RAI_PARAM("opt/", int,    stopLineSteps, 10)
  RAI_PARAM("opt/", int,    stopTinySteps, 4)
  RAI_PARAM("opt/", double, stepInit, 1.)
  RAI_PARAM("opt/", double, stepMin, -1.)
  RAI_PARAM("opt/", double, stepMax, .2)
  RAI_PARAM("opt/", double, stepInc, 1.5)
  RAI_PARAM("opt/", double, stepDec, .5)
  RAI_PARAM("opt/", double, damping, 1.)
  RAI_PARAM("opt/", double, wolfe, .01)
  RAI_PARAM("opt/", double, muInit, 1.)
  RAI_PARAM("opt/", double, muInc, 5.)
  RAI_PARAM("opt/", double, muMax, 1e4)
  RAI_PARAM("opt/", double, muLBInit, .1)
  RAI_PARAM("opt/", double, muLBDec, .2)
  RAI_PARAM("opt/", double, lambdaMax, -1.)
  RAI_PARAM("opt/", double, interiorPadding, 1e-2)
  RAI_PARAM("opt/", bool,   boundedNewton, true)
  RAI_PARAM_ENUM("opt/", OptMethod, method, M_augmentedLag)
//  void write(std::ostream& os) const;
};
//stdOutPipe(OptOptions)

OptOptions& globalOptOptions();

} //namespace

#define DEFAULT_OPTIONS (rai::globalOptOptions())

