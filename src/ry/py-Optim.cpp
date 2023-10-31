/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Optim/NLP_Factory.h"
#include "../Optim/NLP_Solver.h"
#include "../KOMO/opt-benchmarks.h"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

void init_Optim(pybind11::module& m) {

  //===========================================================================

  pybind11::class_<NLP, shared_ptr<NLP>> __mp(m, "NLP", "Representation of a Nonlinear Mathematical Program");
  __mp

  .def("evaluate", [](std::shared_ptr<NLP>& self, const arr& x) {
    arr phi, J;
    self->evaluate(phi, J, x);
    return std::tuple<arr, arr>(phi, J);
  },
  "query the NLP at a point $x$; returns the tuple $(phi,J)$, which is the feature vector and its Jacobian; features define cost terms, sum-of-square (sos) terms, inequalities, and equalities depending on 'getFeatureTypes'"
      )

  .def("getFeatureTypes", [](std::shared_ptr<NLP>& self) {
    return Array2vec<ObjectiveType>(self->featureTypes);
  },
  "features (entries of $phi$) can be of one of (ry.OT.f, ry.OT.sos, ry.OT.ineq, ry.OT.eq), which means (cost, sum-of-square, inequality, equality). The total cost $f(x)$ is the sum of all f-terms plus sum-of-squares of sos-terms."
      )

  .def("getDimension", &NLP::getDimension, "return the dimensionality of $x$")

  .def("getBounds", [](std::shared_ptr<NLP>& self) {
    arr lo, up;
    self->getBounds(lo, up);
    return std::tuple<arr, arr>(lo, up);
  },
  "returns the tuple $(b_{lo},b_{up})$, where both vectors are of same dimensionality of $x$ (or size zero, if there are no bounds)")

  .def("getInitializationSample",
       &NLP::getInitializationSample,
       "returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible",
       pybind11::arg("previousOptima") = arr()
      )

  .def("getFHessian",  [](std::shared_ptr<NLP>& self, const arr& x) {
    arr H;
    self->getFHessian(H, x);
    return H;
  },
  "returns Hessian of the sum of $f$-terms"
      )

  .def("report",  [](std::shared_ptr<NLP>& self, int verbose) {
    rai::String str;
    self->report(str, verbose);
    return std::string(str.p);
  },
  "displays semantic information on the last query"
      )

  ;

  //===========================================================================

  pybind11::class_<NLP_Factory, std::shared_ptr<NLP_Factory>>(m, "NLP_Factory", __mp)

      .def(pybind11::init<>())

      .def("setDimension", &NLP_Factory::setDimension)
      .def("setFeatureTypes", &NLP_Factory::setFeatureTypes)
      .def("setBounds", &NLP_Factory::setBounds)
      .def("setEvalCallback", &NLP_Factory::setEvalCallback2)

  .def("testCallingEvalCallback", [](std::shared_ptr<NLP_Factory>& self, const arr& x) {
    arr y, J;
    self->evaluate(y, J, x);
    return std::tuple<arr, arr>(y, J);
  })

  ;

  //===========================================================================

  //  pybind11::module_ mBench = m.def_submodule("nlp_benchmark", "ry submodule to define optimization benchmarks");
//  pybind11::class_<NLP, shared_ptr<NLP>>(m, "NLP")

  pybind11::class_<OptBench_InvKin_Endeff, std::shared_ptr<OptBench_InvKin_Endeff>>(m, "OptBenchmark_InvKin_Endeff")
      .def(pybind11::init<const char*, bool>())
      .def("get", &OptBench_InvKin_Endeff::get)
      ;

  pybind11::class_<OptBench_Skeleton_Pick, std::shared_ptr<OptBench_Skeleton_Pick>>(m, "OptBench_Skeleton_Pick")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_Pick::get)
      ;

  pybind11::class_<OptBench_Skeleton_Handover, std::shared_ptr<OptBench_Skeleton_Handover>>(m, "OptBench_Skeleton_Handover")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_Handover::get)
      ;

  pybind11::class_<OptBench_Skeleton_StackAndBalance, std::shared_ptr<OptBench_Skeleton_StackAndBalance>>(m, "OptBench_Skeleton_StackAndBalance")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_StackAndBalance::get)
      ;

  //===========================================================================

  pybind11::class_<rai::OptOptions, std::shared_ptr<rai::OptOptions>>(m, "NLP_SolverOptions", "solver options")

      .def(pybind11::init<>())
    #define MEMBER(type, name, x) .def("set_" #name, &rai::OptOptions::set_##name)
    MEMBER(int, verbose, 1)
    MEMBER(double, stopTolerance, 1e-2)
    MEMBER(double, stopFTolerance, -1.)
    MEMBER(double, stopGTolerance, -1.)
    MEMBER(int, stopEvals, 1000)
    MEMBER(double, maxStep, .2)
    MEMBER(double, damping, 1.)
    MEMBER(double, stepInc, 1.5)
    MEMBER(double, stepDec, .5)
    MEMBER(double, wolfe, .01)
    MEMBER(double, muInit, 1.)
    MEMBER(double, muInc, 5.)
    MEMBER(double, muMax, 1e4)
    MEMBER(double, muLBInit, .1)
    MEMBER(double, muLBDec, .2)
    #undef MEMBER

      .def("dict", [](std::shared_ptr<rai::OptOptions>& self) {
        return graph2dict(rai::Graph{
                    #define MEMBER(type, name, x) {#name, self->name},
                    MEMBER(int, verbose, 1)
                    MEMBER(double, stopTolerance, 1e-2)
                    MEMBER(double, stopFTolerance, -1.)
                    MEMBER(double, stopGTolerance, -1.)
                    MEMBER(int, stopEvals, 1000)
                    MEMBER(double, maxStep, .2)
                    MEMBER(double, damping, 1.)
                    MEMBER(double, stepInc, 1.5)
                    MEMBER(double, stepDec, .5)
                    MEMBER(double, wolfe, .01)
                    MEMBER(double, muInit, 1.)
                    MEMBER(double, muInc, 5.)
                    MEMBER(double, muMax, 1e4)
                    MEMBER(double, muLBInit, .1)
                    MEMBER(double, muLBDec, .2)
                    #undef MEMBER
                          });

        })

      ;

  //===========================================================================

  pybind11::class_<NLP_Solver, std::shared_ptr<NLP_Solver>>(m, "NLP_Solver", "An interface to portfolio of solvers")

      .def(pybind11::init<>())
      .def(pybind11::init<const shared_ptr<NLP>&, int>(), "",
           pybind11::arg("problem"),
           pybind11::arg("verbose")=0
           )

      .def("setProblem", &NLP_Solver::setProblem)
      .def("setSolver", &NLP_Solver::setSolver)

      .def("setTracing", &NLP_Solver::setTracing)
      .def("solve", &NLP_Solver::solve, "", pybind11::arg("resampleInitialization")=-1)

      .def("getTrace_x", &NLP_Solver::getTrace_x)
      .def("getTrace_costs", &NLP_Solver::getTrace_costs)
      .def("getTrace_phi", &NLP_Solver::getTrace_phi)
      .def("getTrace_J", &NLP_Solver::getTrace_J)

      .def("reportLagrangeGradients", [](std::shared_ptr<NLP_Solver>& self, const StringA& featureNames){
        rai::Graph R = self->reportLangrangeGradients(featureNames);
        return graph2dict(R);
       })

      .def("getOptions", [](std::shared_ptr<NLP_Solver>& self){ return self->opt; } )
      .def("setOptions", [](std::shared_ptr<NLP_Solver>& self
         #define MEMBER(type, name, x) ,type name
         MEMBER(int, verbose, 1)
         MEMBER(double, stopTolerance, 1e-2)
         MEMBER(double, stopFTolerance, -1.)
         MEMBER(double, stopGTolerance, -1.)
         MEMBER(int, stopEvals, 1000)
         MEMBER(double, maxStep, .2)
         MEMBER(double, damping, 1.)
         MEMBER(double, stepInc, 1.5)
         MEMBER(double, stepDec, .5)
         MEMBER(double, wolfe, .01)
         MEMBER(double, muInit, 1.)
         MEMBER(double, muInc, 5.)
         MEMBER(double, muMax, 1e4)
         MEMBER(double, muLBInit, .1)
         MEMBER(double, muLBDec, .2)
         #undef MEMBER
           ){
    self->opt
    #define MEMBER(type, name, x) .set_##name(name)
    MEMBER(int, verbose, 1)
    MEMBER(double, stopTolerance, 1e-2)
    MEMBER(double, stopFTolerance, -1.)
    MEMBER(double, stopGTolerance, -1.)
    MEMBER(int, stopEvals, 1000)
    MEMBER(double, maxStep, .2)
    MEMBER(double, damping, 1.)
    MEMBER(double, stepInc, 1.5)
    MEMBER(double, stepDec, .5)
    MEMBER(double, wolfe, .01)
    MEMBER(double, muInit, 1.)
    MEMBER(double, muInc, 5.)
    MEMBER(double, muMax, 1e4)
    MEMBER(double, muLBInit, .1)
    MEMBER(double, muLBDec, .2)
    #undef MEMBER
        ;
        return self;
        }, "set solver options"
    #define MEMBER(type, name, x) , pybind11::arg(#name) = x
    MEMBER(int, verbose, 1)
    MEMBER(double, stopTolerance, 1e-2)
    MEMBER(double, stopFTolerance, -1.)
    MEMBER(double, stopGTolerance, -1.)
    MEMBER(int, stopEvals, 1000)
    MEMBER(double, maxStep, .2)
    MEMBER(double, damping, 1.)
    MEMBER(double, stepInc, 1.5)
    MEMBER(double, stepDec, .5)
    MEMBER(double, wolfe, .01)
    MEMBER(double, muInit, 1.)
    MEMBER(double, muInc, 5.)
    MEMBER(double, muMax, 1e4)
    MEMBER(double, muLBInit, .1)
    MEMBER(double, muLBDec, .2)
    #undef MEMBER
    )

      ;

  //===========================================================================

  pybind11::class_<SolverReturn, std::shared_ptr<SolverReturn>>(m, "SolverReturn", "return of nlp solve call")

      .def(pybind11::init<>())
      .def_readwrite("x", &SolverReturn::x)
      .def_readwrite("evals", &SolverReturn::evals)
      .def_readwrite("time", &SolverReturn::time)
      .def_readwrite("feasible", &SolverReturn::feasible)
      .def_readwrite("done", &SolverReturn::done)
      .def_readwrite("f", &SolverReturn::f)
      .def_readwrite("sos", &SolverReturn::sos)
      .def_readwrite("ineq", &SolverReturn::ineq)
      .def_readwrite("eq", &SolverReturn::eq)
      .def("__str__", [](std::shared_ptr<SolverReturn>& self) {  rai::String str;  str <<(*self);  return std::string(str.p); } )

  .def("dict", [](std::shared_ptr<SolverReturn>& self) {
    return graph2dict(rai::Graph{
      {"evals", self->evals},
      {"time", self->time},
      {"done", self->done},
      {"feasible", self->feasible},
      {"f", self->f},
      {"sos", self->sos},
      {"ineq", self->ineq},
      {"eq", self->eq},
    });
  })

  ;

  //===========================================================================

#undef ENUMVAL
#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<NLP_SolverID>(m, "NLP_SolverID")
  ENUMVAL(NLPS, gradientDescent) ENUMVAL(NLPS, rprop) ENUMVAL(NLPS, LBFGS) ENUMVAL(NLPS, newton)
  ENUMVAL(NLPS, augmentedLag) ENUMVAL(NLPS, squaredPenalty) ENUMVAL(NLPS, logBarrier) ENUMVAL(NLPS, singleSquaredPenalty)
  ENUMVAL(NLPS, NLopt) ENUMVAL(NLPS, Ipopt) ENUMVAL(NLPS, Ceres)
  .export_values();

  pybind11::enum_<ObjectiveType>(m, "OT")
  ENUMVAL(OT, none)
  ENUMVAL(OT, f)
  ENUMVAL(OT, sos)
  ENUMVAL(OT, ineq)
  ENUMVAL(OT, eq)
  ENUMVAL(OT, ineqB)
  ENUMVAL(OT, ineqP)
  .export_values();

#undef ENUMVAL

}

#endif
