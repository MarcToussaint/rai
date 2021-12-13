/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "../ry/types.h"
#include "../Optim/MP_Factory.h"
#include "../Optim/MP_Solver.h"
#include "../KOMO/opt-benchmarks.h"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

void init_Optim(pybind11::module& m) {

  //===========================================================================

  pybind11::class_<MathematicalProgram, shared_ptr<MathematicalProgram>> __mp(m, "MathematicalProgram");
  __mp

  .def("evaluate", [](std::shared_ptr<MathematicalProgram>& self, const arr& x){
    arr phi, J;
    self->evaluate(phi, J, x);
    return std::tuple<arr,arr>(phi, J);
  },
  "query the MP at a point $x$; returns the tuple $(phi,J)$, which is the feature vector and its Jacobian; features define cost terms, sum-of-square (sos) terms, inequalities, and equalities depending on 'getFeatureTypes'"
  )

  .def("getFeatureTypes", &MathematicalProgram::getFeatureTypes,
       "features (entries of $phi$) can be of one of (ry.OT.f, ry.OT.sos, ry.OT.ineq, ry.OT.eq), which means (cost, sum-of-square, inequality, equality). The total cost $f(x)$ is the sum of all f-terms plus sum-of-squares of sos-terms."
  )

  .def("getDimension", &MathematicalProgram::getDimension, "return the dimensionality of $x$")

  .def("getBounds", [](std::shared_ptr<MathematicalProgram>& self){
    arr lo,up;
    self->getBounds(lo, up);
    return std::tuple<arr,arr>(lo, up);
  },
  "returns the tuple $(b_{lo},b_{up})$, where both vectors are of same dimensionality of $x$ (or size zero, if there are no bounds)")

  .def("getInitializationSample",
       &MathematicalProgram::getInitializationSample,
       "returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible",
       pybind11::arg("previousOptima") = arr()
      )

  .def("getFHessian",  [](std::shared_ptr<MathematicalProgram>& self, const arr& x){
    arr H;
    self->getFHessian(H, x);
    return H;
  },
  "returns Hessian of the sum of $f$-terms"
  )

  .def("report",  [](std::shared_ptr<MathematicalProgram>& self, int verbose){
    rai::String str;
    self->report(str, verbose);
    return std::string(str.p);
  },
  "displays semantic information on the last query"
  )

  ;

  //===========================================================================

  pybind11::class_<MP_Factory, std::shared_ptr<MP_Factory>>(m, "MP_Factory", __mp)

      .def(pybind11::init<>())

      .def("setDimension", &MP_Factory::setDimension)
      .def("setFeatureTypes", &MP_Factory::setFeatureTypes)
      .def("setBounds", &MP_Factory::setBounds)
      .def("setEvalCallback", &MP_Factory::setEvalCallback2)

  .def("testCallingEvalCallback", [](std::shared_ptr<MP_Factory>& self, const arr& x){
    arr y, J;
    self->evaluate(y, J, x);
    return std::tuple<arr,arr>(y, J);
  } )

  ;

  //===========================================================================

  //  pybind11::module_ mBench = m.def_submodule("nlp_benchmark", "ry submodule to define optimization benchmarks");
//  pybind11::class_<MathematicalProgram, shared_ptr<MathematicalProgram>>(m, "MathematicalProgram")

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

  pybind11::class_<MP_Solver, std::shared_ptr<MP_Solver>>(m, "MP_Solver", "An interface to portfolio of solvers")

      .def(pybind11::init<>())
//      .def("setProblem", &MP_Solver::setProblem)
      .def("setProblem", &MP_Solver::setProblem)
      .def("setSolver", &MP_Solver::setSolver)

      .def("setTracing", &MP_Solver::setTracing)
      .def("solve", &MP_Solver::solve)

      .def("getTrace_x", &MP_Solver::getTrace_x)
      .def("getTrace_costs", &MP_Solver::getTrace_costs)
      .def("getTrace_phi", &MP_Solver::getTrace_phi)
      .def("getTrace_J", &MP_Solver::getTrace_J)

      ;

  //===========================================================================

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<MP_SolverID>(m, "MP_SolverID")
      ENUMVAL(MPS, gradientDescent) ENUMVAL(MPS, rprop) ENUMVAL(MPS, LBFGS) ENUMVAL(MPS, newton)
      ENUMVAL(MPS, augmentedLag) ENUMVAL(MPS, squaredPenalty) ENUMVAL(MPS, logBarrier) ENUMVAL(MPS, singleSquaredPenalty)
      ENUMVAL(MPS, NLopt) ENUMVAL(MPS, Ipopt) ENUMVAL(MPS, Ceres)
      .export_values();


  pybind11::enum_<ObjectiveType>(m, "OT")
  ENUMVAL(OT, none)
  ENUMVAL(OT, f)
  ENUMVAL(OT, sos)
  ENUMVAL(OT, ineq)
  ENUMVAL(OT, eq)
  .export_values();

#undef ENUMVAL

}

#endif
