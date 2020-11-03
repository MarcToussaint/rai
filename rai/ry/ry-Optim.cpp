/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "../ry/types.h"
#include "../Optim/NLP_Factory.h"
#include "../Optim/solver.h"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

void init_Optim(pybind11::module& m) {

  pybind11::class_<NLP_Factory, std::shared_ptr<NLP_Factory>>(m, "NLP_Factory", "An interface to specify an NLP")

      .def(pybind11::init<>())

      .def("setDimension", &NLP_Factory::setDimension)
      .def("setFeatureTypes", &NLP_Factory::setFeatureTypes)
      .def("setBounds", &NLP_Factory::setBounds)
      .def("setEvalCallback", &NLP_Factory::setEvalCallback2)


  .def("testCallingEvalCallback", [](std::shared_ptr<NLP_Factory>& self, const arr& x){
    arr y, J;
    self->evaluate(y, J, x);
    return std::tuple<arr,arr>(y, J);
  } )

  ;

  pybind11::class_<MathematicalProgram, shared_ptr<MathematicalProgram>>(m, "MathematicalProgram");

  pybind11::class_<NLP_Solver, std::shared_ptr<NLP_Solver>>(m, "NLP_Solver", "An interface to portfolio of solvers")

      .def(pybind11::init<>())
//      .def("setProblem", &NLP_Solver::setProblem)
      .def("setProblem", [](std::shared_ptr<NLP_Solver>& self, std::shared_ptr<NLP_Factory>& P){
    self->setProblem(*P);
      } )
      .def("setSolver", &NLP_Solver::setSolver)

      .def("getOptions", &NLP_Solver::getOptions)
      .def("setOptions", &NLP_Solver::setOptions)
      .def("setLogging", &NLP_Solver::setLogging)
      .def("solve", &NLP_Solver::solve)

      .def("getLog_x", &NLP_Solver::getLog_x)
      .def("getLog_costs", &NLP_Solver::getLog_costs)
      .def("getLog_phi", &NLP_Solver::getLog_phi)
      .def("getLog_J", &NLP_Solver::getLog_J)

      ;

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
  .export_values();
}

#endif
