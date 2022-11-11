/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "ry-Config.h"
#include "../Control/control.h"

void init_CtrlSet(pybind11::module& m) {
  pybind11::class_<CtrlSet, std::shared_ptr<CtrlSet>>(m, "CtrlSet", "A set of control objectives, which define a control mode and can be passed to a CtrlSolver")

      .def(pybind11::init<>())
      .def("addObjective", &CtrlSet::addObjective,
           pybind11::arg("feature"),
           pybind11::arg("type"),
           pybind11::arg("transientStep")=-1.
           )
      .def("add_qControlObjective", &CtrlSet::add_qControlObjective)

  .def("canBeInitiated", [](std::shared_ptr<CtrlSet>& self, std::shared_ptr<CtrlSolver>& solver) {
      return self->canBeInitiated(solver->komo.pathConfig);
  })

  .def("isConverged", [](std::shared_ptr<CtrlSet>& self, std::shared_ptr<CtrlSolver>& solver) {
      return self->isConverged(solver->komo.pathConfig);
  })
  ;

  pybind11::class_<CtrlObjective, shared_ptr<CtrlObjective>>(m, "CtrlObjective");

}

void init_CtrlSolver(pybind11::module& m) {
  pybind11::class_<CtrlSolver, std::shared_ptr<CtrlSolver>>(m, "CtrlSolver", "A control solver")

      .def(pybind11::init<rai::Configuration&, double, uint>(),
           pybind11::arg("configuration"),
           pybind11::arg("tau"),
           pybind11::arg("order"))
      .def("set", &CtrlSolver::set)
      .def("update", &CtrlSolver::update)
      //      void report(ostream& os=std::cout);
      .def("solve", &CtrlSolver::solve)
      ;
}


#endif
