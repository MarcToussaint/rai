/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../PathAlgos/RRT_PathFinder.h"

void init_PathAlgos(pybind11::module& m) {
  pybind11::class_<rai::PathFinder, std::shared_ptr<rai::PathFinder>>(m, "PathFinder", "todo doc")

  .def(pybind11::init<>())
  .def("setProblem", &rai::PathFinder::setProblem, "", pybind11::arg("Configuration"), pybind11::arg("starts"), pybind11::arg("goals") )
  .def("setExplicitCollisionPairs", &rai::PathFinder::setExplicitCollisionPairs, "", pybind11::arg("collisionPairs") )
  .def("solve", &rai::PathFinder::solve, "" )

  ;

}

#endif
