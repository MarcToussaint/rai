/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
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
      .def("setProblem", &rai::PathFinder::setProblem, "", pybind11::arg("Configuration"), pybind11::arg("starts"), pybind11::arg("goals"), pybind11::arg("collisionTolerance")=1e-4)
      .def("setExplicitCollisionPairs", &rai::PathFinder::setExplicitCollisionPairs, "only after setProblem", pybind11::arg("collisionPairs"))
      .def("solve", &rai::PathFinder::solve, "")
      .def("get_resampledPath", &rai::PathFinder::get_resampledPath, "")

      ;

}

#endif
