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
  pybind11::class_<rai::RRT_PathFinder, std::shared_ptr<rai::RRT_PathFinder>>(m, "RRT_PathFinder", "todo doc")

      .def(pybind11::init<>())
      .def("setProblem", &rai::RRT_PathFinder::setProblem, "", pybind11::arg("Configuration"))
      .def("setStartGoal", &rai::RRT_PathFinder::setStartGoal, "", pybind11::arg("starts"), pybind11::arg("goals"))
      .def("setExplicitCollisionPairs", &rai::RRT_PathFinder::setExplicitCollisionPairs, "only after setProblem", pybind11::arg("collisionPairs"))
      .def("solve", &rai::RRT_PathFinder::solve, "", pybind11::arg("verbose")=1)
      .def("get_resampledPath", &rai::RRT_PathFinder::get_resampledPath, "")

      ;

}

#endif
