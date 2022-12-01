/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../KOMO/skeleton.h"

void init_Skeleton(pybind11::module& m) {
  pybind11::class_<rai::Skeleton, std::shared_ptr<rai::Skeleton>>(m, "Skeleton")

  .def(pybind11::init<>())
  .def("addEntry", &rai::Skeleton::addEntry, "", pybind11::arg("timeInterval"), pybind11::arg("symbol"), pybind11::arg("frames") )
  .def("addExplicitCollisions", &rai::Skeleton::addExplicitCollisions, "", pybind11::arg("collisions") )
  .def("addLiftPriors", &rai::Skeleton::addLiftPriors, "", pybind11::arg("lift") )
  .def("getMaxPhase", &rai::Skeleton::getMaxPhase, "")
  .def("getKomo_path", &rai::Skeleton::getKomo_path, "", pybind11::arg("Configuration"), pybind11::arg("stepsPerPhase"), pybind11::arg("accScale"), pybind11::arg("lenScale"), pybind11::arg("homingScale") )
  .def("getKomo_waypoints", &rai::Skeleton::getKomo_waypoints, "", pybind11::arg("Configuration"), pybind11::arg("lenScale"), pybind11::arg("homingScale") )
  .def("getKOMO_finalSlice", &rai::Skeleton::getKOMO_finalSlice, "", pybind11::arg("Configuration"), pybind11::arg("lenScale"), pybind11::arg("homingScale") )

  .def("getTwoWaypointProblem", [](std::shared_ptr<rai::Skeleton>& self, int t2, KOMO& komoWays){
    auto C = make_shared<rai::Configuration>();
    arr q1, q2;
    self->getTwoWaypointProblem(t2, *C, q1, q2, komoWays);
    pybind11::tuple tuple(3);
    tuple[0] = *C; tuple[1]=q1; tuple[2]=q2;
    return tuple;
  }, "", pybind11::arg("t2"), pybind11::arg("komoWays") )

  ;

}

#endif
