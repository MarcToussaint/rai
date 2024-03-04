/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../KOMO/skeleton.h"
#include "../KOMO/komo.h"

void list2skeleton(rai::Skeleton& S, const pybind11::list& L) {
  for(uint i=0; i<L.size(); i+=3) {
    std::vector<double> when = L[i].cast<std::vector<double>>();
    CHECK(when.size()<=2, "Skeleton error entry " <<i/3 <<" time interval: interval needs no, 1, or 2 elements");
    if(when.size()==0) when= {0., -1.};
    if(when.size()==1) when= {when[0], when[0]};
    rai::SkeletonSymbol symbol=rai::SY_none;
    try {
      symbol = L[i+1].cast<rai::SkeletonSymbol>();
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<i/3 <<" symbol: " <<err.what() <<endl;
    }
    StringA frames;
    try {
      frames = L[i+2].cast<StringA>();
    } catch(std::runtime_error& err) {
      LOG(-1) <<"Skeleton error line " <<i/3 <<" frames: " <<err.what() <<endl;
    }
    S.S.append(rai::SkeletonEntry(when[0], when[1], symbol, frames));
  }
}

//===========================================================================

void init_Skeleton(pybind11::module& m) {
  pybind11::class_<rai::Skeleton, std::shared_ptr<rai::Skeleton>>(m, "Skeleton")

      .def(pybind11::init<>())

  .def("add",  [](std::shared_ptr<rai::Skeleton>& self, const pybind11::list& L) {
    list2skeleton(*self, L);
  }, "")

  .def("addEntry", &rai::Skeleton::addEntry, "", pybind11::arg("timeInterval"), pybind11::arg("symbol"), pybind11::arg("frames"))
  .def("addExplicitCollisions", &rai::Skeleton::addExplicitCollisions, "", pybind11::arg("collisions"))
  .def("addLiftPriors", &rai::Skeleton::addLiftPriors, "", pybind11::arg("lift"))
  .def("getMaxPhase", &rai::Skeleton::getMaxPhase, "")
  .def("getKomo_path", &rai::Skeleton::getKomo_path, "", pybind11::arg("Configuration"), pybind11::arg("stepsPerPhase"), pybind11::arg("accScale"), pybind11::arg("lenScale"), pybind11::arg("homingScale"), pybind11::arg("collScale"))
  .def("getKomo_waypoints", &rai::Skeleton::getKomo_waypoints, "", pybind11::arg("Configuration"), pybind11::arg("lenScale"), pybind11::arg("homingScale"), pybind11::arg("collScale"))
  .def("getKomo_finalSlice", &rai::Skeleton::getKomo_finalSlice, "", pybind11::arg("Configuration"), pybind11::arg("lenScale"), pybind11::arg("homingScale"), pybind11::arg("collScale"))

  .def("enableAccumulatedCollisions", [](std::shared_ptr<rai::Skeleton>& self, bool enable) {
    self->collisions = enable;
  }, "", pybind11::arg("enable")=true)

  .def("getTwoWaypointProblem", [](std::shared_ptr<rai::Skeleton>& self, int t2, KOMO& komoWays) {
    auto C = make_shared<rai::Configuration>();
    arr q1, q2;
    self->getTwoWaypointProblem(t2, *C, q1, q2, komoWays);
    pybind11::tuple tuple(3);
    tuple[0] = *C; tuple[1]=q1; tuple[2]=q2;
    return tuple;
  }, "", pybind11::arg("t2"), pybind11::arg("komoWays"))

  ;

  //===========================================================================

}

#endif
