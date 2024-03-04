/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-PhysX.h"
#include "py-Config.h"
#include "types.h"

#include "../Core/util.h"
#include "../Kin/kin_physx.h"

void init_PhysX(pybind11::module& m) {
  pybind11::class_<PhysXInterface, std::shared_ptr<PhysXInterface>>(m, "PhysXInterface")

      .def("step", &PhysXInterface::step)

  .def("step", [](PhysXInterface& self, shared_ptr<rai::Configuration>& C) {
    self.pushKinematicStates(*C);
    self.step();
    self.pullDynamicStates(*C);
  })

  .def("getState", [](PhysXInterface& self, shared_ptr<rai::Configuration>& C) {
    arr V;
    self.pullDynamicStates(*C, V);
    return arr2numpy(V);
  })

  .def("setState", [](PhysXInterface& self, shared_ptr<rai::Configuration>& C, const pybind11::array_t<double>& velocities) {
    self.pushFrameStates(*C, numpy2arr(velocities));
  })
  ;
}

#endif
