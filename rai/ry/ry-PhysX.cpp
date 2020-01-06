#ifdef RAI_PYBIND

#include "ry-PhysX.h"
#include "ry-Config.h"
#include "types.h"

#include <Kin/kin_physx.h>

void init_PhysX(pybind11::module &m) {
pybind11::class_<PhysXInterface, std::shared_ptr<PhysXInterface>>(m, "PhysXInterface")

.def("step", &PhysXInterface::step)

.def("step", [](PhysXInterface& self, ry::Config& C) {
  self.pushKinematicStates(C.get()->frames);
  self.step();
  self.pullDynamicStates(C.set()->frames);
})

.def("getState", [](PhysXInterface& self, ry::Config& C) {
  arr V;
  self.pullDynamicStates(C.set()->frames, V);
  return pybind11::array(V.dim(), V.p);
})

.def("setState", [](PhysXInterface& self, ry::Config& C, const pybind11::array& velocities) {
  self.pushFullState(C.get()->frames, numpy2arr(velocities));
})
;
}

#endif
