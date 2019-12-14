#ifdef RAI_PYBIND

#include "ry-PhysX.h"
#include "ry-Config.h"
#include "types.h"

#include <Kin/kin_physx.h>

void init_PhysX(pybind11::module &m) {
pybind11::class_<ry::RyPhysX>(m, "RyPhysX")
.def("step", [](ry::RyPhysX& self) {
  self.physx->step();
})

.def("step", [](ry::RyPhysX& self, ry::Config& C) {
  self.physx->pushKinematicStates(C.get()->frames);
  self.physx->step();
  self.physx->pullDynamicStates(C.set()->frames);
})

.def("getState", [](ry::RyPhysX& self, ry::Config& C) {
  arr V;
  self.physx->pullDynamicStates(C.set()->frames, V);
  return pybind11::array(V.dim(), V.p);
})

.def("setState", [](ry::RyPhysX& self, ry::Config& C, const pybind11::array& velocities) {
  self.physx->pushFullState(C.get()->frames, numpy2arr(velocities));
})
;
}

#endif
