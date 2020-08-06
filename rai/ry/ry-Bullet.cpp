#ifdef RAI_PYBIND

#include "ry-Bullet.h"
#include "ry-Config.h"
#include "types.h"
#include "../Kin/kin_bullet.h"

void init_Bullet(pybind11::module &m) {
pybind11::class_<BulletInterface, std::shared_ptr<BulletInterface>>(m, "BulletInterface")

.def("step", &BulletInterface::step)

.def("step", [](BulletInterface& self, ry::Config& C) {
  self.pushKinematicStates(C.get()->frames);
  self.step();
  self.pullDynamicStates(C.set()->frames);
})

.def("getState", [](BulletInterface& self, ry::Config& C) {
  arr V;
  self.pullDynamicStates(C.set()->frames, V);
  return pybind11::array(V.dim(), V.p);
})

.def("setState", [](BulletInterface& self, ry::Config& C, const pybind11::array_t<double>& velocities) {
  self.pushFullState(C.get()->frames, numpy2arr(velocities));
})
;
}

#endif
