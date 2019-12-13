#include "ry-Bullet.h"
#include "ry-Config.h"
#include "types.h"

#include <Kin/kin_bullet.h>

void init_Bullet(pybind11::module &m) {
pybind11::class_<ry::RyBullet>(m, "RyBullet")
.def("step", [](ry::RyBullet& self) {
  self.bullet->step();
})

.def("step", [](ry::RyBullet& self, ry::Config& C) {
  self.bullet->pushKinematicStates(C.get()->frames);
  self.bullet->step();
  self.bullet->pullDynamicStates(C.set()->frames);
})

.def("getState", [](ry::RyBullet& self, ry::Config& C) {
  arr V;
  self.bullet->pullDynamicStates(C.set()->frames, V);
  return pybind11::array(V.dim(), V.p);
})

.def("setState", [](ry::RyBullet& self, ry::Config& C, const pybind11::array& velocities) {
  self.bullet->pushFullState(C.get()->frames, numpy2arr(velocities));
})
;
}
