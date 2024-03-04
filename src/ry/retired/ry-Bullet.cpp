/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-Bullet.h"
#include "py-Config.h"
#include "types.h"
#include "../Kin/kin_bullet.h"

void init_Bullet(pybind11::module& m) {
  pybind11::class_<BulletInterface, std::shared_ptr<BulletInterface>>(m, "BulletInterface")

      .def("step", &BulletInterface::step)

  .def("step", [](BulletInterface& self, shared_ptr<rai::Configuration>& C) {
    self.pushKinematicStates(*C);
    self.step();
    self.pullDynamicStates(*C);
  })

  .def("getState", [](BulletInterface& self, shared_ptr<rai::Configuration>& C) {
    arr V;
    self.pullDynamicStates(*C, V);
    return arr2numpy(V);
  })

  .def("setState", [](BulletInterface& self, shared_ptr<rai::Configuration>& C, const pybind11::array_t<double>& velocities) {
    self.pushFullState(*C, numpy2arr(velocities));
  })
  ;
}

#endif
