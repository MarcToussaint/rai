/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Bullet.h"
#include "ry-Config.h"
#include "types.h"
#include "../Kin/kin_bullet.h"

void init_Bullet(pybind11::module& m) {
  pybind11::class_<BulletInterface, std::shared_ptr<BulletInterface>>(m, "BulletInterface")

      .def("step", &BulletInterface::step)

  .def("step", [](BulletInterface& self, shared_ptr<rai::Configuration>& C) {
    self.pushKinematicStates(C->frames);
    self.step();
    self.pullDynamicStates(C->frames);
  })

  .def("getState", [](BulletInterface& self, shared_ptr<rai::Configuration>& C) {
    arr V;
    self.pullDynamicStates(C->frames, V);
    return pybind11::array(V.dim(), V.p);
  })

  .def("setState", [](BulletInterface& self, shared_ptr<rai::Configuration>& C, const pybind11::array_t<double>& velocities) {
    self.pushFullState(C->frames, numpy2arr(velocities));
  })
  ;
}

#endif
