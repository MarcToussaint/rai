/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Geo/geo.h"

void init_Quaternion(pybind11::module& m) {

  //===========================================================================

  pybind11::class_<rai::Quaternion, shared_ptr<rai::Quaternion>>(m, "Quaternion", "")

      .def(pybind11::init<>(), "non-initialized")

      .def("setZero", &rai::Quaternion::setZero, "")
      .def("set", pybind11::overload_cast<const arr&>(&rai::Quaternion::set), "", pybind11::arg("q"))
      .def("setRandom", &rai::Quaternion::setRandom, "")
      .def("setExp", &rai::Quaternion::setExp, "", pybind11::arg("vector_w"))
      .def("setRad", pybind11::overload_cast<double, const rai::Vector&>(&rai::Quaternion::setRad), "", pybind11::arg("radians"), pybind11::arg("axis"))
      .def("setMatrix", pybind11::overload_cast<const arr&>(&rai::Quaternion::setMatrix), "", pybind11::arg("R"))
      .def("setEuler", &rai::Quaternion::setEuler, "", pybind11::arg("euler_zxz"))
      .def("setRollPitchYaw", &rai::Quaternion::setRollPitchYaw, "", pybind11::arg("roll_pitch_yaw"))
      .def("setDiff", &rai::Quaternion::setDiff, "", pybind11::arg("from"), pybind11::arg("to"))
      .def("setInterpolateEmbedded", &rai::Quaternion::setInterpolateEmbedded, "", pybind11::arg("t"), pybind11::arg("from"), pybind11::arg("to"))
      .def("setInterpolateProper", &rai::Quaternion::setInterpolateProper, "", pybind11::arg("t"), pybind11::arg("from"), pybind11::arg("to"))

      .def("getLog", &rai::Quaternion::getLog, "")
      .def("getMatrix", pybind11::overload_cast<>(&rai::Quaternion::getMatrix, pybind11::const_), "")
      .def("getRad", pybind11::overload_cast<>(&rai::Quaternion::getRad, pybind11::const_), "")
      .def("getArr", &rai::Quaternion::getArr, "")
      .def("getRollPitchYaw", &rai::Quaternion::getRollPitchYaw, "")
      .def("getJacobian", &rai::Quaternion::getJacobian, "")

      .def("invert", &rai::Quaternion::invert, "")
      .def("flipSign", &rai::Quaternion::flipSign, "")
      .def("normalize", &rai::Quaternion::normalize, "")
      .def("append", &rai::Quaternion::append, "", pybind11::arg("q"))
      .def("multiply", &rai::Quaternion::multiply, "", pybind11::arg("f"))

      .def("sqrNorm", &rai::Quaternion::sqrNorm, "")
      .def("applyOnPointArray", &rai::Quaternion::applyOnPointArray, "", pybind11::arg("pts"))

      ;

  //===========================================================================


}

#endif
