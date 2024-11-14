/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../DataGen/shapenetGrasps.h"
#include "../Kin/kin_physx.h"

void init_DataGen(pybind11::module& m) {

  pybind11::module_ mod = m.def_submodule("DataGen", "rai data generators");

  pybind11::class_<ShapenetGrasps, std::shared_ptr<ShapenetGrasps>>(mod, "ShapenetGrasps", "A generator of random grasps on random shapenet objects")

      .def(pybind11::init<>())

      .def("getSamples",  [](std::shared_ptr<ShapenetGrasps>& self, uint n) {
        arr X, Z, S;
        self->getSamples(X, Z, S, n);
        return std::tuple<arr, arr, arr>(X, Z, S);
      }, "return three arrays: samples X, contexts Z, scores S (each row are scores for one sample - see evaluateSamples)", pybind11::arg("number of samples to be returned"))
      .def("evaluateSample", &ShapenetGrasps::evaluateSample, "returns scores for a single sample - this (row) are numbers where a single 'negative' means fail",
           pybind11::arg("sample x"),
           pybind11::arg("context z"))
      .def("displaySamples", &ShapenetGrasps::displaySamples, "displays all samples",
           pybind11::arg("samples X"),
           pybind11::arg("context Z"),
           pybind11::arg("scores S (optional)") = arr{})

      .def("setOptions", [](std::shared_ptr<ShapenetGrasps>& self, std::shared_ptr<ShapenetGrasps_Options>& opt) { self->opt = *opt; }, "")
      .def("setPhysxOptions", [](std::shared_ptr<ShapenetGrasps>& self, std::shared_ptr<rai::PhysX_Options>& opt) { self->physxOpt = *opt; }, "")

  ;

  pybind11::class_<ShapenetGrasps_Options, std::shared_ptr<ShapenetGrasps_Options>>(mod, "ShapenetGrasps_Options", "options")
      .def(pybind11::init<>())
    #define OPT(name) .def_readwrite(#name, &ShapenetGrasps_Options::name)
      OPT(verbose)
      OPT(dataPerShape)
      OPT(filesPrefix)
      OPT(numShapes)
      OPT(startShape)
      OPT(evaluationsFile)
      OPT(simVerbose)
      OPT(optVerbose)
      OPT(simTau)
      OPT(gripperCloseSpeed)
      OPT(moveSpeed)
      OPT(pregraspNormalSdv)
      OPT(trainingFile)
    #undef OPT
      ;

  pybind11::class_<rai::PhysX_Options, std::shared_ptr<rai::PhysX_Options>>(mod, "PhysX_Options", "options")
      .def(pybind11::init<>())
    #define OPT(name) .def_readwrite(#name, &rai::PhysX_Options::name)
      OPT(verbose)
      OPT(yGravity)
      OPT(softBody)
      OPT(multiBody)
      OPT(multiBodyDisableGravity)
      OPT(jointedBodies)
      OPT(angularDamping)
      OPT(defaultFriction)
      OPT(defaultRestitution)
      OPT(motorKp)
      OPT(motorKd)
      OPT(gripperKp)
      OPT(gripperKd)
    #undef OPT
      ;

}

#endif
