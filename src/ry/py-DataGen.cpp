/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "py-Config.h"

#include "../DataGen/shapenetGrasps.h"
#include "../DataGen/rndStableConfigs.h"
#include "../Kin/kin_physx.h"

void init_DataGen(pybind11::module& m) {

  pybind11::module_ mod = m.def_submodule("DataGen", "rai data generators");

  mod.def("sampleGraspCandidate", &::sampleGraspCandidate, "sample random grasp candidates on an object represented as points",
          pybind11::arg("C"),
          pybind11::arg("ptsFrame"),
          pybind11::arg("refFrame"),
          pybind11::arg("pregraspNormalSdv")=.2,
          pybind11::arg("verbose")=1);

  //===========================================================================

  pybind11::class_<ShapenetGrasps, std::shared_ptr<ShapenetGrasps>>(mod, "ShapenetGrasps", "A generator of random grasps on random shapenet objects")

      .def(pybind11::init<>())

  .def("getSamples",  [](std::shared_ptr<ShapenetGrasps>& self, uint n) {
    arr X, S;
    uintA Z;
    self->getSamples(X, Z, S, n);
    return std::tuple<arr, uintA, arr>(X, Z, S);
  },
  "(batch interface) return three arrays: samples X, contexts Z, scores S (each row are scores for one sample - see evaluateSamples)",
  pybind11::arg("nSamples"))
  .def("evaluateSample", &ShapenetGrasps::evaluateSample,
       "(batch interface) returns scores for a single sample - this (row) are numbers where a single 'negative' means fail",
       pybind11::arg("sample"),
       pybind11::arg("context"))
  .def("displaySamples", &ShapenetGrasps::displaySamples,
       "(batch interface) displays all samples",
       pybind11::arg("samples"),
       pybind11::arg("context"),
       pybind11::arg("scores") = arr{})

  .def("loadObject", &ShapenetGrasps::loadObject,
       "(direct interface) clear scene and load object and gripper",
       pybind11::arg("shape"),
       pybind11::arg("rndPose")=true)
  .def("getPointCloud", &ShapenetGrasps::getPointCloud,
       "(direct interface) return pcl of loaded object")
  .def("sampleGraspPose", &ShapenetGrasps::sampleGraspPose,
       "(direct interface) return (relative) pose of random sampled grasp candidate")
  .def("setGraspPose", &ShapenetGrasps::setGraspPose,
       "(direct interface) set (relative) pose of grasp candidate",
       pybind11::arg("pose"),
       pybind11::arg("objPts")="objPts0")
  .def("evaluateGrasp", &ShapenetGrasps::evaluateGrasp,
       "(direct interface) return scores of grasp candidate (min(scores)<0. means fail)")

  .def("getConfig",  [](std::shared_ptr<ShapenetGrasps>& self) { return std::shared_ptr<rai::Configuration>(&(self->C), &Config_null_deleter); }, "")

  .def("setOptions", [](std::shared_ptr<ShapenetGrasps>& self
#define OPT(type, name, x) ,type name
                        OPT(int, verbose, 1)
                        OPT(rai::String, filesPrefix, "shapenet/models/")
                        OPT(int, numShapes, -1)
                        OPT(int, startShape, 0)
                        OPT(int, simVerbose, 0)
                        OPT(int, optVerbose, 0)
                        OPT(double, simTau, .01)
                        OPT(double, gripperCloseSpeed, .001)
                        OPT(double, moveSpeed, .005)
                        OPT(double, pregraspNormalSdv, .2)
#undef OPT
  ) {
    self->opt
#define OPT(type, name, x) .set_##name(name)
    OPT(int, verbose, 1)
    OPT(rai::String, filesPrefix, "shapenet/models/")
    OPT(int, numShapes, -1)
    OPT(int, startShape, 0)
    OPT(int, simVerbose, 0)
    OPT(int, optVerbose, 0)
    OPT(double, simTau, .01)
    OPT(double, gripperCloseSpeed, .001)
    OPT(double, moveSpeed, .005)
    OPT(double, pregraspNormalSdv, .2)
#undef OPT
    ;
    return self;
  }, "set options"
#define OPT(type, name, x) , pybind11::arg(#name) = x
  OPT(int, verbose, 1)
  OPT(rai::String, filesPrefix, "shapenet/models/")
  OPT(int, numShapes, -1)
  OPT(int, startShape, 0)
  OPT(int, simVerbose, 0)
  OPT(int, optVerbose, 0)
  OPT(double, simTau, .01)
  OPT(double, gripperCloseSpeed, .001)
  OPT(double, moveSpeed, .005)
  OPT(double, pregraspNormalSdv, .2)
#undef OPT
      )

  .def("setPhysxOptions", [](std::shared_ptr<ShapenetGrasps>& self
#define OPT(type, name, x) ,type name
                             OPT(int, verbose, 1)
                             OPT(bool, yGravity, false)
                             OPT(double, angularDamping, .1)
                             OPT(double, defaultFriction, 1.)
                             OPT(double, defaultRestitution, .1) //restitution=1 should be elastic...
                             OPT(double, motorKp, 1000.)
                             OPT(double, motorKd, 100.)
                             OPT(double, gripperKp, 10000.)
                             OPT(double, gripperKd, 100.)
#undef OPT
  ) {
    self->physxOpt
#define OPT(type, name, x) .set_##name(name)
    OPT(int, verbose, 1)
    OPT(bool, yGravity, false)
    OPT(double, angularDamping, .1)
    OPT(double, defaultFriction, 1.)
    OPT(double, defaultRestitution, .1) //restitution=1 should be elastic...
    OPT(double, motorKp, 1000.)
    OPT(double, motorKd, 100.)
    OPT(double, gripperKp, 10000.)
    OPT(double, gripperKd, 100.)
#undef OPT
    ;
    return self;
  }, "set options"
#define OPT(type, name, x) , pybind11::arg(#name) = x
  OPT(int, verbose, 1)
  OPT(bool, yGravity, false)
  OPT(double, angularDamping, .1)
  OPT(double, defaultFriction, 1.)
  OPT(double, defaultRestitution, .1) //restitution=1 should be elastic...
  OPT(double, motorKp, 1000.)
  OPT(double, motorKd, 100.)
  OPT(double, gripperKp, 10000.)
  OPT(double, gripperKd, 100.)
#undef OPT
      )

  ;

  //===========================================================================

  pybind11::class_<RndStableConfigs, std::shared_ptr<RndStableConfigs>>(mod, "RndStableConfigs", "A generator of random stable configurations")

      .def(pybind11::init<>())

      .def("getSample", &RndStableConfigs::getSample,
           "sample a random configuration - displayed, access via config passed at construction",
           pybind11::arg("config"),
           pybind11::arg("supports"))

      .def("report", &RndStableConfigs::report, "info on newton steps -per- feasible sample")

      .def("setOptions", [](std::shared_ptr<RndStableConfigs>& self
#define OPT(type, name, x) ,type name
                            OPT(int, verbose, 1)
                            OPT(double, frictionCone_mu, .8)
#undef OPT
  ) {
    self->opt
#define OPT(type, name, x) .set_##name(name)
    OPT(int, verbose, 1)
    OPT(double, frictionCone_mu, .8)
#undef OPT
    ;
    return self;
  }, "set options"
#define OPT(type, name, x) , pybind11::arg(#name) = x
  OPT(int, verbose, 1)
  OPT(double, frictionCone_mu, .8)
#undef OPT
          )

  ;
}

#endif
