/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Config.h"
#include "ry-Simulation.h"
#include "types.h"

#include "../Kin/frame.h"
#include "../Kin/simulation.h"
#include "../Geo/depth2PointCloud.h"

namespace rai {
struct SimulationState {
  arr frameState;
  arr frameVels;

  SimulationState(const arr& _frameState, const arr& _frameVels) : frameState(_frameState), frameVels(_frameVels) {}
};
}

void init_Simulation(pybind11::module& m) {
  pybind11::class_<rai::Simulation, std::shared_ptr<rai::Simulation>>(m, "Simulation")

  .def(pybind11::init([](shared_ptr<rai::Configuration>& C, rai::Simulation::SimulatorEngine engine, int verbose) {
    return make_shared<rai::Simulation>(*C, engine, verbose);
  }))

  .def("step", &rai::Simulation::step,
       "",
       pybind11::arg("u_control"),
       pybind11::arg("tau") = .01,
       pybind11::arg("u_mode") = rai::Simulation::_velocity
      )

  .def("get_q", &rai::Simulation::get_q)

  .def("get_qDot", &rai::Simulation::get_qDot)

  .def("openGripper", &rai::Simulation::openGripper,
       "",
       pybind11::arg("gripperFrameName"),
       pybind11::arg("width") = .075,
       pybind11::arg("speed") = .3
      )

  .def("closeGripper", &rai::Simulation::closeGripper,
       "",
       pybind11::arg("gripperFrameName"),
       pybind11::arg("width") = .05,
       pybind11::arg("speed") = .3,
       pybind11::arg("force") = 20.
      )

  .def("getGripperWidth", &rai::Simulation::getGripperWidth,
       "",
       pybind11::arg("gripperFrameName")
      )

  .def("getGripperIsGrasping", &rai::Simulation::getGripperIsGrasping,
       "",
       pybind11::arg("gripperFrameName")
      )

  .def("getImageAndDepth", [](std::shared_ptr<rai::Simulation>& self) {
    byteA rgb;
    floatA depth;
    self->getImageAndDepth(rgb, depth);
    return pybind11::make_tuple(pybind11::array_t<byte>(rgb.dim(), rgb.p),
                                pybind11::array_t<float>(depth.dim(), depth.p));
  })

//  .def("getSegmentation", [](std::shared_ptr<rai::Simulation>& self) {
//    byteA seg;
//    self->getSegmentation(seg);
//    return pybind11::array_t<byte>(seg.dim(), seg.p);
//  })

  .def("addSensor",  &rai::Simulation::addSensor,
       "",
       pybind11::arg("sensorName"),
       pybind11::arg("frameAttached") = std::string(),
       pybind11::arg("width") = 640,
       pybind11::arg("height") = 360,
       pybind11::arg("focalLength") = -1.,
       pybind11::arg("orthoAbsHeight") = -1.,
       pybind11::arg("zRange") = std::vector<double>()
      )
  .def("selectSensor",  &rai::Simulation::selectSensor,
       "",
       pybind11::arg("sensorName")
      )

  .def("getGroundTruthPosition", [](std::shared_ptr<rai::Simulation>& self, const char* frame) {
    rai::Frame* f = self->C.getFrame(frame);
    arr x = f->getPosition();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getGroundTruthRotationMatrix", [](std::shared_ptr<rai::Simulation>& self, const char* frame) {
    rai::Frame* f = self->C.getFrame(frame);
    arr x = f->getRotationMatrix();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("getGroundTruthSize", [](std::shared_ptr<rai::Simulation>& self, const char* frame) {
    rai::Frame* f = self->C.getFrame(frame);
    arr x = f->getSize();
    return pybind11::array_t<double>(x.dim(), x.p);
  })

  .def("addImp", &rai::Simulation::addImp)

  .def("getState", [](std::shared_ptr<rai::Simulation>& self) {
    ptr<rai::SimulationState> state = self->getState();
    return pybind11::make_tuple(pybind11::array_t<double>(state->frameState.dim(), state->frameState.p),
                                pybind11::array_t<double>(state->frameVels.dim(), state->frameVels.p));
  })

  .def("restoreState", &rai::Simulation::restoreState)

  .def("setState", &rai::Simulation::setState,
       "",
       pybind11::arg("frameState"),
       pybind11::arg("frameVelocities") = std::vector<double>()
      )

  .def("depthData2pointCloud", [](std::shared_ptr<rai::Simulation>& self, const pybind11::array_t<float>& depth, const std::vector<double>& Fxypxy) {
    arr points;
    floatA _depth = numpy2arr<float>(depth);
    depthData2pointCloud(points, _depth, arr(Fxypxy, true));
    return pybind11::array(points.dim(), points.p);
  })

  .def("getScreenshot", &rai::Simulation::getScreenshot)

  ;

  pybind11::class_<rai::CameraView::Sensor, std::shared_ptr<rai::CameraView::Sensor>>(m, "CameraViewSensor");

}

#endif
