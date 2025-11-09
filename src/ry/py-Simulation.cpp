/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-Config.h"
#include "py-Simulation.h"
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
  pybind11::class_<rai::Simulation::State, std::shared_ptr<rai::Simulation::State>>(m, "SimulationState", "used only for set|getState")
      .def(pybind11::init<>())
      .def_readwrite("time", &rai::Simulation::State::time)
      .def_readwrite("q", &rai::Simulation::State::q)
      .def_readwrite("qDot", &rai::Simulation::State::qDot)
      .def_readwrite("freePos", &rai::Simulation::State::freePos)
      .def_readwrite("freeVel", &rai::Simulation::State::freeVel)
      .def("__str__", [](std::shared_ptr<rai::Simulation::State>& self) {  str s;  s <<"time: "<< self->time <<", q: " <<self->q <<", qDot: " <<self->qDot <<", freePos: " <<self->freePos <<", freeVel: " <<self->freeVel;  return std::string(s.p); })
      ;

  pybind11::class_<rai::Simulation, std::shared_ptr<rai::Simulation>>(m, "Simulation", "A direct simulation interface to physics engines (Nvidia PhysX, Bullet) -- see https://marctoussaint.github.io/robotics-course/tutorials/simulation.html")

      .def(pybind11::init<rai::Configuration&, rai::Simulation::Engine, int>(), "create a Simulation that is associated/attached to the given configuration",
           pybind11::arg("C"),
           pybind11::arg("engine"),
           pybind11::arg("verbose") = 2)

//  .def(pybind11::init([](shared_ptr<rai::Configuration>& C, rai::Simulation::Engine engine, int verbose) {
//    return make_shared<rai::Simulation>(*C, engine, verbose);
//  }))

      .def("step", &rai::Simulation::step,
           "",
           pybind11::arg("u_control") = arr{},
           pybind11::arg("tau") = .01,
           pybind11::arg("u_mode") = rai::Simulation::_none
          )

      .def("multi_step", &rai::Simulation::multi_step, "run multiple steps of tau_sim using spline control; tau_step should be multiple of tau_sim",
           pybind11::arg("tau_step"),
           pybind11::arg("tau_sim"))

      .def("setSplineRef", &rai::Simulation::setSplineRef,
           "set the spline reference to generate motion"
           "\n* path: single configuration, or sequence of spline control points"
           "\n* times: array with single total duration, or time for each control point (times.N==path.d0)"
           "\n* append: append (with zero-velocity at append), or smoothly overwrite",
           pybind11::arg("path"),
           pybind11::arg("times"),
           pybind11::arg("append") = true
          )

      .def("resetSplineRef", &rai::Simulation::resetSplineRef,
           "reset the spline reference, i.e., clear the current spline buffer and initialize it to constant spline at current position (to which setSplineRef can append); ctrl_time=-1 means current control time",
           pybind11::arg("ctrl_time")=-1.)

      .def("getTimeToSplineEnd", &rai::Simulation::getTimeToSplineEnd)

      .def("get_t", &rai::Simulation::get_t)
      .def("get_q", &rai::Simulation::get_q)
      .def("get_qDot", &rai::Simulation::get_qDot)
      .def("get_frameVelocities", &rai::Simulation::get_frameVelocities)

      .def("moveGripper", &rai::Simulation::moveGripper, "", pybind11::arg("gripperFrameName"), pybind11::arg("width"), pybind11::arg("speed") = .3)
      .def("gripperIsDone", &rai::Simulation::gripperIsDone, "", pybind11::arg("gripperFrameName"))
      .def("getGripperWidth", &rai::Simulation::getGripperWidth, "", pybind11::arg("gripperFrameName"))

  .def("getImageAndDepth", [](std::shared_ptr<rai::Simulation>& self) {
    byteA rgb;
    floatA depth;
    self->getImageAndDepth(rgb, depth);
    return pybind11::make_tuple(Array2numpy<byte>(rgb),
                                Array2numpy<float>(depth));
  })

  .def("addSensor",  &rai::Simulation::addSensor,
       "",
       pybind11::arg("sensorName"),
       pybind11::arg("width") = 640,
       pybind11::arg("height") = 360,
       pybind11::arg("focalLength") = -1.,
       pybind11::arg("orthoAbsHeight") = -1.,
       pybind11::arg("zRange") = std::vector<double>()
      )
  .def("setSimulateDepthNoise", &rai::Simulation::setSimulateDepthNoise, "specify (boolean) on whether to simulate noise", pybind11::arg("_setSimulateDepthNoise"))
  .def("selectSensor",  &rai::Simulation::selectSensor,
       "",
       pybind11::arg("sensorName")
      )

  .def("getState", &rai::Simulation::getState, "returns struct with 5 fields (time, q, qDot, freePos, freeVel)")

  .def("getFreeFrames", [](std::shared_ptr<rai::Simulation>& self) {
	FrameL frames = self->getFreeFrames();
	std::vector<shared_ptr<rai::Frame>> F;
	for(rai::Frame* f:frames) F.push_back(shared_ptr<rai::Frame>(f, &null_deleter)); //giving it a non-deleter!
	return F;
      }, "")

  .def("setState", &rai::Simulation::setState, "", pybind11::arg("state"))

  .def("resetTime", &rai::Simulation::resetTime, "", pybind11::arg("time")=1.)

  .def("pushConfigToSim", &rai::Simulation::pushConfigToSim,
       "set the simulator to the full (frame) state of the configuration",
       pybind11::arg("frameVelocities") = NoArr,
       pybind11::arg("jointVelocities") = NoArr
      )

  .def("attach", &rai::Simulation::attach, "", pybind11::arg("from"), pybind11::arg("to") )
  .def("detach", &rai::Simulation::detach, "", pybind11::arg("from"), pybind11::arg("to") )

  .def("depthData2pointCloud", [](std::shared_ptr<rai::Simulation>& self, const pybind11::array_t<float>& depth, const std::vector<double>& fxycxy) {
    arr points;
    floatA _depth = numpy2arr<float>(depth);
    depthData2pointCloud(points, _depth, as_arr(fxycxy, true));
    return arr2numpy(points);
  })

  .def("setVerbose", [](std::shared_ptr<rai::Simulation>& self, int verbose) { self->verbose=verbose; }, "", pybind11::arg("verbose"))
  .def("getScreenshot", &rai::Simulation::getScreenshot)

  ;

  pybind11::class_<rai::CameraView::CameraFrame, std::shared_ptr<rai::CameraView::CameraFrame>>(m, "CameraViewSensor");

}

#endif
