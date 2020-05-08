#ifdef RAI_PYBIND

#include "ry-Simulation.h"
#include "types.h"

#include "../Kin/simulation.h"
#include "../Perception/depth2PointCloud.h"

namespace rai{
  struct SimulationState {
    arr frameState;
    arr frameVels;

    SimulationState(const arr& _frameState, const arr& _frameVels) : frameState(_frameState), frameVels(_frameVels) {}
  };
}

void init_Simulation(pybind11::module &m) {
  pybind11::class_<rai::Simulation, std::shared_ptr<rai::Simulation>>(m, "RySimulation")

  .def("step", &rai::Simulation::step,
//       [](std::shared_ptr<rai::Simulation>& self, const std::vector<double>& u_control, double tau=.01, rai::Simulation::ControlMode u_mode) {
//    arr u = conv_stdvec2arr(u_control);
//    self->step(u, tau, u_mode);
//  },
  "",
    pybind11::arg("u_control"),
    pybind11::arg("tau") = .01,
    pybind11::arg("u_mode") = rai::Simulation::_velocity
  )

  .def("openGripper", [](std::shared_ptr<rai::Simulation>& self, const char* gripperFrameName, double width, double speed) {
     self->openGripper(gripperFrameName, width, speed);
  }, "",
    pybind11::arg("gripperFrameName"),
    pybind11::arg("width") = .075,
    pybind11::arg("speed") = .2
  )

  .def("closeGripper", [](std::shared_ptr<rai::Simulation>& self, const char* gripperFrameName, double width, double speed, double force) {
    self->closeGripper(gripperFrameName, width, speed, force);
  }, "",
    pybind11::arg("gripperFrameName"),
    pybind11::arg("width") = .05,
    pybind11::arg("speed") = 1.,
    pybind11::arg("force") = 20.
  )



  .def("get_q", [](std::shared_ptr<rai::Simulation>& self) {
     arr q = self->get_q();
     return pybind11::array(q.dim(), q.p);
  })

  .def("get_qDot", [](std::shared_ptr<rai::Simulation>& self) {
    arr qdot = self->qdot();
    return pybind11::array(qdot.dim(), qdot.p);
  })

  .def("getGripperWidth", [](std::shared_ptr<rai::Simulation>& self, const char* gripperFrameName) {
    return self->getGripperWidth(gripperFrameName);
  }, "",
    pybind11::arg("gripperFrameName")
  )

  .def("getGripperIsGrasping", [](std::shared_ptr<rai::Simulation>& self, const char* gripperFrameName) {
    return self->getGripperIsGrasping(gripperFrameName);
  }, "",
    pybind11::arg("gripperFrameName")
  )


  .def("addSensor", [](std::shared_ptr<rai::Simulation>& self, const char* cameraFrameName) {
    self->cameraview().addSensor(cameraFrameName);
  }, "",
    pybind11::arg("cameraFrameName")
  )

  .def("getImageAndDepth", [](std::shared_ptr<rai::Simulation>& self) {
    byteA rgb;
    floatA depth;
    self->getImageAndDepth(rgb, depth);
    return pybind11::make_tuple(pybind11::array_t<byte>(rgb.dim(), rgb.p),
                                pybind11::array_t<float>(depth.dim(), depth.p));
  })



  .def("getState", [](std::shared_ptr<rai::Simulation>& self) {
    return self->getState();
   })

  .def("getState", [](std::shared_ptr<rai::Simulation>& self, const ptr<rai::SimulationState>& state) {
    return self->restoreState(state);
   })

  .def("setState", [](std::shared_ptr<rai::Simulation>& self, const pybind11::array& frameState, const pybind11::array& frameVelocities) {
    arr X = numpy2arr<double>(frameState);
    X.reshape(X.N/7, 7);
    arr V = numpy2arr<double>(frameVelocities);
    V.reshape(V.N/6, 2, 3);
    return self->setState(X, V);
   }, "",
     pybind11::arg("frameState"),
     pybind11::arg("frameVelocities") = std::vector<double>()
   )

  .def("pushConfigurationToSimulator", [](std::shared_ptr<rai::Simulation>& self, const std::vector<double>& frameVelocities) {
    return self->pushConfigurationToSimulator(conv_stdvec2arr(frameVelocities));
  }, "after you modified the configuration (from which you derived the RySimulation), use this to push the modified configuration back into the simulation engine.\n Optionally: with frameVelocities",
    pybind11::arg("frameVelocities") = std::vector<double>()
  )

  .def("depthData2pointCloud", [](std::shared_ptr<rai::Simulation>& self, const pybind11::array_t<float>& depth, const std::vector<double>& Fxypxy) {
      arr points;
      floatA _depth = numpy2arr<float>(depth);
      depthData2pointCloud(points, _depth, arr(Fxypxy));
      return pybind11::array(points.dim(), points.p);
    })


//  .def("getSegmentation", [](std::shared_ptr<rai::Simulation>& self) {
//    byteA seg;
//    self->getSegmentation(seg);
//    return pybind11::array_t<byte>(seg.dim(), seg.p);
//  })

  ;
}

#endif
