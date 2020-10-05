/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Operate.h"
#include "ry-Config.h"
#include "types.h"

#include "../Operate/robotOperation.h"

void init_Operate(pybind11::module& m) {
  pybind11::class_<ry::RyOperate>(m, "RyOperate")
  .def("move", [](ry::RyOperate& self, const std::vector<std::vector<double>>& poses, const std::vector<double>& times, bool append) {
    arr path(poses.size(), poses[0].size());
    for(uint i=0; i<path.d0; i++) path[i] = poses[i];
    self.R->move(path, arr(times, true), append);
  })

  .def("move", [](ry::RyOperate& self, const pybind11::array_t<double>& path, const std::vector<double>& times, bool append) {
    arr _path = numpy2arr(path);
    self.R->move(_path, arr(times, true), append);
  })

  .def("moveHard", [](ry::RyOperate& self, const pybind11::array_t<double>& pose) {
    arr _pose = numpy2arr(pose);
    self.R->moveHard(_pose);
  })

  .def("timeToGo", [](ry::RyOperate& self) {
    return self.R->timeToGo();
  })

  .def("wait", [](ry::RyOperate& self) {
    return self.R->wait();
  })

  .def("getJointPositions", [](ry::RyOperate& self, shared_ptr<rai::Configuration>& C) {
    arr q = self.R->getJointPositions();
    return pybind11::array(q.dim(), q.p);
  })

  .def("getGripperGrabbed", [](ry::RyOperate& self, const std::string& whichArm) {
    return self.R->getGripperGrabbed(whichArm);
  })

  .def("getGripperOpened", [](ry::RyOperate& self, const std::string& whichArm) {
    return self.R->getGripperOpened(whichArm);
  })

  .def("sendToReal", [](ry::RyOperate& self, bool activate) {
    self.R->sendToReal(activate);
  })

  .def("sync", [](ry::RyOperate& self, shared_ptr<rai::Configuration>& C) {
    self.R->sync(*C);
  })
  ;
}

#endif
