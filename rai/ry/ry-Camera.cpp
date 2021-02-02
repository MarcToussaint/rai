/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry-Camera.h"
#include "types.h"
#include "../RosCom/rosCamera.h"
#include "../Geo/depth2PointCloud.h"

void init_Camera(pybind11::module& m) {
  pybind11::class_<ry::RyCamera>(m, "Camera")
  .def(pybind11::init<const char*, const char*, const char*, bool>()
       , "", pybind11::arg("rosNodeName"),
       pybind11::arg("rgb_topic"),
       pybind11::arg("depth_topic"),
       pybind11::arg("useUint") = false)

  .def("getRgb", [](ry::RyCamera& self) {
    byteA rgb = self.rgb.get();
    return pybind11::array_t<byte>(rgb.dim(), rgb.p);
  })

  .def("getDepth", [](ry::RyCamera& self) {
    floatA depth = self.depth.get();
    return pybind11::array_t<float>(depth.dim(), depth.p);
  })

  .def("getPoints", [](ry::RyCamera& self, const std::vector<double>& Fxypxy) {
    floatA _depth = self.depth.get();
    arr _points;
    CHECK_EQ(Fxypxy.size(), 4, "I need 4 intrinsic calibration parameters")
    depthData2pointCloud(_points, _depth, Fxypxy[0], Fxypxy[1], Fxypxy[2], Fxypxy[3]);
    return pybind11::array_t<double>(_points.dim(), _points.p);
  })

  .def("transform_image2world", [](ry::RyCamera& self, const std::vector<double>& pt, const char* cameraFrame, const std::vector<double>& Fxypxy) {
    NIY
//    arr _pt = pt;
//    depthData2point(_pt, conv_stdvec2arr(Fxypxy)); //transforms the point to camera xyz coordinates

//    pcl->X.applyOnPoint(pt); //transforms into world coordinates

//    return pybind11::array_t<double>(_pt.dim(), _pt.p);
  })
  ;
}

#endif
