/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-Camera.h"
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
    return Array2numpy<byte>(rgb);
  })

  .def("getDepth", [](ry::RyCamera& self) {
    floatA depth = self.depth.get();
    return Array2numpy<float>(depth);
  })

  .def("getPoints", [](ry::RyCamera& self, const std::vector<double>& fxycxy) {
    floatA _depth = self.depth.get();
    arr _points;
    CHECK_EQ(fxycxy.size(), 4, "I need 4 intrinsic calibration parameters")
    depthData2pointCloud(_points, _depth, fxycxy[0], fxycxy[1], fxycxy[2], fxycxy[3]);
    return arr2numpy(_points);
  })

  .def("transform_image2world", [](ry::RyCamera& self, const std::vector<double>& pt, const char* cameraFrame, const std::vector<double>& fxycxy) {
    NIY
//    arr _pt = pt;
//    depthData2point(_pt, conv_stdvec2arr(fxycxy)); //transforms the point to camera xyz coordinates

//    pcl->X.applyOnPoint(pt); //transforms into world coordinates

//    return pybind11::array_t<double>(_pt.dim(), _pt.p);
  })
  ;
}

#endif
