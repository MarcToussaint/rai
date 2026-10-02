/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../Perception/calibration.h"
#include "../Perception/aruco.h"

void init_calib(pybind11::module& m) {

    pybind11::module_ mod = m.def_submodule("calib", "basic calibration methods");

    mod.def("undistort_images", &rai::undistort_images, "", pybind11::arg("C"), pybind11::arg("imgs"));
    mod.def("render_images", &rai::render_images, "", pybind11::arg("C"), pybind11::arg("qs"), pybind11::arg("T")=0);

    mod.def("detect_arucos", &rai::detect_arucos, "", pybind11::arg("imgs"), pybind11::arg("verbose")=0);
    mod.def("calibrate_intrinsics", &rai::calibrate_intrinsics, "",
            pybind11::arg("imgs"),
            pybind11::arg("distortionDofs"),
            pybind11::arg("verbose")=2,
            pybind11::arg("square_len_m")=0.055,
            pybind11::arg("marker_len_m")=0.041);

    mod.def("komo_calibrate", &rai::komo_calibrate, "",
            pybind11::arg("C"),
            pybind11::arg("ids"),
            pybind11::arg("pts"),
            pybind11::arg("qs"),
            pybind11::arg("exclude_times") = intA{},
            pybind11::arg("calibrate_cams") = true,
            pybind11::arg("calibrate_arucos") = true,
            pybind11::arg("calibrate_joints") = true,
            pybind11::arg("calibrate_objPoses") = false,
            pybind11::arg("calib_joint_regularization") = 1e1,
            pybind11::arg("verbose") = 1);

}

#endif
