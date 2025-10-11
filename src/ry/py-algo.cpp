/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "py-Config.h"

#include "../Algo/trilinear.h"
#include "../Algo/marching_cubes.h"

void init_algo(pybind11::module& m) {

  pybind11::module_ mod = m.def_submodule("algo", "basic algorithmic methods");

  mod.def("trilinear_interpolate", &rai::trilinear_interpolate<float>, "use trilinear interpolation to sample values from a 3D grid of values",
          pybind11::arg("pts"),
          pybind11::arg("grid_values"),
          pybind11::arg("grid_res"));

  mod.def("marching_cubes", &rai::marching_cubes, "use Lewiner's original Marching Cubes algorithm compute a zero-levelset mesh from a 3D tensor. Returned vertices are centered and assume given total box size",
          pybind11::arg("grid_values"),
          pybind11::arg("size"));


  mod.def("box_filter", [](const floatA& grid_values, uint width) { floatA x=integral(grid_values); return differencing(x, width); }, "apply a box filter of given width (typically 3) -- use multiple times to approximate Gaussian filter",
          pybind11::arg("grid_values"),
          pybind11::arg("width")=3);

}

#endif
