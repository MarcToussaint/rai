/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Algo/spline.h"

void init_Spline(pybind11::module& m) {

  //===========================================================================

  pybind11::class_<rai::BSpline, shared_ptr<rai::BSpline>>(m, "BSpline", "")

      .def(pybind11::init<>(), "non-initialized")

      .def("setKnots", &rai::BSpline::setKnots,
           "set degree and knots by providing *times* (e.g. uniform linspace(0,1,T) -- duplicated knots at start/end and inter-time placing for even degrees is done internally",
           pybind11::arg("degree"),
           pybind11::arg("times"))
      .def("getBmatrix", &rai::BSpline::getBmatrix,
           "return the B-matrix mapping from ctrlPoints to (e.g. finer) sampleTimes (e.g. uniform linspace(0,1,T)",
           pybind11::arg("sampleTimes"),
           pybind11::arg("startDuplicates")=false,
           pybind11::arg("endDuplicates")=false)
      .def("setCtrlPoints", &rai::BSpline::setCtrlPoints, "set the ctrl points, automatically duplicating them as needed at start/end, optionally setting vels at start/end",
           pybind11::arg("points"),
           pybind11::arg("addStartDuplicates")=true,
           pybind11::arg("addEndDuplicates")=true,
           pybind11::arg("setStartVel")=arr{},
           pybind11::arg("setEndVel")=arr{})
      .def("eval", pybind11::overload_cast<const arr&, uint>(&rai::BSpline::eval, pybind11::const_),
           "evaluate the spline (or its derivative) for given sampleTimes",
           pybind11::arg("sampleTimes"),
           pybind11::arg("derivative")=0)
      .def("getKnots", &rai::BSpline::getKnots)
      .def("getCtrlPoints", &rai::BSpline::getCtrlPoints)

      ;

  //===========================================================================


}

#endif
