/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../LGP/LGP_Tool.h"

void init_LGP(pybind11::module& m) {

  pybind11::class_<rai::TAMP_Provider, shared_ptr<rai::TAMP_Provider>>(m, "TAMP_Provider", "TAMP_Provider");
  pybind11::class_<rai::Logic2KOMO_Translator, shared_ptr<rai::Logic2KOMO_Translator>>(m, "Logic2KOMO_Translator", "Logic2KOMO_Translator");

  pybind11::class_<rai::LGP_Tool, shared_ptr<rai::LGP_Tool>>(m, "LGP_Tool", "Tools to compute things (and solve) a Task-and-Motion Planning problem formulated as Logic-Geometric Program")

      .def(pybind11::init<rai::Configuration&, rai::TAMP_Provider&, rai::Logic2KOMO_Translator&>(), "initialization")

      .def("solve", &rai::LGP_Tool::solve, "compute new solution", pybind11::arg("verbose")=1)
      .def("getSolvedPlan", &rai::LGP_Tool::getSolvedPlan, "return list of discrete decisions of last solution")
      .def("getSolvedKOMO", &rai::LGP_Tool::getSolvedKOMO, "return KOMO object (including its continuous solution) of last solution")
      .def("view_solved", &rai::LGP_Tool::view_solved, "view last computed solution", pybind11::arg("pause"))
      .def("view_close", &rai::LGP_Tool::view_close, "")

  ;

  m.def("default_TAMP_Provider", &rai::default_TAMP_Provider, "", pybind11::arg("C"), pybind11::arg("lgp_config_file"));
  m.def("default_Logic2KOMO_Translator", &rai::default_Logic2KOMO_Translator, "");

}

#endif
