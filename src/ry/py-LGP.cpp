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
      .def("getSolvedPlan", &rai::LGP_Tool::getSolvedPlan, "return list of discrete decisions of current solution")
      .def("getSolvedKOMO", &rai::LGP_Tool::getSolvedKOMO, "return the solved KOMO object (including its continuous solution) of current solution")

      .def("get_piecewiseMotionProblem", &rai::LGP_Tool::get_piecewiseMotionProblem, "return the (unsolved) KOMO object corresponding to the k-th piece of the current solution", pybind11::arg("phase"), pybind11::arg("fixEnd"))
      .def("get_fullMotionProblem", &rai::LGP_Tool::get_fullMotionProblem, "return the (unsolved) KOMO object corresponding to the full joint motion problem spanning all steps", pybind11::arg("initWithWaypoints"))

      .def("solvePiecewiseMotions", &rai::LGP_Tool::solvePiecewiseMotions, "solve full motion of current solution and return the (solved) KOMO object", pybind11::arg("verbose")=1)
      .def("solveFullMotion", &rai::LGP_Tool::solveFullMotion, "solve full motion of current solution and return the (solved) KOMO object", pybind11::arg("verbose")=1)


      .def("view_solved", &rai::LGP_Tool::view_solved, "view last computed solution", pybind11::arg("pause"))
      .def("view_close", &rai::LGP_Tool::view_close, "")

  ;

  m.def("default_TAMP_Provider", &rai::default_TAMP_Provider, "", pybind11::arg("C"), pybind11::arg("lgp_config_file"));
  m.def("default_Logic2KOMO_Translator", &rai::default_Logic2KOMO_Translator, "");

}

//===========================================================================

/*struct TAMP_Provider{
  virtual ~TAMP_Provider() {}
  virtual Array<StringA> getNewPlan() = 0;
  virtual Configuration& getConfig() = 0;
  virtual StringA explicitCollisions() = 0;
};*/


/*struct PyLogic2KOMO_Translator : rai::Logic2KOMO_Translator {
  pybind11::object py_obj; //the python object implementing the class

  PyLogic2KOMO_Translator(pybind11::object py_obj) : py_obj(py_obj){
    dimension = py_obj.attr("getDimension")().cast<int>();
    featureTypes = rai::convert<ObjectiveType>( list2arr<int>(py_obj.attr("getFeatureTypes")()) );
    bounds = numpy2arr<double>(py_obj.attr("getBounds")());
  }

  virtual std::shared_ptr<KOMO> setup_sequence(rai::Configuration& C, uint K){
    pybind11::object _ret = py_obj.attr("setup_sequence")(C, K);
    auto phiJ = _phiJ.cast< std::tuple< pybind11::array_t<double>, pybind11::array_t<double> > >();
    LOG(0) <<"before";
    phi = numpy2arr(std::get<0>(phiJ));
    J = numpy2arr(std::get<1>(phiJ));
    LOG(0) <<"size:" <<phi.dim();

  }

  virtual void add_action_constraints(std::shared_ptr<KOMO>& komo, double time, const StringA& action) = 0;
  virtual void add_action_constraints_motion(std::shared_ptr<KOMO>& komo, double time, const StringA& prev_action, const StringA& action) = 0;

};*/

#endif
