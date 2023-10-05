/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "ry-Config.h"
#include "../KOMO/komo.h"
#include "../KOMO/skeleton.h"

//#include "../LGP/bounds.h"
#include "../Kin/viewer.h"

void init_KOMO(pybind11::module& m) {
  pybind11::class_<KOMO, std::shared_ptr<KOMO>>(m, "KOMO", "Constrained solver to optimize configurations or paths. (KOMO = k-order Markov Optimization)")

    .def(pybind11::init<>())
    .def(pybind11::init<const rai::Configuration&, double, uint, uint, bool>(), "",
         pybind11::arg("config"),
         pybind11::arg("phases"),
         pybind11::arg("slicesPerPhase"),
         pybind11::arg("kOrder"),
         pybind11::arg("enableCollisions")
         )

    .def("setConfig", &KOMO::setConfig, "",
         pybind11::arg("config"),
         pybind11::arg("enableCollisions")
         )

    .def("setTiming", &KOMO::setTiming, "",
         pybind11::arg("phases"),
         pybind11::arg("slicesPerPhase"),
         pybind11::arg("durationPerPhase"),
         pybind11::arg("kOrder")
         )

    .def("addTimeOptimization", &KOMO::addTimeOptimization)

    .def("clearObjectives", &KOMO::clearObjectives)

    .def("addObjective", [](std::shared_ptr<KOMO>& self, const arr& times, const FeatureSymbol& feature, const std::vector<std::string>& frames, const ObjectiveType& type, const arr& scale, const arr& target, int order) {
	self->addObjective(times, feature, strvec2StringA(frames), type, scale, target, order);
      }, "",
      pybind11::arg("times"),
      pybind11::arg("feature"),
      pybind11::arg("frames")=std::vector<std::string>(),
      pybind11::arg("type"),
      pybind11::arg("scale")=arr(),
      pybind11::arg("target")=arr(),
      pybind11::arg("order")=-1)

    .def("addQuaternionNorms", &KOMO::addQuaternionNorms, "", pybind11::arg("times")=arr(), pybind11::arg("scale")=3., pybind11::arg("hard")=true )

    .def("addControlObjective", &KOMO::addControlObjective, "", pybind11::arg("times"), pybind11::arg("order"), pybind11::arg("scale")=1.,
         pybind11::arg("target")=arr(), pybind11::arg("deltaFromSlice")=0, pybind11::arg("deltaToSlice")=0 )

    .def("addModeSwitch", &KOMO::addModeSwitch, "", pybind11::arg("times"), pybind11::arg("newMode"), pybind11::arg("frames"), pybind11::arg("firstSwitch")=true )
    .def("addInteraction_elasticBounce", &KOMO::addContact_elasticBounce, "", pybind11::arg("time"), pybind11::arg("from"), pybind11::arg("to"),
	 pybind11::arg("elasticity") = .8, pybind11::arg("stickiness") = 0. )

    //-- initialize (=set state)

    .def("initOrg", &KOMO::initOrg, "" )
    .def("initRandom", &KOMO::initRandom, "", pybind11::arg("verbose")=0 )
    .def("initWithConstant", &KOMO::initWithConstant, "", pybind11::arg("q") )
    .def("initWithPath_qOrg", &KOMO::initWithPath_qOrg, "", pybind11::arg("q") )
    .def("initWithWaypoints", &KOMO::initWithWaypoints, "", pybind11::arg("waypoints"), pybind11::arg("waypointSlicesPerPhase")=1, pybind11::arg("interpolate")=false, pybind11::arg("verbose")=-1 )
    .def("initPhaseWithDofsPath", &KOMO::initPhaseWithDofsPath, "", pybind11::arg("t_phase"), pybind11::arg("dofIDs"), pybind11::arg("path"), pybind11::arg("autoResamplePath")=false )

    //-- solve

    .def("nlp", &KOMO::nlp, "return the problem NLP")

    // .def("optimize", [](std::shared_ptr<KOMO>& self, double addInitializationNoise) {
    // 	self->optimize(addInitializationNoise);
    //   }, "",
    //   pybind11::arg("addInitializationNoise")=0.01)

    // //-- reinitialize with configuration
    // .def("setConfigurations", [](std::shared_ptr<KOMO>& self, shared_ptr<rai::Configuration>& C) {
    // 	arr X = C->getFrameState();
    // 	for(uint t=0; t<self->T; t++) {
    // 	  self->pathConfig.setFrameState(X, self->timeSlices[t]);
    // 	}
    //   })

    //-- read out

    .def("getT", [](std::shared_ptr<KOMO>& self) { return self->T; } )

    .def("getFrameState", &KOMO::getConfiguration_X)
    .def("getPath", &KOMO::getPath_qOrg)
    .def("getPath_qAll",  &KOMO::getPath_qAll)
    .def("getPathFrames", &KOMO::getPath_X)
    .def("getPathTau", &KOMO::getPath_tau)

    .def("getForceInteractions", [](std::shared_ptr<KOMO>& self) {
	rai::Graph G = self->pathConfig.reportForces();
	return graph2list(G);
      })

    .def("report", [](std::shared_ptr<KOMO>& self, bool plotOverTime) {
        rai::Graph R = self->report(plotOverTime);
        return graph2dict(R);
      },
      "returns a dict with full report on features, optionally plotting costs/violations over time",
      pybind11::arg("plotOverTime") = false)

    .def("getReport", [](std::shared_ptr<KOMO>& self, bool plotOverTime) {
        rai::Graph R = self->getReport(plotOverTime);
	return graph2dict(R);
      })

    .def("getFeatureNames", [](std::shared_ptr<KOMO>& self) {
        return self->featureNames;
     }, "returns a long list of features (per time slice!), to be used by an NLP_Solver")

    .def("reportProblem", [](std::shared_ptr<KOMO>& self) {
        rai::String str;
	self->reportProblem(str);
	return pybind11::str(str.p, str.N);
      })


    //-- update
    .def("updateRootObjects", &KOMO::updateRootObjects,
         "update root frames (without parents) within all KOMO configurations", pybind11::arg("config"))

    //-- display

    .def("view", &KOMO::view, "", pybind11::arg("pause") = false, pybind11::arg("txt") = nullptr)
    .def("view_play", &KOMO::view_play, "", pybind11::arg("pause") = false, pybind11::arg("delay") = 0.1, pybind11::arg("saveVideoPath") = nullptr)
    .def("view_close",  &KOMO::view_close)

    ;


  //===========================================================================

  //  pybind11::class_<ry::ConfigViewer>(m, "ConfigViewer");
  pybind11::class_<Objective, shared_ptr<Objective>>(m, "KOMO_Objective");

}

#endif
