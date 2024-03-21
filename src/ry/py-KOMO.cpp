/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "py-Config.h"
#include "../KOMO/komo.h"
#include "../KOMO/skeleton.h"

//#include "../LGP/bounds.h"
#include "../Kin/viewer.h"

void init_KOMO(pybind11::module& m) {
  pybind11::class_<KOMO, std::shared_ptr<KOMO>>(m, "KOMO", "A framework to define manipulation problems (IK, path optimization, sequential manipulation) as Nonlinear Mathematical Program (NLP). The actual NLP_Solver class is separate. (KOMO = k-order Markov Optimization) -- see https://marctoussaint.github.io/robotics-course/tutorials/1c-komo.html")

      .def(pybind11::init<>(), "[deprecated] please use the other constructor")

      .def(pybind11::init<const rai::Configuration&, double, uint, uint, bool>(), "constructor"
           "\n* config: the configuration, which is copied once (for IK) or many times (for waypoints/paths) to be the optimization variable"
           "\n* phases: the number P of phases (which essentially defines the real-valued interval [0,P] over which objectives can be formulated)"
           "\n* slicesPerPhase: the discretizations per phase -> in total we have phases*slicesPerPhases configurations which form the path and over which we optimize"
           "\n* kOrder: the 'Markov-order', i.e., maximal tuple of configurations over which we formulate features (e.g. take finite differences)"
           "\n* enableCollisions: if True, KOMO runs a broadphase collision check (using libFCL) in each optimization step -- only then accumulative collision/penetration features will correctly evaluate to non-zero. But this is costly.",
           pybind11::arg("config"),
           pybind11::arg("phases"),
           pybind11::arg("slicesPerPhase"),
           pybind11::arg("kOrder"),
           pybind11::arg("enableCollisions")
          )

      .def("setConfig", &KOMO::setConfig, "[deprecated] please set directly in constructor",
           pybind11::arg("config"),
           pybind11::arg("enableCollisions")
          )

      .def("setTiming", &KOMO::setTiming, "[deprecated] please set directly in constructor",
           pybind11::arg("phases"),
           pybind11::arg("slicesPerPhase"),
           pybind11::arg("durationPerPhase"),
           pybind11::arg("kOrder")
          )

      .def("addTimeOptimization", &KOMO::addTimeOptimization)

      .def("clearObjectives", &KOMO::clearObjectives)

  .def("addObjective", [](std::shared_ptr<KOMO>& self, const arr& times, const FeatureSymbol& feature, const StringA& frames, const ObjectiveType& type, const arr& scale, const arr& target, int order) {
    self->addObjective(times, feature, frames, type, scale, target, order);
  }, "central method to define objectives in the KOMO NLP:"
  "\n* times: the time intervals (subset of configurations in a path) over which this feature is active (irrelevant for IK)"
  "\n* feature: the feature symbol (see advanced `Feature` tutorial)"
  "\n* frames: the frames for which the feature is computed, given as list of frame names"
  "\n* type: whether this is a sum-of-squares (sos) cost, or eq or ineq constraint"
  "\n* scale: the matrix(!) by which the feature is multiplied"
  "\n* target: the offset which is substracted from the feature (before scaling)",
  pybind11::arg("times"),
  pybind11::arg("feature"),
  pybind11::arg("frames"),
  pybind11::arg("type"),
  pybind11::arg("scale")=arr(),
  pybind11::arg("target")=arr(),
  pybind11::arg("order")=-1)

  .def("addQuaternionNorms", &KOMO::addQuaternionNorms, "", pybind11::arg("times")=arr(), pybind11::arg("scale")=3., pybind11::arg("hard")=true)

  .def("addControlObjective", &KOMO::addControlObjective, "\n* times: (as for `addObjective`) the phase-interval in which this objective holds; [] means all times"
       "\n* order: Do we penalize the jointState directly (order=0: penalizing sqr distance to qHome, order=1: penalizing sqr distances between consecutive configurations (velocities), order=2: penalizing accelerations across 3 configurations)"
       "\n* scale: as usual, but modulated by a factor 'sqrt(delta t)' that somehow ensures total control costs in approximately independent of the choice of stepsPerPhase",
       pybind11::arg("times"), pybind11::arg("order"), pybind11::arg("scale")=1.,
       pybind11::arg("target")=arr(), pybind11::arg("deltaFromSlice")=0, pybind11::arg("deltaToSlice")=0)

  .def("addModeSwitch", &KOMO::addModeSwitch, "", pybind11::arg("times"), pybind11::arg("newMode"), pybind11::arg("frames"), pybind11::arg("firstSwitch")=true)

  .def("addStableFrame", [](std::shared_ptr<KOMO>& self, rai::JointType jointType, const char* parent, const char* name, const char* initFrame) {
    rai::Frame* f = self->addStableFrame(jointType, parent, name, initFrame);
    return std::shared_ptr<rai::Frame>(f, [](auto*){}); // giving it a no-op deleter!
  }, "complicated...",
  pybind11::arg("jointType"), pybind11::arg("parent"), pybind11::arg("name"), pybind11::arg("init")=0)

  //-- initialize (=set state)

  .def("initOrg", &KOMO::initOrg, "")
  .def("initRandom", &KOMO::initRandom, "", pybind11::arg("verbose")=0)
  .def("initWithConstant", &KOMO::initWithConstant, "", pybind11::arg("q"))
  .def("initWithPath", &KOMO::initWithPath_qOrg, "", pybind11::arg("q"))
  .def("initWithWaypoints", &KOMO::initWithWaypoints, "",
       pybind11::arg("waypoints"), pybind11::arg("waypointSlicesPerPhase")=1, pybind11::arg("interpolate")=false, pybind11::arg("qHomeInterpolate")=0., pybind11::arg("verbose")=-1)
  .def("initPhaseWithDofsPath", &KOMO::initPhaseWithDofsPath, "", pybind11::arg("t_phase"), pybind11::arg("dofIDs"), pybind11::arg("path"), pybind11::arg("autoResamplePath")=false)

  //-- solve

  .def("nlp", &KOMO::nlp, "return the problem NLP")

  // .def("optimize", [](std::shared_ptr<KOMO>& self, double addInitializationNoise) {
  //  self->optimize(addInitializationNoise);
  //   }, "",
  //   pybind11::arg("addInitializationNoise")=0.01)

  // //-- reinitialize with configuration
  // .def("setConfigurations", [](std::shared_ptr<KOMO>& self, std::shared_ptr<rai::Configuration>& C) {
  //  arr X = C->getFrameState();
  //  for(uint t=0; t<self->T; t++) {
  //    self->pathConfig.setFrameState(X, self->timeSlices[t]);
  //  }
  //   })

  //-- read out

  .def("getT", [](std::shared_ptr<KOMO>& self) { return self->T; })

  .def("getFrameState", &KOMO::getConfiguration_X)
  .def("getPath", &KOMO::getPath_qOrg)
  .def("getPath_qAll",  &KOMO::getPath_qAll)
  .def("getPathFrames", &KOMO::getPath_X)
  .def("getPathTau", &KOMO::getPath_tau)

  .def("getForceInteractions", [](std::shared_ptr<KOMO>& self) {
    rai::Graph G = self->pathConfig.reportForces();
    return graph2list(G);
  })

  .def("report", &KOMO::report,
//         [](std::shared_ptr<KOMO>& self, bool specs, bool listObjectives, bool plotOverTime) {
//          rai::Graph G = self->report(specs, listObjectives, plotOverTime);
//          return graph2dict(G);
//        },
       "returns a dict with full list of features, optionally also on problem specs and plotting costs/violations over time",
       pybind11::arg("specs") = false,
       pybind11::arg("listObjectives") = true,
       pybind11::arg("plotOverTime") = false)

  .def("getFeatureNames", [](std::shared_ptr<KOMO>& self) {
    return self->featureNames;
  }, "returns a long list of features (per time slice!), to be used by an NLP_Solver")

  .def("reportProblem", [](std::shared_ptr<KOMO>& self) {
    str s;
    s <<self->report(true, false);
    return pybind11::str(s.p, s.N);
  })

  //-- update
  .def("updateRootObjects", &KOMO::updateRootObjects,
       "update root frames (without parents) within all KOMO configurations", pybind11::arg("config"))

  .def("getSubProblem", [](std::shared_ptr<KOMO>& self, uint phase) {
    auto C = make_shared<rai::Configuration>();
    arr q0, q1;
    self->getSubProblem(phase, *C, q0, q1);
    return pybind11::make_tuple(C, arr2numpy(q0), arr2numpy(q1));
  },
  "return a tuple of (configuration, start q0, end q1) for given phase of this komo problem",
  pybind11::arg("phase"))

  //-- display

  .def("view", &KOMO::view, "", pybind11::arg("pause") = false, pybind11::arg("txt") = nullptr)
  .def("view_play", &KOMO::view_play, "", pybind11::arg("pause") = false, pybind11::arg("delay") = 0.1, pybind11::arg("saveVideoPath") = nullptr)
  .def("view_close",  &KOMO::view_close)

  ;

  //===========================================================================

  //  pybind11::class_<ry::ConfigViewer>(m, "ConfigViewer");
  pybind11::class_<Objective, std::shared_ptr<Objective>>(m, "KOMO_Objective");

}

#endif
