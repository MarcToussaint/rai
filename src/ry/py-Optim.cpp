/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include "../Optim/NLP_Factory.h"
#include "../Optim/NLP_Solver.h"
#include "../Optim/NLP_Sampler.h"
#include "../KOMO/testProblems_KOMO.h"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

/* Explanation: The python user implement a derivation of the nlp.NLP template class, which is defined in the separate nlp.py
 * that ensures that the class has methods "getDimension, getFeatureTypes, getBounds, evaluate, report"
 * that class is passed to setPyProblem
 * that creates a PyNLP container which hosts the python class as py_nlp and implements the C++ rai::NLP
 */
struct PyNLP : NLP {
  pybind11::object py_nlp; //the python object implementing the NLP
  
  PyNLP(pybind11::object py_nlp) : py_nlp(py_nlp){
    if(!pybind11::hasattr(py_nlp, "dimension")) { HALT("PyNLP needs a dimension member (integer), defining the dimension of the decision variable"); }
    if(!pybind11::hasattr(py_nlp, "featureTypes")) { HALT("PyNLP needs a featureTypes member (list of OT), defining the ObjectiveTypes of each NLP feature"); }
    if(!pybind11::hasattr(py_nlp, "bounds")) { HALT("PyNLP needs a bounds member (2*n matrix), defining the lower and upper bounds of the decision variable"); }

    dimension = py_nlp.attr("dimension").cast<int>();
    featureTypes = rai::convert<ObjectiveType>( list2arr<int>( py_nlp.attr("featureTypes").cast<pybind11::list>() ) );
    bounds = numpy2arr<double>(py_nlp.attr("bounds").cast<pybind11::array_t<double>>());
  }

  virtual void evaluate(arr& phi, arr& J, const arr& x){
    pybind11::object _phiJ = py_nlp.attr("evaluate")(arr2numpy(x));
    auto phiJ = _phiJ.cast< std::tuple< pybind11::array_t<double>, pybind11::array_t<double> > >();
    phi = numpy2arr(std::get<0>(phiJ));
    J = numpy2arr(std::get<1>(phiJ));
    if(J.d0!=phi.N && J.d1==3){ //assume sparse triplet storage!!
      arr T = J;
      J.sparse().setFromTriplets(T, phi.N, x.N);
    }
  }

  virtual void getFHessian(arr& H, const arr& x) {
    if(pybind11::hasattr(py_nlp, "getFHessian")){
      pybind11::object _H = py_nlp.attr("getFHessian")(arr2numpy(x));
      H = numpy2arr(_H.cast<pybind11::array_t<double>>());
    }
    H.clear();
  }

  virtual arr getInitializationSample(){
    if(pybind11::hasattr(py_nlp, "getInitializationSample")){
      pybind11::object _x = py_nlp.attr("getInitializationSample")();
      return numpy2arr(_x.cast<pybind11::array_t<double>>());
    }
    LOG(-1) <<"no getInitializationSample implemented -- using default fallback";
    return NLP::getInitializationSample();
  }

  virtual void report(ostream& os, int verbose, const char* nomsg=0){
    LOG(0) <<"here";
    NLP::report(os, verbose, "(binding in py-Optim.cpp:52)");
    pybind11::object _msg = py_nlp.attr("report")(verbose);
    auto msg = _msg.cast<std::string>();
    os <<"user report: " <<msg <<endl;
  }

};

void init_Optim(pybind11::module& m) {

  //===========================================================================

  m.def("make_NLP_Problem", rai::make_NLP_Problem, "create a benchmark NLP", pybind11::arg("problem_name"));
  m.def("get_NLP_Problem_names", rai::get_NLP_Problem_names, "return all problem names");

  //===========================================================================

  pybind11::class_<NLP, shared_ptr<NLP>> __mp(m, "NLP", "A Nonlinear Mathematical Program (bindings to the c++ object - distinct from the python template nlp.NLP" );
  __mp

  .def("evaluate", [](std::shared_ptr<NLP>& self, const arr& x) {
    arr phi, J;
    self->evaluate(phi, J, x);
    return std::tuple<arr, arr>(phi, J);
  },
  "query the NLP at a point $x$; returns the tuple $(phi,J)$, which is the feature vector and its Jacobian; features define cost terms, sum-of-square (sos) terms, inequalities, and equalities depending on 'getFeatureTypes'"
      )

  .def("getFeatureTypes", [](std::shared_ptr<NLP>& self) {
    return Array2vec<ObjectiveType>(self->featureTypes);
  },
  "features (entries of $phi$) can be of one of (ry.OT.f, ry.OT.sos, ry.OT.ineq, ry.OT.eq), which means (cost, sum-of-square, inequality, equality). The total cost $f(x)$ is the sum of all f-terms plus sum-of-squares of sos-terms."
      )

  .def("getDimension", [](std::shared_ptr<NLP>& self) {
    return self->dimension;
  },
  "return the dimensionality of $x$")

  .def("getBounds", [](std::shared_ptr<NLP>& self) {
    return self->bounds;
  },
  "returns a 2xn array with lower and upper bounds as rows (or size zero, if there are no bounds)")

  .def("getInitializationSample",
       &NLP::getInitializationSample,
       "returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible"
      )

  .def("getFHessian",  [](std::shared_ptr<NLP>& self, const arr& x) {
    arr H;
    self->getFHessian(H, x);
    return H;
  },
  "returns Hessian of the sum of $f$-terms"
      )

  .def("report",  [](std::shared_ptr<NLP>& self, int verbose) {
    str s;
    self->report(s, verbose);
    return std::string(s.p);
  },
  "displays semantic information on the last query"
      )

  .def("checkJacobian", &NLP::checkJacobian, "", pybind11::arg("x"), pybind11::arg("tolerance"), pybind11::arg("featureNames")=StringA{})
  .def("checkHessian", &NLP::checkHessian, "", pybind11::arg("x"), pybind11::arg("tolerance"))


  ;

  //===========================================================================

  pybind11::class_<NLP_Factory, std::shared_ptr<NLP_Factory>>(m, "NLP_Factory", __mp)

      .def(pybind11::init<>())

      .def("setDimension", &NLP_Factory::setDimension)
      .def("setFeatureTypes", &NLP_Factory::setFeatureTypes)
      .def("setBounds", &NLP_Factory::setBounds)
      .def("setEvalCallback", &NLP_Factory::setEvalCallback2)

  .def("testCallingEvalCallback", [](std::shared_ptr<NLP_Factory>& self, const arr& x) {
    arr y, J;
    self->evaluate(y, J, x);
    return std::tuple<arr, arr>(y, J);
  })

  ;

  //===========================================================================

  //  pybind11::module_ mBench = m.def_submodule("nlp_benchmark", "ry submodule to define optimization benchmarks");
//  pybind11::class_<NLP, shared_ptr<NLP>>(m, "NLP")

  pybind11::class_<OptBench_InvKin_Endeff, std::shared_ptr<OptBench_InvKin_Endeff>>(m, "OptBenchmark_InvKin_Endeff")
      .def(pybind11::init<const char*, bool>())
      .def("get", &OptBench_InvKin_Endeff::get)
      ;

  pybind11::class_<OptBench_Skeleton_Pick, std::shared_ptr<OptBench_Skeleton_Pick>>(m, "OptBench_Skeleton_Pick")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_Pick::get)
      ;

  pybind11::class_<OptBench_Skeleton_Handover, std::shared_ptr<OptBench_Skeleton_Handover>>(m, "OptBench_Skeleton_Handover")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_Handover::get)
      ;

  pybind11::class_<OptBench_Skeleton_StackAndBalance, std::shared_ptr<OptBench_Skeleton_StackAndBalance>>(m, "OptBench_Skeleton_StackAndBalance")
      .def(pybind11::init<rai::ArgWord>())
      .def("get", &OptBench_Skeleton_StackAndBalance::get)
      ;

  //===========================================================================

  pybind11::class_<rai::OptOptions, std::shared_ptr<rai::OptOptions>>(m, "NLP_SolverOptions", "solver options")

      .def(pybind11::init<>())
#define MEMBER(type, name, x) .def("set_" #name, &rai::OptOptions::set_##name)
      MEMBER(int, verbose, 1)
      MEMBER(double, stopTolerance, 1e-2)
      MEMBER(double, stopFTolerance, -1.)
      MEMBER(double, stopGTolerance, -1.)
      MEMBER(int, stopEvals, 1000)
      MEMBER(int, stopInners, 1000)
      MEMBER(int, stopOuters, 1000)
      MEMBER(double, stepMax, .2)
      MEMBER(double, damping, 1.)
      MEMBER(double, stepInc, 1.5)
      MEMBER(double, stepDec, .5)
      MEMBER(double, wolfe, .01)
      MEMBER(double, muInit, 1.)
      MEMBER(double, muInc, 5.)
      MEMBER(double, muMax, 1e4)
      MEMBER(double, muLBInit, .1)
      MEMBER(double, muLBDec, .2)
      MEMBER(double, lambdaMax, -1.)
#undef MEMBER

  .def("dict", [](std::shared_ptr<rai::OptOptions>& self) {
    return graph2dict(rai::Graph{
#define MEMBER(type, name, x) {#name, self->name},
      MEMBER(int, verbose, 1)
      MEMBER(double, stopTolerance, 1e-2)
      MEMBER(double, stopFTolerance, -1.)
      MEMBER(double, stopGTolerance, -1.)
      MEMBER(int, stopEvals, 1000)
      MEMBER(int, stopInners, 1000)
      MEMBER(int, stopOuters, 1000)
      MEMBER(double, stepMax, .2)
      MEMBER(double, damping, 1.)
      MEMBER(double, stepInc, 1.5)
      MEMBER(double, stepDec, .5)
      MEMBER(double, wolfe, .01)
      MEMBER(double, muInit, 1.)
      MEMBER(double, muInc, 5.)
      MEMBER(double, muMax, 1e4)
      MEMBER(double, muLBInit, .1)
      MEMBER(double, muLBDec, .2)
      MEMBER(double, lambdaMax, -1.)
#undef MEMBER
    });

  })

  ;

  //===========================================================================

  pybind11::class_<NLP_Sampler, std::shared_ptr<NLP_Sampler>>(m, "NLP_Sampler", "An interface to an NLP sampler")

      .def(pybind11::init<const shared_ptr<NLP>&>(), "",
           pybind11::arg("problem")
          )

      .def("sample", &NLP_Sampler::sample, "")
      .def("setOptions", [](std::shared_ptr<NLP_Sampler>& self
    #define MEMBER(type, name, x) ,type name
           MEMBER(double, eps, .05)
           MEMBER(bool, useCentering, true)
           MEMBER(int, verbose, 1)
           MEMBER(rai::String, seedMethod, "uni")
           MEMBER(uint, seedCandidates, 10)
           MEMBER(double, penaltyMu, 1.)
           MEMBER(rai::String, downhillMethod, "GN")
           MEMBER(int, downhillMaxSteps, 50)
           MEMBER(double, slackStepAlpha, 1.)
           MEMBER(double, slackMaxStep, .1)
           MEMBER(double, slackRegLambda, 1e-2)
           MEMBER(double, ineqOverstep, -1)
           MEMBER(rai::String, downhillNoiseMethod, "none")
           MEMBER(rai::String, downhillRejectMethod, "none")
           MEMBER(double, downhillNoiseSigma, .1)
           MEMBER(rai::String, interiorMethod, "HR")
           MEMBER(int, interiorBurnInSteps, 0)
           MEMBER(int, interiorSampleSteps, 1)
           MEMBER(rai::String, interiorNoiseMethod, "iso")
           MEMBER(double, hitRunEqMargin, .1)
           MEMBER(double, interiorNoiseSigma, .5)
           MEMBER(double, langevinTauPrime, -1.)
    #undef MEMBER
      ) {
        self->opt
    #define MEMBER(type, name, x) .set_##name(name)
            MEMBER(double, eps, .05)
            MEMBER(bool, useCentering, true)
            MEMBER(int, verbose, 1)
            MEMBER(rai::String, seedMethod, "uni")
            MEMBER(uint, seedCandidates, 10)
            MEMBER(double, penaltyMu, 1.)
            MEMBER(rai::String, downhillMethod, "GN")
            MEMBER(int, downhillMaxSteps, 50)
            MEMBER(double, slackStepAlpha, 1.)
            MEMBER(double, slackMaxStep, .1)
            MEMBER(double, slackRegLambda, 1e-2)
            MEMBER(double, ineqOverstep, -1)
            MEMBER(rai::String, downhillNoiseMethod, "none")
            MEMBER(rai::String, downhillRejectMethod, "none")
            MEMBER(double, downhillNoiseSigma, .1)
            MEMBER(rai::String, interiorMethod, "HR")
            MEMBER(int, interiorBurnInSteps, 0)
            MEMBER(int, interiorSampleSteps, 1)
            MEMBER(rai::String, interiorNoiseMethod, "iso")
            MEMBER(double, hitRunEqMargin, .1)
            MEMBER(double, interiorNoiseSigma, .5)
            MEMBER(double, langevinTauPrime, -1.)
    #undef MEMBER
        ;
        return self;
      }, "set solver options"
    #define MEMBER(type, name, x) , pybind11::arg(#name) = x
  MEMBER(double, eps, .05)
  MEMBER(bool, useCentering, true)
  MEMBER(int, verbose, 1)
  MEMBER(rai::String, seedMethod, "uni")
  MEMBER(uint, seedCandidates, 10)
  MEMBER(double, penaltyMu, 1.)
  MEMBER(rai::String, downhillMethod, "GN")
  MEMBER(int, downhillMaxSteps, 50)
  MEMBER(double, slackStepAlpha, 1.)
  MEMBER(double, slackMaxStep, .1)
  MEMBER(double, slackRegLambda, 1e-2)
  MEMBER(double, ineqOverstep, -1)
  MEMBER(rai::String, downhillNoiseMethod, "none")
  MEMBER(rai::String, downhillRejectMethod, "none")
  MEMBER(double, downhillNoiseSigma, .1)
  MEMBER(rai::String, interiorMethod, "HR")
  MEMBER(int, interiorBurnInSteps, 0)
  MEMBER(int, interiorSampleSteps, 1)
  MEMBER(rai::String, interiorNoiseMethod, "iso")
  MEMBER(double, hitRunEqMargin, .1)
  MEMBER(double, interiorNoiseSigma, .5)
  MEMBER(double, langevinTauPrime, -1.)
    #undef MEMBER
          )
      ;
  //===========================================================================

  pybind11::class_<rai::NLP_Solver, std::shared_ptr<rai::NLP_Solver>>(m, "NLP_Solver", "An interface to portfolio of solvers")

      .def(pybind11::init<>())
      .def(pybind11::init<const shared_ptr<NLP>&, int>(), "",
           pybind11::arg("problem"),
           pybind11::arg("verbose")=0
          )

      .def_readwrite("x", &rai::NLP_Solver::x)
      .def_readwrite("dual", &rai::NLP_Solver::dual)

      .def("setSolver", &rai::NLP_Solver::setSolver, "")
      .def("setProblem", &rai::NLP_Solver::setProblem, "")
      .def("setInitialization", &rai::NLP_Solver::setInitialization, "")

      .def("setTracing", &rai::NLP_Solver::setTracing, "")
      .def("solve", &rai::NLP_Solver::solve,
           "resampleInitialization=-1 means: only when not already solved",
           pybind11::arg("resampleInitialization")=-1, pybind11::arg("verbose")=-100)

      .def("getProblem", &rai::NLP_Solver::getProblem, "returns the NLP problem")
      .def("getTrace_x", &rai::NLP_Solver::getTrace_x, "returns steps-times-n array with queries points in each row")
      .def("getTrace_costs", &rai::NLP_Solver::getTrace_costs, "returns steps-times-3 array with rows (f+sos-costs, ineq, eq)")
      .def("getTrace_phi", &rai::NLP_Solver::getTrace_phi, "")
      .def("getTrace_J", &rai::NLP_Solver::getTrace_J, "")

  .def("reportLagrangeGradients", [](std::shared_ptr<rai::NLP_Solver>& self, const StringA& featureNames) {
    rai::Graph R = self->reportLagrangeGradients(featureNames);
    return graph2dict(R);
  },"return dictionary of Lagrange gradients per objective",
    pybind11::arg("featureNames")=StringA{}
  )

  .def("setPyProblem", [](std::shared_ptr<rai::NLP_Solver>& self, pybind11::object py_nlp){
    std::shared_ptr<NLP> nlp = std::dynamic_pointer_cast<NLP>( std::make_shared<PyNLP>(py_nlp) );
    self->setProblem(nlp);
  })

  .def("getOptions", [](std::shared_ptr<rai::NLP_Solver>& self) { return self->opt; })
  .def("setOptions", [](std::shared_ptr<rai::NLP_Solver>& self
#define MEMBER(type, name, x) ,type name
  MEMBER(int, verbose, 1)
  MEMBER(double, stopTolerance, 1e-2)
  MEMBER(double, stopFTolerance, -1.)
  MEMBER(double, stopGTolerance, -1.)
  MEMBER(int,    stopEvals, 1000)
  MEMBER(int,    stopInners, 1000)
  MEMBER(int,    stopOuters, 1000)
  MEMBER(int,    stopLineSteps, 10)
  MEMBER(int,    stopTinySteps, 4)
  MEMBER(double, stepInit, 1.)
  MEMBER(double, stepMin, -1.)
  MEMBER(double, stepMax, .2)
  MEMBER(double, stepInc, 1.5)
  MEMBER(double, stepDec, .5)
  MEMBER(double, damping, 1.)
  MEMBER(double, wolfe, .01)
  MEMBER(double, muInit, 1.)
  MEMBER(double, muInc, 5.)
  MEMBER(double, muMax, 1e4)
  MEMBER(double, muLBInit, .1)
  MEMBER(double, muLBDec, .2)
  MEMBER(double, lambdaMax, -1.)
  MEMBER(double, interiorPadding, 1e-2)
  MEMBER(bool,   boundedNewton, true)
  MEMBER(double, finiteDifference, -1.)
#undef MEMBER
  ) {
#define MEMBER(type, name, x) self->opt->name = name;
  MEMBER(int, verbose, 1)
  MEMBER(double, stopTolerance, 1e-2)
  MEMBER(double, stopFTolerance, -1.)
  MEMBER(double, stopGTolerance, -1.)
  MEMBER(int,    stopEvals, 1000)
  MEMBER(int,    stopInners, 1000)
  MEMBER(int,    stopOuters, 1000)
  MEMBER(int,    stopLineSteps, 10)
  MEMBER(int,    stopTinySteps, 4)
  MEMBER(double, stepInit, 1.)
  MEMBER(double, stepMin, -1.)
  MEMBER(double, stepMax, .2)
  MEMBER(double, stepInc, 1.5)
  MEMBER(double, stepDec, .5)
  MEMBER(double, damping, 1.)
  MEMBER(double, wolfe, .01)
  MEMBER(double, muInit, 1.)
  MEMBER(double, muInc, 5.)
  MEMBER(double, muMax, 1e4)
  MEMBER(double, muLBInit, .1)
  MEMBER(double, muLBDec, .2)
  MEMBER(double, lambdaMax, -1.)
  MEMBER(double, interiorPadding, 1e-2)
  MEMBER(bool,   boundedNewton, true)
  MEMBER(double, finiteDifference, -1.)
#undef MEMBER
    ;
    return self;
  }, "set solver options"
#define MEMBER(type, name, x) , pybind11::arg(#name) = x
  MEMBER(int, verbose, 1)
  MEMBER(double, stopTolerance, 1e-2)
  MEMBER(double, stopFTolerance, -1.)
  MEMBER(double, stopGTolerance, -1.)
  MEMBER(int,    stopEvals, 1000)
  MEMBER(int,    stopInners, 1000)
  MEMBER(int,    stopOuters, 1000)
  MEMBER(int,    stopLineSteps, 10)
  MEMBER(int,    stopTinySteps, 4)
  MEMBER(double, stepInit, 1.)
  MEMBER(double, stepMin, -1.)
  MEMBER(double, stepMax, .2)
  MEMBER(double, stepInc, 1.5)
  MEMBER(double, stepDec, .5)
  MEMBER(double, damping, 1.)
  MEMBER(double, wolfe, .01)
  MEMBER(double, muInit, 1.)
  MEMBER(double, muInc, 5.)
  MEMBER(double, muMax, 1e4)
  MEMBER(double, muLBInit, .1)
  MEMBER(double, muLBDec, .2)
  MEMBER(double, lambdaMax, -1.)
  MEMBER(double, interiorPadding, 1e-2)
  MEMBER(bool,   boundedNewton, true)
  MEMBER(double, finiteDifference, -1.)
#undef MEMBER
      )

  ;

  //===========================================================================

  pybind11::class_<SolverReturn, std::shared_ptr<SolverReturn>>(m, "SolverReturn", "return of nlp solve call")

      .def(pybind11::init<>())
      .def_readwrite("x", &SolverReturn::x)
      .def_readwrite("evals", &SolverReturn::evals)
      .def_readwrite("time", &SolverReturn::time)
      .def_readwrite("feasible", &SolverReturn::feasible)
      .def_readwrite("done", &SolverReturn::done)
      .def_readwrite("f", &SolverReturn::f)
      .def_readwrite("sos", &SolverReturn::sos)
      .def_readwrite("ineq", &SolverReturn::ineq)
      .def_readwrite("eq", &SolverReturn::eq)
  .def("__str__", [](std::shared_ptr<SolverReturn>& self) {  str s;  s <<(*self);  return std::string(s.p); })

  .def("dict", [](std::shared_ptr<SolverReturn>& self) {
    return graph2dict(rai::Graph{
      {"evals", self->evals},
      {"time", self->time},
      {"done", self->done},
      {"feasible", self->feasible},
      {"f", self->f},
      {"sos", self->sos},
      {"ineq", self->ineq},
      {"eq", self->eq},
    });
  })

  ;

  //===========================================================================

#define ENUMVAL(x) .value(#x, rai::M_##x)
  pybind11::enum_<rai::OptMethod>(m, "OptMethod")
    ENUMVAL(none)
    ENUMVAL(GradientDescent) ENUMVAL(Rprop) ENUMVAL(LBFGS) ENUMVAL(Newton)
    ENUMVAL(AugmentedLag) ENUMVAL(LogBarrier) ENUMVAL(slackGN_logBarrier) ENUMVAL(SquaredPenalty) ENUMVAL(singleSquaredPenalty)
    ENUMVAL(slackGN)
    ENUMVAL(NLopt) ENUMVAL(Ipopt) ENUMVAL(slackGN_Ipopt) ENUMVAL(Ceres)
    ENUMVAL(LSZO) ENUMVAL(greedy) ENUMVAL(NelderMead) ENUMVAL(CMA) ENUMVAL(ES)
  ;
#undef ENUMVAL

#define ENUMVAL(x) .value(#x, OT_##x)
  pybind11::enum_<ObjectiveType>(m, "OT")
      ENUMVAL(f) ENUMVAL(sos) ENUMVAL(ineq) ENUMVAL(eq) ENUMVAL(ineqB) ENUMVAL(ineqP) ENUMVAL(none)
  ;
#undef ENUMVAL

}

#endif
