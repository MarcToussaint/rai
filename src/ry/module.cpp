/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-algo.h"
#include "py-Config.h"
#include "py-Feature.h"
#include "py-Frame.h"
#include "py-KOMO.h"
#include "py-LGP.h"
#include "py-Simulation.h"
#include "py-Spline.h"
#include "py-Quaternion.h"
//#include "py-Camera.h"
//#include "py-LGP_Tree.h"
//#include "py-Operate.h"
//#include "py-Control.h"
#include "py-Optim.h"
#include "py-tests.h"
#include "py-DataGen.h"
#include "types.h"

#include <pybind11/pybind11.h>
#include "../Core/util.h"
#include "../Core/graph.h"

void init_CfgFileParameters() {
  char* argv[1] = {(char*)"rai-pybind"};
  int argc = 1;
  rai::initCmdLine(argc, argv);
}

namespace pybind11 {
bool logCallback(const char* str, int log_level) {
  std::string _str(str);
  //pybind11::print("[rai]", str, "flush"_a=true);
  //pybind11::print("flush"_a=true);
  return false;
}
}

void init_LogToPythonConsole() {
  //rai::_log.callback = pybind11::logCallback;
  //LOG(0) <<"initializing ry log callback";
}

void init_enums(pybind11::module& m);
void init_params(pybind11::module& m);

#ifdef RAI_BotOp
void init_BotOp(pybind11::module& m);
#endif

PYBIND11_MODULE(_robotic, m) {
  m.doc() = "rai bindings";

  init_LogToPythonConsole();
  init_CfgFileParameters();
  init_enums(m);

  m.def("setRaiPath", &rai::setRaiPath, "redefine the rai (or rai-robotModels) path");
  m.def("raiPath", &rai::raiPath, "get a path relative to rai base path");
  m.def("compiled", []() { std::stringstream msg; msg <<"compile time: "<< __DATE__ <<' ' <<__TIME__; return msg.str(); }, "return a compile date+time version string");

  m.def("rnd_seed_random", []() { rnd.seed_random(); }, "seed rnd randomly");
  m.def("rnd_seed", [](int s) { rnd.seed(s); }, "seed rnd with s", pybind11::arg("s"));

  init_params(m);
  init_algo(m);
  init_Frame(m);
  init_Config(m);
//  init_Feature(m);
  init_KOMO(m);
  init_LGP(m);
  init_Skeleton(m);
  init_PathAlgos(m);
  init_Spline(m);
  init_Quaternion(m);
//  init_LGP_Tree(m);
//  init_Operate(m);
//  init_Camera(m);
  init_Simulation(m);
//  init_CtrlSet(m);
//  init_CtrlSolver(m);

  init_Optim(m);
  init_tests(m);
  init_DataGen(m);
#ifdef RAI_BotOp
  init_BotOp(m);
#endif
}

void init_params(pybind11::module& m) {
  m.def("params_add", [](const pybind11::dict& D) { rai::Graph G = dict2graph(D); for(rai::Node *n:G) rai::params()->set(n); }, "add/set parameters", pybind11::arg("python dictionary or params to add"));
  m.def("params_file", [](const char* filename) {
    ifstream fil(filename);
    if(fil.good()) rai::params()->read(fil);
    else LOG(0) <<"could not add params file '" <<filename <<"'";
  }, "add parameters from a file", pybind11::arg("filename"));
  m.def("params_print", []() { cout <<rai::params()(); }, "print the parameters");
  m.def("params_get", []() { return graph2dict(rai::params()()); }, "return parameters as dict");
  m.def("params_clear", []() { rai::params()->clear(); }, "clear all parameters");
}

//void init_BotOp(pybind11::module& m){}

#endif
