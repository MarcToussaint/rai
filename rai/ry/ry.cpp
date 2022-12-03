/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry.h"

#include <pybind11/pybind11.h>
#include "../Core/util.h"


void init_CfgFileParameters(){
  char* argv[2] = {(char*)"rai-pybind", (char*)"-python"};
  int argc = 2;
  rai::initCmdLine(argc, argv);
}


namespace pybind11{
  bool logCallback(const char* str, int log_level){
    std::string _str(str);
    pybind11::print("[rai]", str, "flush"_a=true);
    //pybind11::print("flush"_a=true);
    return false;
  }
}

void init_LogToPythonConsole(){
  rai::_log.callback = pybind11::logCallback;
  //LOG(0) <<"initializing ry log callback";
}

void init_enums(pybind11::module& m);


PYBIND11_MODULE(libry, m) {
  m.doc() = "rai bindings";

  init_LogToPythonConsole();
  init_CfgFileParameters();
  init_enums(m);

  m.def("setRaiPath", &rai::setRaiPath, "redefine the rai (or rai-robotModels) path");

  init_Config(m);
  init_Feature(m);
  init_Frame(m);
  init_KOMO(m);
  init_Skeleton(m);
  init_PathAlgos(m);
//  init_LGP_Tree(m);
  init_Bullet(m);
//  init_PhysX(m);
//  init_Operate(m);
//  init_Camera(m);
  init_Simulation(m);
//  init_CtrlSet(m);
//  init_CtrlSolver(m);

  init_Optim(m);
  init_tests(m);
}

void init_enums(pybind11::module& m){

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::_##x)

 pybind11::enum_<rai::ArgWord>(m, "ArgWord")
    ENUMVAL(left)
    ENUMVAL(right)
    ENUMVAL(sequence)
    ENUMVAL(path)
     .export_values();
}

#endif
