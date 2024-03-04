/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

#undef MAIN
#define MAIN test_komo_main
#include "../../test/KOMO/komo/main.cpp"

#undef MAIN
#define MAIN test_simulation_main
#include "../../test/Kin/simulation/main.cpp"

void init_tests(pybind11::module& m) {

  pybind11::module_ mt = m.def_submodule("test", "rai test methods");

#define addTest(name) mt.def(#name, & test##name, "rai test " #name);
  addTest(Easy)
  addTest(Align)
  addTest(Thin)
  addTest(PR2)
  addTest(Threading)

  addTest(RndScene)
  addTest(Friction)
  addTest(StackOfBlocks)
  addTest(Compound)
  addTest(Pushes)
  addTest(OpenClose)
  addTest(Grasp)
#undef addTest

}

#endif
