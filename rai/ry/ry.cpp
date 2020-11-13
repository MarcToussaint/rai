/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry.h"

#include <pybind11/pybind11.h>

PYBIND11_MODULE(libry, m) {
  m.doc() = "rai bindings";

#ifdef RAI_BIND_KOMO
  init_Config(m);
  init_Feature(m);
  init_Frame(m);
  init_KOMO(m);
//  init_LGP_Tree(m);
//  init_Bullet(m);
//  init_PhysX(m);
//  init_Operate(m);
//  init_Camera(m);
  init_Simulation(m);
  init_CtrlSet(m);
  init_CtrlSolver(m);
#endif

  init_Optim(m);
}

#endif
