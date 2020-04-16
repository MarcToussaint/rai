/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include <pybind11/pybind11.h>

void init_Config(pybind11::module &m);
void init_Feature(pybind11::module &m);
void init_Frame(pybind11::module &m);
void init_KOMO(pybind11::module &m);
void init_LGP_Tree(pybind11::module &m);
void init_Bullet(pybind11::module &m);
void init_PhysX(pybind11::module &m);
void init_Operate(pybind11::module &m);
void init_Camera(pybind11::module &m);
void init_Simulation(pybind11::module &m);

PYBIND11_MODULE(libry, m) {
  m.doc() = "rai bindings";

  init_Config(m);
  init_Feature(m);
  init_Frame(m);
  init_KOMO(m);
  init_LGP_Tree(m);
  init_Bullet(m);
  init_PhysX(m);
  init_Operate(m);
  init_Camera(m);
  init_Simulation(m);

}

#endif
