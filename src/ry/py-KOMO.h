/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#ifdef RAI_PYBIND

#include <pybind11/pybind11.h>

void init_KOMO(pybind11::module& m);
void init_Skeleton(pybind11::module& m);
void init_PathAlgos(pybind11::module& m);

#endif
