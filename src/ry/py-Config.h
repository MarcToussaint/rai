/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"

#ifdef RAI_PYBIND

#include <pybind11/pybind11.h>

void init_Config(pybind11::module& m);

#endif

namespace rai {
struct Frame;
struct Configuration;
struct ConfigurationViewer;
struct CameraView;
}

void null_deleter(rai::Frame*);
void Config_null_deleter(rai::Configuration*);
