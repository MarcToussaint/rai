/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"

namespace rai {
struct Configuration;
struct ConfigurationViewer;
struct CameraView;
}

namespace ry {

struct RyCameraView {
  ptr<rai::CameraView> cam;
  Var<byteA> image;
  Var<floatA> depth;
  Var<byteA> segmentation;
  Var<arr> pts;
};

}
