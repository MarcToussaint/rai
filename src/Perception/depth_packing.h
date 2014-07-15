#pragma once

#include <Core/module.h>

BEGIN_MODULE(KinectDepthPacking)
    ACCESS(MT::Array<uint16_t>, kinect_depth);
    ACCESS(byteA, kinect_depthRgb);
END_MODULE()

namespace MLR {
  // pack 16bit depth image into 3 8-bit channels
  void pack_kindepth2rgb(const MT::Array<uint16_t>& depth, byteA& buffer);
}
