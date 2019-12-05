#pragma once

#include <Core/thread.h>

namespace rai{
    struct Configuration;
    struct CameraView;
}
struct Feature;
struct KinViewer;
struct KinPoseViewer;
struct ImageViewer;
struct PointCloudViewer;

namespace ry{

typedef Var<rai::Configuration> Config;

struct RyFeature { ptr<Feature> feature; };

struct ConfigViewer { ptr<KinViewer> view; };
struct PathViewer { ptr<KinPoseViewer> view; };
struct ImageViewer { ptr<::ImageViewer> view; };
struct PointCloudViewer { ptr<::PointCloudViewer> view; };

struct RyCameraView {
  ptr<rai::CameraView> cam;
  Var<byteA> image;
  Var<floatA> depth;
  Var<byteA> segmentation;
  Var<arr> pts;
};

}
