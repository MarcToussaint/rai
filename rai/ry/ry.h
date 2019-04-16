#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <Core/array.h>
#include <Core/graph.h>
#include <Kin/kin.h>
#include <Core/thread.h>
#include <Kin/kinViewer.h>
#include <KOMO/komo.h>
#include <Kin/cameraview.h>
#include <Gui/viewer.h>
#include <LGP/LGP_tree.h>
#include <Operate/robotOperation.h>

struct BulletInterface;

namespace ry{

  typedef Var<rai::KinematicWorld> Config;

  struct ConfigViewer { ptr<KinViewer> view; };
  struct PathViewer { ptr<KinPoseViewer> view; };
  struct ImageViewer { ptr<::ImageViewer> view; };
  struct PointCloudViewer { ptr<::PointCloudViewer> view; };

  struct RyKOMO{
    RyKOMO(){}
    RyKOMO(ry::Config& self){
      komo = make_shared<KOMO>(self.get());
      config.set() = komo->world;
      komo->setIKOpt();
    }
    RyKOMO(ry::Config& self, uint numConfigs){
      CHECK_GE(numConfigs, 1, "");
      komo = make_shared<KOMO>(self.get());
      config.set() = komo->world;
      komo->setDiscreteOpt(numConfigs);
    }
    RyKOMO(ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase){
      komo = make_shared<KOMO>(self.get());
      config.set() = komo->world;
      komo->setPathOpt(phases, stepsPerPhase, timePerPhase);
    }
    RyKOMO(const ptr<KOMO>& _komo){
      komo = _komo;
    }

    ptr<KOMO> komo;
    Var<rai::KinematicWorld> config;
    Var<arr> path;
  };

  struct RyLGP_Tree { ptr<LGP_Tree_Thread> lgp; };

  struct RyFeature { ptr<Feature> feature; };
  struct RyFrame { rai::Frame *frame=0; };

  struct RyCameraView {
    ptr<rai::CameraView> cam;
    Var<byteA> image;
    Var<floatA> depth;
    Var<byteA> segmentation;
    Var<arr> pts;
  };

  struct RyBullet { std::shared_ptr<BulletInterface> bullet; };

  struct RyOperate { std::shared_ptr<RobotOperation> R; };
}

namespace ry{

typedef std::pair<std::vector<unsigned int>, std::vector<double> > I_arr;
typedef std::vector<std::string> I_StringA;
typedef std::map<std::string, std::string> I_dict;
typedef std::map<std::string, std::vector<double> > I_args;

typedef std::tuple<std::vector<double>, int, int, I_StringA, I_args> I_feature;
typedef std::vector<I_feature> I_features;

typedef std::tuple<std::vector<double>, std::string, I_StringA, I_args> I_objective;
typedef std::vector<I_objective> I_objectives;

struct FrameInfo{
  int ID;
  std::string name;
  std::string parent;
  I_StringA children;
  std::vector<double> X;
  std::vector<double> Q;
};

}

inline StringA I_conv(const ry::I_StringA& x){
  StringA y(x.size());
  for(uint i=0;i<y.N;i++) y(i) = x[i];
  return y;
}

inline ry::I_StringA I_conv(const StringA& x){
  ry::I_StringA y;
  for(const rai::String& s:x) y.push_back(s.p);
  return y;
}

inline Graph I_conv(const ry::I_dict& x){
  return Graph(x);
}

inline ry::I_arr I_conv(const arr& x){
  ry::I_arr y;
  y.first = x.dim();
  y.second = x;
  return y;
}

inline arr I_conv(const ry::I_arr& x){
  arr y;
  y = conv_stdvec2arr(x.second);
  y.reshape(conv_stdvec2arr(x.first));
  return y;
}
