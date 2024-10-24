/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-Config.h"
#include "types.h"

#include "../Kin/kin.h"
#include "../Kin/forceExchange.h"
#include "../Kin/kin_bullet.h"
#include "../Kin/kin_physx.h"
#include "../Kin/proxy.h"
#include "../Kin/viewer.h"
#include "../Kin/cameraview.h"
#include "../Kin/simulation.h"
#include "../Geo/fclInterface.h"
#include "../LGP/LGP_tree.h"
#include "../Geo/depth2PointCloud.h"

//void checkView(shared_ptr<rai::Configuration>& self){ if(self->hasView()) self->view(); }
void null_deleter(rai::Frame*) {}
void Config_null_deleter(rai::Configuration*) {}

void init_Config(pybind11::module& m) {

  //===========================================================================

  m.def("depthImage2PointCloud", [](const pybind11::array_t<float>& depth, const arr& fxycxy) {
    arr pts;
    depthData2pointCloud(pts, numpy2arr<float>(depth), fxycxy);
    return pts;
  }, "return the point cloud from the depth image",
  pybind11::arg("depth"),
  pybind11::arg("fxycxy")
       );

  //===========================================================================

  pybind11::class_<rai::Configuration, shared_ptr<rai::Configuration>>(m, "Config", "Core data structure to represent a kinematic configuration (essentially a tree of frames). See https://marctoussaint.github.io/robotics-course/tutorials/1a-configurations.html")

  .def(pybind11::init<>(), "initializes to an empty configuration, with no frames")

  .def("clear", [](shared_ptr<rai::Configuration>& self) {
    self->clear();
  }, "clear all frames and additional data; becomes the empty configuration, with no frames")

  .def("addFile", [](shared_ptr<rai::Configuration>& self, const char* filename, const char* namePrefix) {
    rai::Frame* f = self->addFile(filename, namePrefix);
    return shared_ptr<rai::Frame>(f, &null_deleter);  //giving it a non-sense deleter!
  },
  "add the contents of the file to C",
  pybind11::arg("filename"),
  pybind11::arg("namePrefix") = nullptr
      )

  .def("addFrame", [](shared_ptr<rai::Configuration>& self, const std::string& name, const std::string& parent, const std::string& args) {
    rai::Frame* f = self->addFrame(name.c_str(), parent.c_str(), args.c_str());
    return shared_ptr<rai::Frame>(f, &null_deleter);  //giving it a non-sense deleter!
  },
  "add a new frame to C; optionally make this a child to the given parent; use the Frame methods to set properties of the new frame",
  pybind11::arg("name"),
  pybind11::arg("parent") = std::string(),
  pybind11::arg("args") = std::string()
      )

  .def("addConfigurationCopy", [](shared_ptr<rai::Configuration>& self, shared_ptr<rai::Configuration>& other, const str& prefix, double tau) {
    rai::Frame* f = self->addConfigurationCopy(*other, prefix, tau);
    return shared_ptr<rai::Frame>(f, &null_deleter);  //giving it a non-sense deleter!
  }, "",
  pybind11::arg("config"),
  pybind11::arg("prefix")=str{},
  pybind11::arg("tau")=1.
      )

  .def("getFrame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName, bool warnIfNotExist) {
    rai::Frame* f = self->getFrame(frameName.c_str(), warnIfNotExist);
    return shared_ptr<rai::Frame>(f, &null_deleter);  //giving it a non-sense deleter!
  },
  "get access to a frame by name; use the Frame methods to set/get frame properties",
  pybind11::arg("frameName"),
  pybind11::arg("warnIfNotExist")=true
      )

  .def("frame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName, bool warnIfNotExist) {
    rai::Frame* f = self->getFrame(frameName.c_str(), warnIfNotExist);
    return shared_ptr<rai::Frame>(f, &null_deleter);  //giving it a non-sense deleter!
  },
  "get access to a frame by name; use the Frame methods to set/get frame properties",
  pybind11::arg("frameName"),
  pybind11::arg("warnIfNotExist")=true
      )

  .def("frames", [](shared_ptr<rai::Configuration>& self) {
    std::vector<shared_ptr<rai::Frame>> F;
    for(rai::Frame* f:self->frames) F.push_back(shared_ptr<rai::Frame>(f, &null_deleter)); //giving it a non-sense deleter!
    return F;
  })

  .def("delFrame", [](shared_ptr<rai::Configuration>& self, const std::string& frameName) {
    rai::Frame* p = self->getFrame(frameName.c_str(), true);
    if(p) delete p;
  },
  "destroy and remove a frame from C",
  pybind11::arg("frameName")
      )

  .def("getJointNames", &rai::Configuration::getJointNames, "get the list of joint names")
  .def("getJointDimension", &rai::Configuration::getJointStateDimension, "get the total number of degrees of freedom")
  .def("getJointState", [](shared_ptr<rai::Configuration>& self) {
    return self->getJointState();
  },
  "get the joint state as a numpy vector, optionally only for a subset of joints specified as list of joint names"
      )

  .def("getJointLimits", [](shared_ptr<rai::Configuration>& self) {
    return self->getJointLimits();
  },
  "get the joint limits as a n-by-2 matrix; for dofs that do not have limits defined, the entries are [0,-1] (i.e. upper limit < lower limit)"
      )

  .def("setJointState", [](std::shared_ptr<rai::Configuration>& self, const arr& q, const pybind11::list& joints) {
    if(!joints.size()) {
      self->setJointState(q);
    } else {
      self->setJointState(q, self->getFrames(list2arr<str>(joints)));
    }
  },
  "set the joint state, optionally only for a subset of joints specified as list of joint names",
  pybind11::arg("q"),
  pybind11::arg("joints") = pybind11::list()
      )

  .def("setJointStateSlice", [](shared_ptr<rai::Configuration>& self, const std::vector<double>& q, uint t) {
    self->setJointStateSlice(arr(q, true), t);
  }, "")

  .def("getFrameNames", [](shared_ptr<rai::Configuration>& self) {
    return StringA2strvec(self->getFrameNames());
  },
  "get the list of frame names"
      )

  .def("getFrameDimension", [](shared_ptr<rai::Configuration>& self) {
    return self->frames.N;
  },
  "get the total number of frames"
      )

  .def("getFrameState", [](shared_ptr<rai::Configuration>& self) {
    arr X = self->getFrameState();
    return arr2numpy(X);
  },
  "get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame"
      )

  .def("setFrameState", [](shared_ptr<rai::Configuration>& self, const std::vector<double>& X, const std::vector<std::string>& frames) {
    arr _X(X, true);
    _X.reshape(_X.N/7, 7);
    if(frames.size()) {
      self->setFrameState(_X, self->getFrames(strvec2StringA(frames)));
    } else {
      self->setFrameState(_X);
    }
  },
  "set the frame state, optionally only for a subset of frames specified as list of frame names",
  pybind11::arg("X"),
  pybind11::arg("frames") = std::vector<std::string>()
      )

  .def("setFrameState", [](shared_ptr<rai::Configuration>& self, const pybind11::array& X, const std::vector<std::string>& frames) {
    arr _X = numpy2arr<double>(X);
    _X.reshape(_X.N/7, 7);
    if(frames.size()) {
      self->setFrameState(_X, self->getFrames(strvec2StringA(frames)));
    } else {
      self->setFrameState(_X);
    }
  },
  "set the frame state, optionally only for a subset of frames specified as list of frame names",
  pybind11::arg("X"),
  pybind11::arg("frames") = std::vector<std::string>()
      )

//  .def("feature", [](shared_ptr<rai::Configuration>& self, FeatureSymbol featureSymbol, const std::vector<std::string>& frameNames, const std::vector<double>& scale, const std::vector<double>& target, int order) {
//    return symbols2feature(featureSymbol, strvec2StringA(frameNames), *self, arr(scale, true), arr(target, true), order);
//  },
//  "create a feature (a differentiable map from joint state to a vector space), as they're typically used for IK or optimization. See the dedicated tutorial for details. "
//  "featureSymbol defines which mapping this is (position, vectors, collision distance, etc). "
//  "many mapping refer to one or several frames, which need to be specified using frameNames",
//  pybind11::arg("featureSymbol"),
//  pybind11::arg("frameNames")=std::vector<std::string>(),
//    pybind11::arg("scale")=std::vector<double>(),
//    pybind11::arg("target")=std::vector<double>(),
//    pybind11::arg("order")=-1
//    )

  .def("eval", [](shared_ptr<rai::Configuration>& self, FeatureSymbol fs, const StringA& frames, const arr& scale, const arr& target, int order) {
    arr y = self->eval(fs, frames, scale, target, order);
    return pybind11::make_tuple(arr2numpy(y), arr2numpy(y.J()));
  }, "evaluate a feature -- see https://marctoussaint.github.io/robotics-course/tutorials/features.html",
  pybind11::arg("featureSymbol"),
  pybind11::arg("frames")=StringA{},
  pybind11::arg("scale")=NoArr,
  pybind11::arg("target")=NoArr,
  pybind11::arg("order")=-1
      )

  .def("selectJointsBySubtree", [](shared_ptr<rai::Configuration>& self, const std::shared_ptr<rai::Frame>& root){ self->selectJointsBySubtrees(FrameL{root.get()}); }, "",
       pybind11::arg("root")
       )

  .def("selectJoints", [](shared_ptr<rai::Configuration>& self, const std::vector<std::string>& jointNames, bool notThose) {
    self->selectJointsByName(strvec2StringA(jointNames), notThose);
  },
  "redefine what are considered the DOFs of this configuration: only joints listed in jointNames are considered \
part of the joint state and define the number of DOFs",
  pybind11::arg("jointNames"),
  pybind11::arg("notThose") = false
      )

  .def("attach", [](shared_ptr<rai::Configuration>& self, const std::string& frame1, const std::string& frame2) {
    self->attach(frame1.c_str(), frame2.c_str());
  },
  "change the configuration by creating a rigid joint from frame1 to frame2, adopting their current \
relative pose. This also breaks the first joint that is parental to frame2 and reverses the \
topological order from frame2 to the broken joint"
      )

  .def("computeCollisions", [](shared_ptr<rai::Configuration>& self) {
    self->ensure_proxies();
  },
  "[should be obsolete; getCollision* methods auto ensure proxies] call the broadphase collision engine (SWIFT++ or FCL) to generate the list of collisions (or near proximities) \
between all frame shapes that have the collision tag set non-zero"
      )

  .def("getCollisions", [](shared_ptr<rai::Configuration>& self, double belowMargin) {
    self->fcl()->mode = rai::FclInterface::_broadPhaseOnly;
    self->ensure_proxies(true);

    pybind11::list ret;
    for(const rai::Proxy& p: self->proxies) {
      if(!p.collision)((rai::Proxy*)&p)->calc_coll();
      if(p.d>belowMargin) continue;
      pybind11::tuple tuple(3);
      tuple[0] = p.a->name.p;
      tuple[1] = p.b->name.p;
      tuple[2] = p.d;
//      tuple[3] = p.posA;
//      tuple[4] = p.posB;
      ret.append(tuple) ;
    }
    return ret;
  },
  "return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance). \
Optionally report only on distances below a margin \
To get really precise distances and penetrations use the FS.distance feature with the two frame names",
  pybind11::arg("belowMargin") = 1.
      )

  .def("getCollisionsTotalPenetration", &rai::Configuration::getTotalPenetration,
       "returns the sum of all penetrations (using FCL for broadphase; and low-level GJK/MRP for fine pair-wise distance/penetration computation)")

  .def("getCollisionFree", &rai::Configuration::getCollisionFree,
       "returns if the configuration is collision free (binary collision check, using FCL only; collidable objects need to have contact flag)")

  .def("getCollidablePairs", [](shared_ptr<rai::Configuration>& self) {
    return rai::framesToNames(self->getCollidablePairs());
  }, "returns the list of collisable pairs -- this should help debugging the 'contact' flag settings in a configuration")

  .def("view",  &rai::Configuration::view,
       "open a view window for the configuration",
       pybind11::arg("pause")=false,
       pybind11::arg("message")=nullptr)

  .def("view_recopyMeshes", [](shared_ptr<rai::Configuration>& self) {
    self->get_viewer()->recopyMeshes(self->frames);
  }, "")

  .def("view_getRgb", [](std::shared_ptr<rai::Configuration>& self) {
    return Array2numpy<byte>(self->get_viewer()->getRgb());
  }, "")

  .def("view_getDepth", [](std::shared_ptr<rai::Configuration>& self) {
    return Array2numpy<float>(self->get_viewer()->getDepth());
  }, "")

  .def("view_savePng", [](std::shared_ptr<rai::Configuration>& self, const char* pathPrefix) {
    self->get_viewer()->savePng(pathPrefix);
  }, "saves a png image of the current view, numbered with a global counter, with the intention to make a video",
  pybind11::arg("pathPrefix") = "z.vid/"
      )

  .def("view_close", &rai::Configuration::view_close, "close the view")
  .def("set_viewer",  &rai::Configuration::set_viewer, "")
  .def("get_viewer",  &rai::Configuration::get_viewer, "", pybind11::arg("window_title")=nullptr, pybind11::arg("offscreen")=false)

  .def("view_raise", [](shared_ptr<rai::Configuration>& self) {
    self->get_viewer()->raiseWindow();
  }, "raise the view")

  .def("view_pose", [](shared_ptr<rai::Configuration>& self) {
    rai::Camera& cam = self->get_viewer()->displayCamera();
    return cam.X.getArr7d();
  }, "return the 7D pose of the view camera")

  .def("view_focalLength", [](shared_ptr<rai::Configuration>& self) {
    rai::Camera& cam = self->get_viewer()->displayCamera();
    return cam.focalLength;
  }, "return the focal length of the view camera (only intrinsic parameter)")

  .def("view_fxycxy", [](shared_ptr<rai::Configuration>& self) {
    OpenGL& gl = self->get_viewer()->ensure_gl();
    rai::Camera& cam = self->get_viewer()->displayCamera();
    return cam.getFxycxy(gl.width, gl.height);
  }, "return (fx, fy, cx, cy): the focal length and image center in PIXEL UNITS")

  .def("view_setCamera", [](shared_ptr<rai::Configuration>& self, rai::Frame* frame) {
    self->get_viewer()->setCamera(frame);
  }, "set the camera pose to a frame, and check frame attributes for intrinsic parameters (focalLength, width height)")

  .def("watchFile", &rai::Configuration::watchFile,
       "launch a viewer that listents (inode) to changes of a file (made by you in an editor), and \
reloads, displays and animates the configuration whenever the file is changed"
      )

  .def("animate", [](shared_ptr<rai::Configuration>& self) { self->animate(); },
  "displays while articulating all dofs in a row")

  .def("animateSpline", &rai::Configuration::animateSpline,
       "animate with random spline in limits bounding box [T=#spline points]",
       pybind11::arg("T")=3)

  .def("report", [](shared_ptr<rai::Configuration>& self) {
    str s;
    self->report(s);
    return pybind11::str(s.p, s.N);
  }, "return a string with basic info (#frames, etc)")

  .def("write", [](shared_ptr<rai::Configuration>& self) { str s; self->write(s);  return pybind11::str(s.p, s.N); },
  "write the full configuration in a string (roughly yaml), e.g. for file export")

  .def("writeMesh", &rai::Configuration::writeMesh,
       "write the full configuration in a ply mesh file",
       pybind11::arg("filename"))

  .def("writeMeshes", &rai::Configuration::writeMeshes,
       "write all object meshes in a directory",
       pybind11::arg("pathPrefix"))

  .def("writeURDF", [](shared_ptr<rai::Configuration>& self) { str s; self->writeURDF(s);  return pybind11::str(s.p, s.N); },
  "write the full configuration as URDF in a string, e.g. for file export")

  .def("writeCollada", &rai::Configuration::writeCollada,
       "write the full configuration in a collada file for export",
       pybind11::arg("filename"), pybind11::arg("format")="collada")


  .def("checkConsistency", &rai::Configuration::checkConsistency, "internal use")

  ;

//===========================================================================

  pybind11::class_<rai::CameraView, shared_ptr<rai::CameraView>>(m, "CameraView", "Offscreen rendering")

      .def(pybind11::init<const rai::Configuration&, bool>(), "constructor",
           pybind11::arg("config"),
           pybind11::arg("offscreen") = true)

      .def("setCamera", &rai::CameraView::selectSensor, "select a camera, typically a frame that has camera info attributes",
           pybind11::arg("cameraFrameName"))

  .def("computeImageAndDepth", [](rai::CameraView& self, const rai::Configuration& C, bool visualsOnly) {
    byteA img;
    floatA depth;
    self.updateConfiguration(C);
    if(visualsOnly) self.renderMode = rai::CameraView::visuals;
    else self.renderMode = rai::CameraView::all;
    self.computeImageAndDepth(img, depth);
    return pybind11::make_tuple(Array2numpy<byte>(img),
                                Array2numpy<float>(depth));
  },
  "returns image and depth from a camera sensor; the 'config' argument needs to be the same configuration as in the constructor, but in new state",
  pybind11::arg("config"),
  pybind11::arg("visualsOnly") = true)

  .def("getFxycxy", &rai::CameraView::getFxycxy, "return the camera intrinsics f_x, f_y, c_x, c_y")
  .def("computeSegmentationImage", &rai::CameraView::computeSegmentationImage, "return an rgb image encoding the object ID segmentation")
  .def("computeSegmentationID", &rai::CameraView::computeSegmentationImage, "return a uint16 array with object ID segmentation")
  ;

//===========================================================================

  pybind11::class_<rai::ConfigurationViewer, shared_ptr<rai::ConfigurationViewer>>(m, "ConfigurationViewer", "internal viewer handle (gl window)");

}

#endif
