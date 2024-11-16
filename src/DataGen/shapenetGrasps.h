#pragma once

#include <Core/util.h>
#include <Kin/kin.h>
#include <Kin/kin_physx.h>

struct ShapenetGrasps_Options {
  RAI_PARAM("ShapenetGrasps/", int, verbose, 1)
  RAI_PARAM("ShapenetGrasps/", rai::String, filesPrefix, "shapenet/models/")
  RAI_PARAM("ShapenetGrasps/", int, numShapes, -1)
  RAI_PARAM("ShapenetGrasps/", int, startShape, 0)
  RAI_PARAM("ShapenetGrasps/", int, simVerbose, 0)
  RAI_PARAM("ShapenetGrasps/", int, optVerbose, 0)
  RAI_PARAM("ShapenetGrasps/", double, simTau, .01)
  RAI_PARAM("ShapenetGrasps/", double, gripperCloseSpeed, .001)
  RAI_PARAM("ShapenetGrasps/", double, moveSpeed, .005)
  RAI_PARAM("ShapenetGrasps/", double, pregraspNormalSdv, .2)
};

struct ShapenetGrasps{
  ShapenetGrasps_Options opt;
  rai::PhysX_Options physxOpt;

  ShapenetGrasps();

  void getSamples(arr& X, arr& Contexts, arr& Scores, uint N);
  arr evaluateSample(const arr& x, const arr& context);
  void displaySamples(const arr& X, const arr& Contexts, const arr& Scores={});

private:
  bool generateRndCandidate();
  arr evaluateCandidate();
public:
  StringA files;
  rai::Configuration C;
  void clearScene();
  bool createScene(const char* file, int idx, bool rndPose=true, bool visual=false);
  void setRelGripperPose(const arr& pose, const char* objPts="objPts0");
};
