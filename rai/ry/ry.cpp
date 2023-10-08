/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "ry.h"
#include "types.h"

#include <pybind11/pybind11.h>
#include "../Core/util.h"
#include "../Core/graph.h"


void init_CfgFileParameters(){
  char* argv[2] = {(char*)"rai-pybind", (char*)"-python"};
  int argc = 2;
  rai::initCmdLine(argc, argv);
}


namespace pybind11{
  bool logCallback(const char* str, int log_level){
    std::string _str(str);
    //pybind11::print("[rai]", str, "flush"_a=true);
    //pybind11::print("flush"_a=true);
    return false;
  }
}

void init_LogToPythonConsole(){
  //rai::_log.callback = pybind11::logCallback;
  //LOG(0) <<"initializing ry log callback";
}

void init_enums(pybind11::module& m);
void init_params(pybind11::module& m);

#ifdef RAI_BotOp
void init_BotOp(pybind11::module& m);
#endif

PYBIND11_MODULE(ry, m) {
  m.doc() = "rai bindings";

  init_LogToPythonConsole();
  init_CfgFileParameters();
  init_enums(m);

  m.def("setRaiPath", &rai::setRaiPath, "redefine the rai (or rai-robotModels) path");
  m.def("raiPath", &rai::raiPath, "get a path relative to rai base path");
  m.def("compiled", [](){ std::stringstream msg; msg <<"compile time: "<< __DATE__ <<' ' <<__TIME__; return msg.str(); }, "return a compile date+time version string");

  init_params(m);
  init_Frame(m);
  init_Config(m);
  init_Feature(m);
  init_KOMO(m);
  init_Skeleton(m);
  init_PathAlgos(m);
//  init_LGP_Tree(m);
//  init_Operate(m);
//  init_Camera(m);
  init_Simulation(m);
//  init_CtrlSet(m);
//  init_CtrlSolver(m);

  init_Optim(m);
  init_tests(m);
#ifdef RAI_BotOp
  init_BotOp(m);
#endif
}

void init_params(pybind11::module& m){
  m.def("params_add", [](const pybind11::dict& D){ rai::params()->copy(dict2graph(D), true); }, "add/set parameters");
  m.def("params_file", [](const char* filename){
    ifstream fil(filename);
    if(fil.good()) rai::params()->read(fil);
    else LOG(0) <<"could not add params file '" <<filename <<"'";
  }, "add parameters from a file");
  m.def("params_print", [](){ LOG(0) <<rai::params()(); }, "print the parameters");
  m.def("params_clear", [](){ rai::params()->clear(); }, "clear all parameters");
}

void init_enums(pybind11::module& m){

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::x)

pybind11::enum_<rai::ArgWord>(m, "ArgWord")
  ENUMVAL(_left)
  ENUMVAL(_right)
  ENUMVAL(_sequence)
  ENUMVAL(_path)
    .export_values();

#undef ENUMVAL
#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  pybind11::enum_<rai::JointType>(m, "JT")
  ENUMVAL(rai::JT,hingeX) ENUMVAL(rai::JT,hingeY) ENUMVAL(rai::JT,hingeZ) ENUMVAL(rai::JT,transX) ENUMVAL(rai::JT,transY) ENUMVAL(rai::JT,transZ) ENUMVAL(rai::JT,transXY) ENUMVAL(rai::JT,trans3) ENUMVAL(rai::JT,transXYPhi) ENUMVAL(rai::JT,transYPhi) ENUMVAL(rai::JT,universal) ENUMVAL(rai::JT,rigid) ENUMVAL(rai::JT,quatBall) ENUMVAL(rai::JT,phiTransXY) ENUMVAL(rai::JT,XBall) ENUMVAL(rai::JT,free) ENUMVAL(rai::JT,generic) ENUMVAL(rai::JT,tau)
  .export_values();

  pybind11::enum_<rai::ShapeType>(m, "ST")
  ENUMVAL(rai::ST, none)
  ENUMVAL(rai::ST, box)
  ENUMVAL(rai::ST, sphere)
  ENUMVAL(rai::ST, capsule)
  ENUMVAL(rai::ST, mesh)
  ENUMVAL(rai::ST, cylinder)
  ENUMVAL(rai::ST, marker)
  ENUMVAL(rai::ST, pointCloud)
  ENUMVAL(rai::ST, ssCvx)
  ENUMVAL(rai::ST, ssBox)
  ENUMVAL(rai::ST, ssCylinder)
  ENUMVAL(rai::ST, ssBoxElip)
  ENUMVAL(rai::ST, quad)
  ENUMVAL(rai::ST, camera)
  ENUMVAL(rai::ST, sdf)
    .export_values();


  pybind11::enum_<FeatureSymbol>(m, "FS")
  ENUMVAL(FS, position)
  ENUMVAL(FS, positionDiff)
  ENUMVAL(FS, positionRel)
  ENUMVAL(FS, quaternion)
  ENUMVAL(FS, quaternionDiff)
  ENUMVAL(FS, quaternionRel)
  ENUMVAL(FS, pose)
  ENUMVAL(FS, poseDiff)
  ENUMVAL(FS, poseRel)
  ENUMVAL(FS, vectorX)
  ENUMVAL(FS, vectorXDiff)
  ENUMVAL(FS, vectorXRel)
  ENUMVAL(FS, vectorY)
  ENUMVAL(FS, vectorYDiff)
  ENUMVAL(FS, vectorYRel)
  ENUMVAL(FS, vectorZ)
  ENUMVAL(FS, vectorZDiff)
  ENUMVAL(FS, vectorZRel)
  ENUMVAL(FS, scalarProductXX)
  ENUMVAL(FS, scalarProductXY)
  ENUMVAL(FS, scalarProductXZ)
  ENUMVAL(FS, scalarProductYX)
  ENUMVAL(FS, scalarProductYY)
  ENUMVAL(FS, scalarProductYZ)
  ENUMVAL(FS, scalarProductZZ)
  ENUMVAL(FS, gazeAt)

  ENUMVAL(FS, angularVel)

  ENUMVAL(FS, accumulatedCollisions)
  ENUMVAL(FS, jointLimits)
  ENUMVAL(FS, distance)
  ENUMVAL(FS, negDistance)
  ENUMVAL(FS, oppose)

  ENUMVAL(FS, qItself)
  ENUMVAL(FS, jointState)

  ENUMVAL(FS, aboveBox)
  ENUMVAL(FS, insideBox)

  ENUMVAL(FS, pairCollision_negScalar)
  ENUMVAL(FS, pairCollision_vector)
  ENUMVAL(FS, pairCollision_normal)
  ENUMVAL(FS, pairCollision_p1)
  ENUMVAL(FS, pairCollision_p2)

  ENUMVAL(FS, standingAbove)

  ENUMVAL(FS, physics)
  ENUMVAL(FS, contactConstraints)
  ENUMVAL(FS, energy)

  ENUMVAL(FS, transAccelerations)
  ENUMVAL(FS, transVelocities)
      .export_values();

  pybind11::enum_<rai::SkeletonSymbol>(m, "SY")
  ENUMVAL(rai::SY,touch) ENUMVAL(rai::SY,above) ENUMVAL(rai::SY,inside) ENUMVAL(rai::SY,oppose) ENUMVAL(rai::SY,restingOn)
  ENUMVAL(rai::SY,poseEq) ENUMVAL(rai::SY,positionEq) ENUMVAL(rai::SY,stableRelPose) ENUMVAL(rai::SY,stablePose)
  ENUMVAL(rai::SY,stable) ENUMVAL(rai::SY,stableOn) ENUMVAL(rai::SY,dynamic) ENUMVAL(rai::SY,dynamicOn) ENUMVAL(rai::SY,dynamicTrans) ENUMVAL(rai::SY,quasiStatic) ENUMVAL(rai::SY,quasiStaticOn) ENUMVAL(rai::SY,downUp) ENUMVAL(rai::SY,break) ENUMVAL(rai::SY,stableZero)
  ENUMVAL(rai::SY,contact) ENUMVAL(rai::SY,contactStick) ENUMVAL(rai::SY,contactComplementary) ENUMVAL(rai::SY,bounce) ENUMVAL(rai::SY,push)
  ENUMVAL(rai::SY,magic) ENUMVAL(rai::SY,magicTrans)
  ENUMVAL(rai::SY,pushAndPlace)
  ENUMVAL(rai::SY,topBoxGrasp) ENUMVAL(rai::SY,topBoxPlace)
  ENUMVAL(rai::SY,dampMotion)
  ENUMVAL(rai::SY,identical)
  ENUMVAL(rai::SY,alignByInt)
  ENUMVAL(rai::SY,makeFree) ENUMVAL(rai::SY,forceBalance)
  ENUMVAL(rai::SY,relPosY)
  ENUMVAL(rai::SY,touchBoxNormalX) ENUMVAL(rai::SY,touchBoxNormalY) ENUMVAL(rai::SY,touchBoxNormalZ)
  ENUMVAL(rai::SY,boxGraspX) ENUMVAL(rai::SY,boxGraspY) ENUMVAL(rai::SY,boxGraspZ)
  ENUMVAL(rai::SY,lift)
  ENUMVAL(rai::SY,stableYPhi)
  ENUMVAL(rai::SY,stableOnX)
  ENUMVAL(rai::SY,stableOnY)
  ENUMVAL(rai::SY,end)
  .export_values();

#undef ENUMVAL
#define ENUMVAL(x) .value(#x, rai::Simulation::_##x)

  pybind11::enum_<rai::Simulation::Engine>(m, "SimulationEngine")
  ENUMVAL(physx)
  ENUMVAL(bullet)
  ENUMVAL(kinematic)
  .export_values();

  pybind11::enum_<rai::Simulation::ControlMode>(m, "ControlMode")
  ENUMVAL(none)
  ENUMVAL(position)
  ENUMVAL(velocity)
  ENUMVAL(acceleration)
  ENUMVAL(spline)
  .export_values();

  pybind11::enum_<rai::Simulation::ImpType>(m, "ImpType")
  ENUMVAL(closeGripper)
  ENUMVAL(moveGripper)
  ENUMVAL(depthNoise)
  ENUMVAL(rgbNoise)
  ENUMVAL(adversarialDropper)
  ENUMVAL(objectImpulses)
  ENUMVAL(noPenetrations)
  .export_values();

}

//void init_BotOp(pybind11::module& m){}

#endif
