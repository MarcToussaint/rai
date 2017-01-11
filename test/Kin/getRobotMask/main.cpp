#include <Kin/kin.h>
#include <Gui/opengl.h>


void TEST(GetRobotMask){
  mlr::KinematicWorld robot("pr2_model/pr2_model.ors");
  robot.gl().camera.setKinect();
  robot.gl().camera.X = robot.getShapeByName("endeffKinect")->X * robot.gl().camera.X;
//  robot.gl().watch(); //if commented, glut/gtk is never initiated
  byteA indexRgb, depth;
  robot.glGetMasks(580, 480);
  write_ppm(robot.gl().captureImage, "z.rgb.ppm");
  write_ppm(convert<byte>(255.f*robot.gl().captureDepth), "z.depth.ppm");
}

// ============================================================================

int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);

  testGetRobotMask();

  return 0;
}


