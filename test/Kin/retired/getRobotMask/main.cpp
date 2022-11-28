#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>

void TEST(GetRobotMask){
  rai::Configuration robot("../../../../rai-robotModels/pr2/pr2.g");
  robot.gl().camera.setKinect();
  robot.gl().camera.X = robot.getFrameByName("endeffEyes")->ensure_X() * robot.gl().camera.X;
  robot.view(true); //if commented, glut/gtk is never initiated
  byteA indexRgb, depth;
  robot.glGetMasks(580, 480);
  write_ppm(robot.gl().captureImage, "z.rgb.ppm");
  write_ppm(convert<byte>(255.f*robot.gl().captureDepth), "z.depth.ppm");
}

// ============================================================================

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

  testGetRobotMask();

  cout <<"DONE -- view the output images" <<endl;

  return 0;
}

