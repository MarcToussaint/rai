#include <Gui/opengl.h>

void draw1(void*,OpenGL& gl){
  glStandardLight(nullptr, gl);
  glColor(1,0,0);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
  glDrawAxes(1.);
  glFrontFace(GL_CCW);
}

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  if(false){
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    glutInit(&argc, argv);
  }

  bool offscreen=true;

  OpenGL gl("bla", 800, 600, offscreen);
  gl.camera.setZRange(8, 10);
  gl.add(draw1,0);
//  gl.update();
  gl.renderInBack();

  write_ppm(gl.captureImage, "z.ppm", true);
  arr depth;
  rai::copy(depth, gl.captureDepth);
  depth *= 255.;
  write_ppm(rai::convert<byte>(depth), "z.depth.ppm", true);

  if(!offscreen){
    gl.watch();
    OpenGL gl2("depth", 800, 600);
    gl2.watchImage(gl.captureImage, true, 1.);
    gl2.displayGrey(depth, true, 1.);
  }

  cout <<"DONE!" <<endl;

  return 0;
}
