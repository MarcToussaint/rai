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

  OpenGL gl("bla", 800, 600, true);
  gl.add(draw1,0);
//  gl.update();
//  gl.watch();
  gl.renderInBack();

  write_ppm(gl.captureImage, "z.ppm", true);
//  write_ppm(convert<byte>(255.f*gl.captureDepth), "z.ppm", true);

//  OpenGL gl2("depth", 800, 600);
//  gl2.displayGrey(convert<double>(gl.captureDepth), true, 1.);

//  gl.watch(); //if this is commented, never ever glut/gtk is initalized

  cout <<"DONE!" <<endl;

  return 0;
}
