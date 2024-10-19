#include <Gui/opengl.h>

#include <GL/gl.h>
#include <GL/glut.h>

//implement a drawer
struct MyDrawer : GLDrawer{
  void glDraw(OpenGL& gl){
    glStandardLight(nullptr, gl);
    glColor(1,0,0);
    glFrontFace(GL_CW);
    glutSolidTeapot(1.);
    glFrontFace(GL_CCW);
  }
};

void TEST(Mini) {
  OpenGL gl;
  MyDrawer d;
//  gl.reportEvents=true;
//  gl.reportSelects=true;
  gl.add(d);
  gl.watch();
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  testMini();

  rai::wait();
  return 0;
}

