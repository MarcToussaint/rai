/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//void glDrawFloor(float, float, float, float){ NICO }
//void glGrabImage(rai::Array<unsigned char>&){ NICO }
//void glStandardLight(void*){ NICO }
//void glDrawAxes(double){ NICO }
//void glDrawPointCloud(rai::Array<double>&, rai::Array<double>&) { NICO }
//void glDrawSphere(float) { NICO }
//void glDrawCappedCylinder(float, float) { NICO }
//void glDrawText(char const*, float, float, float){ NICO }
//void glDrawDiamond(float, float, float){ NICO }
//void glDrawDisk(float){ NICO }
//void glDrawBox(float, float, float){ NICO }
//void glDrawCylinder(float, float, bool){ NICO }

//void OpenGL::watchImage(rai::Array<unsigned char> const&, bool, float){}

struct GlutInitializer {
  Mutex lock;
  GlutInitializer() {
    lock.lock();
    int argc=1;
    char* argv[1]= {(char*)"x"};
//    glutInit(&argc, argv);
    lock.unlock();
  }
  ~GlutInitializer() {
  }
};

Singleton<GlutInitializer> SingleGlut;

struct sOpenGL {
  sOpenGL(OpenGL* gl) {
    RAI_MSG("creating dummy OpenGL object");
    gl->width=1;
    gl->height=1;
    SingleGlut();
  }
  sOpenGL(OpenGL* gl, void* container) {
    RAI_MSG("creating dummy OpenGL object");
    gl->width=1;
    gl->height=1;
  }
  void beginGlContext() {}
  void endGlContext() {}
  rai::Vector downVec, downPos, downFoc;
  rai::Quaternion downRot;
};

void OpenGL::postRedrawEvent(bool) {}
void OpenGL::processEvents() {}
//void OpenGL::enterEventLoop(){}
//void OpenGL::exitEventLoop(){}
void OpenGL::resize(int w, int h) {}

void initGlEngine() {}
