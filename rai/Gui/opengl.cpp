/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_GL
#  ifndef RAI_MSVC
#    include <GL/glew.h>
#    include <GL/glx.h>
#  else
#    include <windows.h>
#    undef min
#    undef max
#    include <GL/glew.h>
#    include <GL/glut.h>
#  endif
#  undef Success
#endif

#include "opengl.h"
#include "../Core/array.ipp"
#include "../Geo/geo.h"

#ifdef RAI_GLFW
#  include <GLFW/glfw3.h>
#endif

#ifdef RAI_PNG
#  include <png.h>
#  include <unistd.h>
#endif

OpenGL& NoOpenGL = *((OpenGL*)(nullptr));

OpenGLDrawOptions& GLDrawer::glDrawOptions(OpenGL& gl){ return gl.drawOptions; }

//===========================================================================

Singleton<SingleGLAccess> singleGLAccess;

//===========================================================================

#ifdef RAI_FREEGLUT

#include <GL/freeglut.h>
//===========================================================================
//
// A freeglut singleton to ensure once initialization
//

Mutex& OpenGLMutex();

class FreeglutSpinner : Thread {
 private:
  uint numWins;
  rai::Array<OpenGL*> glwins;

 public:
  FreeglutSpinner() : Thread("FreeglutSpinner", .01), numWins(0) {
    int argc=1;
    char* argv[1]= {(char*)"x"};
    glutInit(&argc, argv);
  }
  ~FreeglutSpinner() {
//    uint i=0;  for(OpenGL* gl:glwins){ if(gl) delGL(i, gl); i++; }
//    th.threadClose();
    if(numWins) rai::wait(.1);
    CHECK(!numWins, "there are still OpenGL windows open");
    glutExit(); //also glut as already shut down during deinit
    threadClose();
  }

  void addGL(uint i, OpenGL* gl) {
    if(glwins.N<=i) glwins.resizeCopy(i+1);
    glwins(i) = gl;
    if(!numWins) threadLoop(); //start looping
    numWins++;
  }

  void delGL(uint i, OpenGL* gl) {
    CHECK_EQ(glwins(i), gl, "");
    glwins(i)=nullptr;
    numWins--;
    if(!numWins) { //stop looping
      OpenGLMutex().unlock();
      threadClose();
      OpenGLMutex().lock(RAI_HERE);
      for(uint i=0; i<10; i++) glutMainLoopEvent(); //ensure that all windows are being closed
    }
  }

  OpenGL* getGL(uint i) {
    return glwins(i);
  }

  void open() {}
  void step() {
    OpenGLMutex().lock(RAI_HERE);
    glutMainLoopEvent();
    OpenGLMutex().unlock();
  }
  void close() {}
};

Singleton<FreeglutSpinner> singletonGlSpinner; //();singleGlProcess;

Mutex& OpenGLMutex() { return singletonGlSpinner.mutex; }

//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL : NonCopyable {
  sOpenGL(OpenGL* gl) {}

  int windowID=-1;                        ///< id of this window in the global glwins list

  //-- callbacks
  static void _Void() { }
  static void _Draw() { auto fg=singletonGlSpinner();  OpenGL* gl=fg->getGL(glutGetWindow()); gl->Draw(gl->width, gl->height); glutSwapBuffers(); gl->isUpdating.setStatus(0); }
  static void _Key(unsigned char key, int x, int y) {        singletonGlSpinner()->getGL(glutGetWindow())->Key(key); }
  static void _Mouse(int button, int updown, int x, int y) { singletonGlSpinner()->getGL(glutGetWindow())->MouseButton(button, updown, x, y); }
  static void _Motion(int x, int y) {                        singletonGlSpinner()->getGL(glutGetWindow())->MouseMotion(x, y); }
  static void _PassiveMotion(int x, int y) {                 singletonGlSpinner()->getGL(glutGetWindow())->MouseMotion(x, y); }
  static void _Reshape(int w, int h) {                        singletonGlSpinner()->getGL(glutGetWindow())->Reshape(w, h); }
  static void _MouseWheel(int wheel, int dir, int x, int y) { singletonGlSpinner()->getGL(glutGetWindow())->Scroll(wheel, dir); }
  static void _WindowStatus(int status)                     { singletonGlSpinner()->getGL(glutGetWindow())->WindowStatus(status); }
};

//===========================================================================
//
// OpenGL implementations
//

void OpenGL::openWindow() {
  if(self->windowID==-1) {
    {
      auto fg = singletonGlSpinner();
      glutInitWindowSize(width, height);
      //  glutInitWindowPosition(posx,posy);
      glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

      self->windowID = glutCreateWindow(title);
      fg->addGL(self->windowID, this);

      glutDisplayFunc(self->_Draw);
      glutKeyboardFunc(self->_Key);
      glutMouseFunc(self->_Mouse) ;
      glutMotionFunc(self->_Motion) ;
      glutPassiveMotionFunc(self->_PassiveMotion) ;
      glutReshapeFunc(self->_Reshape);
      glutMouseWheelFunc(self->_MouseWheel) ;
      glutWindowStatusFunc(self->_WindowStatus);

      glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    }
  }
}

void OpenGL::closeWindow() {
  if(self->windowID!=-1) {
    auto fg=singletonGlSpinner();
    glutDestroyWindow(self->windowID);
    fg->delGL(self->windowID, this);
  }
  self->windowID=-1;
}

void OpenGL::postRedrawEvent(bool fromWithinCallback) {
  auto fg=singletonGlSpinner();
  openWindow();
  glutSetWindow(self->windowID);
  glutPostRedisplay();
//  glXMakeCurrent(fgDisplay.Display, None, nullptr);
}

void OpenGL::resize(int w, int h) {
  if(self->windowID==-1) {
    Reshape(w, h);
  } else {
    auto fg=singletonGlSpinner();
    glutSetWindow(self->windowID);
    glutReshapeWindow(w, h);
//    glXMakeCurrent(fgDisplay.Display, None, nullptr);
  }
}

void OpenGL::beginNonThreadedDraw() {
  openWindow();
  NIY;
//  auto fg = singletonGlSpinner();
//  fg->mutex.lock(RAI_HERE);
//  glutSetWindow(self->windowID);
//  glfwMakeContextCurrent(self->window);
}

void OpenGL::endNonThreadedDraw() {
//  auto fg = singletonGlSpinner();
//  glfwSwapBuffers(self->window);
//  fg->mutex.unlock();
}

#elif 0

void OpenGL::openWindow() {}
void OpenGL::closeWindow() {}
void OpenGL::postRedrawEvent(bool fromWithinCallback) {}
void OpenGL::resize(int w, int h) {}

struct sOpenGL {
  sOpenGL(OpenGL* gl): gl(gl), windowID(-1) {}
  sOpenGL(OpenGL* gl, void* container) { NIY }
  ~sOpenGL() { gl->closeWindow(); }

  void beginGlContext() {}
  void endGlContext() {}

  //-- private OpenGL data
  OpenGL* gl;
  rai::Vector downVec, downPos, downFoc;
  rai::Quaternion downRot;

  //-- engine specific data
  int windowID;                        ///< id of this window in the global glwins list

  //-- callbacks
  // static void _Void() { }
  // static void _Draw() { auto fg=singletonGlSpinner();  OpenGL *gl=fg->getGL(glutGetWindow()); gl->Draw(gl->width,gl->height); glutSwapBuffers(); gl->isUpdating.setStatus(0); }
  // static void _Key(unsigned char key, int x, int y) {        singletonGlSpinner()->getGL(glutGetWindow())->Key(key,x,y); }
  // static void _Mouse(int button, int updown, int x, int y) { singletonGlSpinner()->getGL(glutGetWindow())->Mouse(button,updown,x,y); }
  // static void _Motion(int x, int y) {                        singletonGlSpinner()->getGL(glutGetWindow())->Motion(x,y); }
  // static void _PassiveMotion(int x, int y) {                 singletonGlSpinner()->getGL(glutGetWindow())->Motion(x,y); }
  // static void _Reshape(int w,int h) {                        singletonGlSpinner()->getGL(glutGetWindow())->Reshape(w,h); }
  // static void _MouseWheel(int wheel, int dir, int x, int y){ singletonGlSpinner()->getGL(glutGetWindow())->MouseWheel(wheel,dir,x,y); }

  void accessWindow() {  //same as above, but also sets gl cocntext (glXMakeCurrent)
  }
  void deaccessWindow() {
  }
};
#endif

#ifdef RAI_GLFW

//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL : NonCopyable {
  sOpenGL(OpenGL* gl) {}

  int needsRedraw=0;

  //-- private OpenGL data

  //-- engine specific data
  GLFWwindow* window=0;
};

struct GlfwSpinner : Thread {
  rai::Array<OpenGL*> glwins;
  Mutex mutex;

  GlfwSpinner() : Thread("GlfwSpinnerSpinner", .01) {
    if(rai::getDisableGui()){ HALT("you must not be here with -disableGui"); }

    glfwSetErrorCallback(error_callback);
    if(!glfwInit()) exit(EXIT_FAILURE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    int argc=1;
    char* argv[1]= {(char*)"x"};
    glutInit(&argc, argv);

    threadLoop(true);
  }
  ~GlfwSpinner() {
    threadClose();
    glfwTerminate();
  }
  void open() {}
  void step() {
//    glfwWaitEvents();
//    glfwWaitEventsTimeout(.1);
//    static uint count=0;
//    cout <<"HERE" <<count++;
    mutex.lock(RAI_HERE);
    glfwPollEvents();
    for(OpenGL* gl: glwins) if(gl->self && !gl->offscreen && gl->self->window && gl->self->needsRedraw) {
        gl->isUpdating.setStatus(1);

        glfwMakeContextCurrent(gl->self->window);
        gl->Draw(gl->width, gl->height);
        glfwSwapBuffers(gl->self->window);
        glfwMakeContextCurrent(nullptr);

        gl->self->needsRedraw--;
        gl->isUpdating.setStatus(0);
      }
    mutex.unlock();
//    cout <<endl;
  }
  void close() {}

  void addGL(OpenGL* gl) {
//    bool start=false;
    mutex.lock(RAI_HERE);
    glwins.append(gl);
#if 0
    gl->self->needsRedraw = 10;
#else
    glfwMakeContextCurrent(gl->self->window);
    gl->Draw(gl->width, gl->height);
    glfwSwapBuffers(gl->self->window);
    glfwMakeContextCurrent(nullptr);
#endif
//    if(glwins.N==1) start=true; //start looping
    mutex.unlock();

//    if(start) threadLoop(true); //start looping
  }

  void delGL(OpenGL* gl) {
//    bool stop=false;
    mutex.lock(RAI_HERE);
    glwins.removeValue(gl);
//    if(!glwins.N) stop=true; //stop looping
    mutex.unlock();

//    if(stop) threadStop(); //stop looping
  }

  static void error_callback(int error, const char* description) {
    HALT("GLFW error " <<error <<": " <<description);
  }

  static void _MouseButton(GLFWwindow* window, int button, int action, int mods) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    if(button==GLFW_MOUSE_BUTTON_RIGHT) button = 2;
    else if(button==GLFW_MOUSE_BUTTON_MIDDLE) button = 1;
    gl->MouseButton(button, 1-action, xpos, ypos, mods);
  }

  static void _MouseMotion(GLFWwindow* window, double xpos, double ypos) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    gl->MouseMotion(xpos, ypos);
  }

  static void _Key(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(action == GLFW_PRESS) {
      OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
      if(key==256) key=27;
      if(key==257) key=13;
      if(key>='A' && key<='Z') key += 'a' - 'A';
      gl->Key(key, mods);
    }
  }

  static void _Resize(GLFWwindow* window, int width, int height) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    gl->Reshape(width, height);
  }

  static void _Close(GLFWwindow* window) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
//    LOG(-1) <<"closing window";
//      if (!time_to_close)
//    OpenGL *gl=(OpenGL*)glfwSetWindowShouldClose(window, GLFW_FALSE);
//    gl->WindowStatus(0);
    glfwHideWindow(window);
//    gl->closeWindow();
  }

  static void _Scroll(GLFWwindow* window, double xoffset, double yoffset) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    gl->Scroll(0, yoffset);
  }

  static void _Refresh(GLFWwindow* window){
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    gl->postRedrawEvent(true);
  }
};

static GlfwSpinner* singletonGlSpinner() {
  static GlfwSpinner instance;
  return &instance;
}

void OpenGL::openWindow() {
  if(rai::getDisableGui()) return;

  if(!self->window) {
    auto fg = singletonGlSpinner();
    fg->mutex.lock(RAI_HERE);

    if(offscreen) {
//      glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_NATIVE_CONTEXT_API);
//      glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_ROBUSTNESS);
//      glfwWindowHintString()
//      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
//      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
//      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
      glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
//      glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    } else {
      glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    }
    if(!title.N) title="GLFW window";
    self->window = glfwCreateWindow(width, height, title.p, nullptr, nullptr);
    if(!offscreen){
      glfwMakeContextCurrent(self->window);
      glfwSetWindowUserPointer(self->window, this);
      glfwSetMouseButtonCallback(self->window, GlfwSpinner::_MouseButton);
      glfwSetCursorPosCallback(self->window, GlfwSpinner::_MouseMotion);
      glfwSetKeyCallback(self->window, GlfwSpinner::_Key);
      glfwSetScrollCallback(self->window, GlfwSpinner::_Scroll);
      glfwSetWindowSizeCallback(self->window, GlfwSpinner::_Resize);
      glfwSetWindowCloseCallback(self->window, GlfwSpinner::_Close);
      glfwSetWindowRefreshCallback(self->window, GlfwSpinner::_Refresh);
 
      glfwSwapInterval(1);
      glfwMakeContextCurrent(nullptr);
    }

    fg->mutex.unlock();

    fg->addGL(this);
  }else{
    //glfwShowWindow(self->window);
  }
}

void OpenGL::closeWindow() {
  self->needsRedraw=0;
  if(self->window) {
    singletonGlSpinner()->delGL(this);
    {
      auto fg = singletonGlSpinner();
      fg->mutex.lock(RAI_HERE);
      glfwDestroyWindow(self->window);
      fg->mutex.unlock();
      isUpdating.setStatus(0);
      watching.setStatus(0);
    }
  }
}

void OpenGL::setTitle(const char* _title) {
  if(_title) title = _title;
  if(self->window) {
    glfwSetWindowTitle(self->window, title.p);
  }
}

void OpenGL::beginNonThreadedDraw() {
  if(rai::getDisableGui()) return;
  openWindow();
  auto fg = singletonGlSpinner();
  fg->mutex.lock(RAI_HERE);
  glfwMakeContextCurrent(self->window);
}

void OpenGL::endNonThreadedDraw() {
  if(rai::getDisableGui()) return;
  auto fg = singletonGlSpinner();
  glfwSwapBuffers(self->window);
  glfwMakeContextCurrent(nullptr);
  fg->mutex.unlock();
}

void OpenGL::postRedrawEvent(bool fromWithinCallback) {
  auto fg = singletonGlSpinner();
  if(!fromWithinCallback) fg->mutex.lock(RAI_HERE);
  if(!self->needsRedraw) self->needsRedraw=1;
  if(!fromWithinCallback) fg->mutex.unlock();
}

void OpenGL::resize(int w, int h) {
  openWindow();
  Reshape(w, h);
  glfwSetWindowSize(self->window, width, height);
}

#endif

#ifndef RAI_GL
int GLUT_ACTIVE_SHIFT = 1;

void OpenGL::openWindow() {}
void OpenGL::closeWindow() {}
void OpenGL::postRedrawEvent(bool fromWithinCallback) {}
void OpenGL::resize(int w, int h) {}

struct sOpenGL : NonCopyable {
  sOpenGL(OpenGL* gl) { }
};

#endif

//===========================================================================
//
// force instantiations
//

template rai::Array<glUI::Button>::Array();
template rai::Array<glUI::Button>::~Array();

//===========================================================================
//
// static objects
//

OpenGL* staticgl [10]; //ten pointers to be potentially used as display windows
uint OpenGL::selectionBuffer[1000];

//===========================================================================
//
// utility implementations
//

void id2color(byte rgb[3], uint id) {
  rgb[0] = ((id>> 6)&0x3f) | ((id&1)<<7) | ((id& 8)<<3);
  rgb[1] = ((id>>12)&0x3f) | ((id&2)<<6) | ((id&16)<<2);
  rgb[2] = ((id>>18)&0x3f) | ((id&4)<<5) | ((id&32)<<1);
}

uint color2id(byte rgb[3]) {
  uint id = 0;
  id |= (rgb[0]&0x80)>>7 | (rgb[1]&0x80)>>6 | (rgb[2]&0x80)>>5;
  id |= (rgb[0]&0x40)>>3 | (rgb[1]&0x40)>>2 | (rgb[2]&0x40)>>1;
  id |= (rgb[0]&0x3f)<<6 | (rgb[1]&0x3f)<<12 | (rgb[2]&0x3f)<<18;
  return id;
}

arr id2color(uint id) {
  byteA rgb(3);
  id2color(rgb.p, id);
  return ARR(rgb(0)/256., rgb(1)/256., rgb(2)/256.);
}

#ifdef RAI_GL
void glStandardLight(void*, OpenGL&) {
  glEnable(GL_LIGHTING);
  static GLfloat ambient[]   = { .5, .5, .5, 1.0 };
  static GLfloat diffuse[]   = { .2, .2, .2, 1.0 };
  static GLfloat specular[]  = { .3, .3, .3, 1.0 };
  static GLfloat position[]  = { 100.0, -100.0, 100.0, 1.0 };
  static GLfloat direction[] = { -1.0, 1.0, -1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, direction);
  glLighti(GL_LIGHT0, GL_SPOT_CUTOFF,   90);
  glLighti(GL_LIGHT0, GL_SPOT_EXPONENT, 10);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  glEnable(GL_LIGHT0);
}

void glStandardScene(void*, OpenGL& gl) {
  glPushAttrib(GL_CURRENT_BIT);
  glStandardLight(nullptr, gl);
  glDrawFloor(10, .5, .55, .6);
  // glDrawFloor(10, 1.5, 0.83, .0);
  // glDrawFloor(10., 108./255., 123./255., 139./255.);
  glDrawAxes(.1);
  glPopAttrib();
}

void glStandardOriginAxes(void*, OpenGL&) {
  glDrawAxes(.1);
}

void glColor(int col) {
  static const GLfloat colorsTab[6][4] = {
    {0.2, 0.2, 1.0, 1.0}, // blue
    {1.0, 0.8, 0.0, 1.0}, // gold
    {1.0, 0.0, 0.0, 1.0}, // red
    {0.7, 0.7, 0.7, 1.0}, // gray
    {1.0, 1.0, 1.0, 1.0}, // white
    {0.2, 1.0, 0.2, 1.0}
  }; // green

  col = col%6; //if(col<0) col=0; if(col>5) col=5;
  glColor(colorsTab[col][0], colorsTab[col][1], colorsTab[col][2], colorsTab[col][3]);
}

void glColor(float r, float g, float b, float alpha) {
  float amb=1.f, diff=1.f, spec=.25f;
  GLfloat ambient[4]  = { r*amb, g*amb, b*amb, alpha };
  GLfloat diffuse[4]  = { r*diff, g*diff, b*diff, alpha };
  GLfloat specular[4] = { spec* (1.f+r), spec* (1.f+g), spec* (1.f+b), alpha };
#if 0
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
#else
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0f);
#endif
  glColor4f(r, g, b, alpha);
}

void glColor(float* rgb) { glColor(rgb[0], rgb[1], rgb[2], 1.); }

void glColor(const arr& col) {
  if(col.N==3) glColor(col.p[0], col.p[1], col.p[2], 1.);
  if(col.N==4) glColor(col.p[0], col.p[1], col.p[2], col.p[3]);
}

void glColorId(uint id) {
  byte rgb[3];
  glDisable(GL_LIGHTING);
  id2color(rgb, id);
  glColor3ubv(rgb);
}

void OpenGL::drawId(uint id) {
  if(drawOptions.drawMode_idColor) {
    glColorId(id);
    drawOptions.drawColors=false;
  } else {
    drawOptions.drawColors=true;
  }
}

/* // shadows do not work with a light source;
   // thus, we need to leave this out. 4. Mar 06 (hh)
void glShadowTransform()
{
  GLfloat matrix[16];
  for(int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-1;  //light_x
  matrix[9]=-1;  //light_y
  matrix[14]=.02; //ground offset
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}
*/

void glTransform(const rai::Transformation& t) {
  double GLmatrix[16];
  t.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
}

void glRotate(const rai::Quaternion& rot) {
  double GLmatrix[16];
  rot.getMatrixGL(GLmatrix);
  glMultMatrixd(GLmatrix);
}

void glTransform(const double pos[3], const double R[12]) {
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf(matrix);
}

static GLboolean glLightIsOn = false;
void glPushLightOff() { glGetBooleanv(GL_LIGHTING, &glLightIsOn); glDisable(GL_LIGHTING); }
void glPopLight() { if(glLightIsOn) glEnable(GL_LIGHTING); }

void glDrawText(const char* txt, float x, float y, float z, bool largeFont) {
  if(!txt) return;
  glDisable(GL_DEPTH_TEST);
  glPushLightOff();
  glRasterPos3f(x, y, z);
  void* font=GLUT_BITMAP_HELVETICA_12;
  if(largeFont) font = GLUT_BITMAP_HELVETICA_18;
  while(*txt) {
    switch(*txt) {
      case '\n':
        y+=15;
        glRasterPos3f(x, y, z);
        break;
      case '\b':
        if(font==GLUT_BITMAP_HELVETICA_12) font=GLUT_BITMAP_HELVETICA_18;
        else font=GLUT_BITMAP_HELVETICA_12;
        break;
      default: {
        glutBitmapCharacter(font, *txt);
      }
    }
    txt++;
  }
  glPopLight();
  glEnable(GL_DEPTH_TEST);
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4,
                float r, float g, float b) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glColor(r, g, b);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x, float y, float z, float r) {
  glDrawRect(x-r, y-r, z, x-r, y+r, z, x+r, y+r, z, x+r, y-r, z);
}

void glDrawPolygon(const arr& P) {
  CHECK_EQ(P.nd, 2, "");
  CHECK_EQ(P.d1, 3, "");
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glBegin(GL_POLYGON);
////  glColor(r, g, b);
//  for(uint i=0;i<P.d0;i++) glVertex3dv(&P(i,0));
//  glVertex3dv(P.p);
//  glEnd();
  glLineWidth(5);
  glBegin(GL_LINE_LOOP);
  for(uint i=0; i<P.d0; i++) glVertex3dv(&P(i, 0));
  glEnd();
}

void glDrawFloor(float x, float r, float g, float b) {
  x/=2.;
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor(r, g, b);
  glBegin(GL_POLYGON);
  glNormal3f(0, 0, 1);
  glVertex3f(-x, -x, 0.);
  glVertex3f(x, -x, 0.);
  glVertex3f(x, x, 0.);
  glVertex3f(-x, x, 0.);
  glVertex3f(-x, -x, 0.);
  glEnd();
#if 1
  glColor(.75, .75, .75);
  for(int i=-4; i<=4; i++) {
    glBegin(GL_LINES);
    glVertex3f(i*x/5., -x, 0.001);
    glVertex3f(i*x/5., x, 0.001);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, i*x/5., 0.001);
    glVertex3f(x, i*x/5., 0.001);
    glEnd();
  }

  glColor(.25, .25, .25);
  glBegin(GL_LINE_STRIP);
  glVertex3f(-x, -x, 0.002);
  glVertex3f(-x, x, 0.002);
  glVertex3f(x, x, 0.002);
  glVertex3f(x, -x, 0.002);
  glVertex3f(-x, -x, 0.002);
  glEnd();
#endif
}

void glDrawBox(float x, float y, float z, bool linesOnly) {
  static GLfloat n[6][3] = {
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0}
  };
  static GLint faces[6][4] = {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  static GLint edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}
  };
  GLfloat v[8][3];
  GLint i;

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -x / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] =  x / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] =  -y / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] =  y / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] =  -z / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] =  z / 2;

  if(!linesOnly) {
    glBegin(GL_QUADS);
    for(i = 5; i >= 0; i--) {
      glNormal3fv(n[i]);
      glVertex3fv(v[faces[i][0]]);
      glVertex3fv(v[faces[i][1]]);
      glVertex3fv(v[faces[i][2]]);
      glVertex3fv(v[faces[i][3]]);
    }
    glEnd();
  } else {
    glBegin(GL_LINES);
    for(uint i=0; i<12; i++) {
      glVertex3fv(v[edges[i][0]]);
      glVertex3fv(v[edges[i][1]]);
    }
    glEnd();
  }
}

void glDrawDiamond(float x, float y, float z) {
//  glDisable(GL_CULL_FACE);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, -z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
//  glEnable(GL_CULL_FACE);
}

void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz) {
  glPushMatrix();
  glTranslated(x, y, z);
  glDrawDiamond(dx, dy, dz);
  glPopMatrix();
}

void glDrawAxis(double scale) {
//    glDisable(GL_CULL_FACE);
  if(scale>=0) glPushMatrix();
  if(scale>=0) glScalef(scale, scale, scale);
  GLUquadric* style=gluNewQuadric();
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(.95, 0, 0);
  glEnd();
  glTranslatef(.8, 0, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(style, .08, 0, .2, 20, 1);
  gluDeleteQuadric(style);
  if(scale>=0) glPopMatrix();
//    glEnable(GL_CULL_FACE);
}

void glDrawAxes(double scale, bool colored) {
  for(uint i=0; i<3; i++) {
    glPushMatrix();
    glScalef(scale, scale, scale);

    switch(i) {
      case 0:  if(colored) glColor(.7, 0, 0);  break;
      case 1:  if(colored) glColor(0, .7, 0);  glRotatef(90, 0, 0, 1);  break;
      case 2:  if(colored) glColor(0, 0, .7);  glRotatef(90, 0, -1, 0);  break;
    }
    glDrawAxis();
    glPopMatrix();
  }
}

void glDrawCamera(const rai::Camera& cam) {
  glTransform(cam.X);

  glDrawAxes(.1);

  double dxFar, dyFar, zFar;
  double dxNear, dyNear, zNear;
  zNear = cam.zNear;
  zFar = cam.zFar;
  if(zFar-zNear > 10.) zFar = zNear + .1;
  if(cam.focalLength) {
    dyNear = zNear * .5/cam.focalLength;
    dyFar = zFar * .5/cam.focalLength;
    dxNear = cam.whRatio * dyNear;
    dxFar = cam.whRatio * dyFar;
  } else {
    CHECK(cam.heightAbs, "");
    dyFar = dyNear = cam.heightAbs/2.;
    dxFar = dxNear = cam.whRatio * dyNear;
  }
  glColor(.5, .5, .5);
  glBegin(GL_LINE_STRIP);
  glVertex3f(-dxNear, -dyNear, -zNear);
  glVertex3f(-dxNear, dyNear, -zNear);
  glVertex3f(dxNear, dyNear, -zNear);
  glVertex3f(dxNear, -dyNear, -zNear);
  glVertex3f(-dxNear, -dyNear, -zNear);
  glEnd();
  glBegin(GL_LINE_STRIP);
  glVertex3f(-dxFar, -dyFar, -zFar);
  glVertex3f(-dxFar, dyFar, -zFar);
  glVertex3f(dxFar, dyFar, -zFar);
  glVertex3f(dxFar, -dyFar, -zFar);
  glVertex3f(-dxFar, -dyFar, -zFar);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(-dxNear, -dyNear, -zNear);
  glVertex3f(0, 0, 0);
  glVertex3f(-dxNear, dyNear, -zNear);
  glVertex3f(0, 0, 0);
  glVertex3f(dxNear, -dyNear, -zNear);
  glVertex3f(0, 0, 0);
  glVertex3f(dxNear, dyNear, -zNear);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(-dxNear, -dyNear, -zNear);
  glVertex3f(-dxFar,  -dyFar,  -zFar);
  glVertex3f(-dxNear, dyNear, -zNear);
  glVertex3f(-dxFar,  dyFar,  -zFar);
  glVertex3f(dxNear, -dyNear, -zNear);
  glVertex3f(dxFar,  -dyFar,  -zFar);
  glVertex3f(dxNear, dyNear, -zNear);
  glVertex3f(dxFar,  dyFar,  -zFar);
  glEnd();
}

void glDrawDisk(float radius) {
  GLUquadric* style=gluNewQuadric();
  gluDisk(style, 0, radius, 10, 1);
  gluDeleteQuadric(style);
}

void glDrawProxy(const arr& p1, const arr& p2, double diskSize, int colorCode, const arr& norm, double _rad1, double _rad2) {
  glLoadIdentity();
  if(!colorCode) glColor(.8, .2, .2);
  else glColor(colorCode);
  glBegin(GL_LINES);
  glVertex3dv(p1.p);
  glVertex3dv(p2.p);
  glEnd();
  glDisable(GL_CULL_FACE);
  rai::Transformation f;
  f.pos=p1;
  if(!!norm) {
    f.rot.setDiff(rai::Vector(0, 0, 1), rai::Vector(norm));
  } else {
    f.rot.setDiff(rai::Vector(0, 0, 1), rai::Vector(p1-p2));
  }
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(diskSize);

  f.pos=p2;
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(diskSize);
  glEnable(GL_CULL_FACE);

  glLoadIdentity();
  if(!!norm && _rad1>0.) {
    arr p = p1 - _rad1*norm;
    glColor(0., 1., 0., 1.);
    glDrawDiamond(p(0), p(1), p(2), .01, .01, .01);
  }
  if(!!norm && _rad1>0.) {
    arr p = p2 + _rad2*norm;
    glColor(0., 0., 1., 1.);
    glDrawDiamond(p(0), p(1), p(2), .01, .01, .01);
  }
}

void glDrawSphere(float radius) {
  GLUquadric* style=gluNewQuadric();
  gluSphere(style, radius, 10, 10); // last two value for detail
  gluDeleteQuadric(style);
}

void glDrawCylinder(float radius, float length, bool closed) {
  GLUquadric* style=gluNewQuadric();
  glTranslatef(0, 0, -length/2);
  gluCylinder(style, radius, radius, length, 20, 1);
  if(closed) {
    glScalef(-1, 1, 1);  //flip orientation of triangles...
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, length);
    glScalef(-1, 1, 1);
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, -length/2);
  } else {
    glTranslatef(0, 0, length/2);
  }
  gluDeleteQuadric(style);
}

void glDrawCappedCylinder(float radius, float length) {
  GLUquadric* style1=gluNewQuadric();
  GLUquadric* style2=gluNewQuadric();
  GLUquadric* style3=gluNewQuadric();

  glTranslatef(0, 0, -length/2);
  gluCylinder(style1, radius, radius, length, 20, 1);
  glTranslatef(0, 0, length);
  gluSphere(style2, radius, 10, 10);
  glTranslatef(0, 0, -length);
  gluSphere(style3, radius, 10, 10);

  gluDeleteQuadric(style1);
  gluDeleteQuadric(style2);
  gluDeleteQuadric(style3);
}

void glDrawGridBox(float x=10.) {
  x/=2.;
  glBegin(GL_LINE_LOOP);
  glVertex3f(-x, -x, -x);
  glVertex3f(-x, -x, x);
  glVertex3f(-x, x, x);
  glVertex3f(-x, x, -x);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(x, -x, -x);
  glVertex3f(x, -x, x);
  glVertex3f(x, x, x);
  glVertex3f(x, x, -x);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(x, x, x);
  glVertex3f(-x, x, x);
  glVertex3f(x, -x, x);
  glVertex3f(-x, -x, x);
  glVertex3f(x, x, -x);
  glVertex3f(-x, x, -x);
  glVertex3f(x, -x, -x);
  glVertex3f(-x, -x, -x);
  glEnd();
}

void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2) {
  glBegin(GL_LINE_STRIP);
  glVertex3f(x1, y1, z1+0.001);
  glVertex3f(x2, y1, z1+0.001);
  glVertex3f(x2, y2, z1+0.001);
  glVertex3f(x1, y2, z1+0.001);
  glVertex3f(x1, y1, z1+0.001);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(x2, y2, z1 +0.001);
  glVertex3f(x2, y2, z2 +0.001);
  glEnd();
}

void glDrawKhepera() {
  GLUquadric* style=gluNewQuadric();
  glPushMatrix();
  glRotatef(-90, 1, 0, 0);
  glColor3f(.3, .3, .3);
  gluCylinder(style, 1.5, 1.5, 2, 20, 1);
  glPopMatrix();

  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 2, 0);
  glVertex3f(0, 2, -2.5);
  glEnd();
  gluDeleteQuadric(style);
}

void glMakeSquare(int num) {
  glNewList(num, GL_COMPILE);
  glColor3f(1., 0., 0.);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-1, -1, 0.);
  glVertex3f(-1, +1, 0.);
  glVertex3f(+1, +1, 0.);
  glVertex3f(+1, -1, 0.);
  glEnd();
  glEndList();
}

void glMakeStdSimplex(int num) {
  glNewList(num, GL_COMPILE);
  //glPolygonMode(GL_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLE_FAN);
  glColor3f(1., 1., 1.);
  glVertex3f(0., 0., 0.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glColor3f(0., 1., 0.);
  glVertex3f(0., 1., 0.);
  glColor3f(0., 0., 1.);
  glVertex3f(0., 0., 1.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glEnd();
  /*
    glColor4f(.5, .5, .5, .9);
    glBegin(GL_POLYGON);
    glVertex3f( 1. , 0. , 0. );
    glVertex3f( 0. , 1. , 0. );
    glVertex3f( 0. , 0. , 1. );
    glEnd();
  */
  glEndList();
}

void glMakeTorus(int num) {
  glNewList(num, GL_COMPILE);

  GLint i, j, rings, sides;
  float theta1, phi1, theta2, phi2;
  float v0[03], v1[3], v2[3], v3[3];
  float t0[03], t1[3], t2[3], t3[3];
  float n0[3], n1[3], n2[3], n3[3];
  float innerRadius=0.4;
  float outerRadius=0.8;
  float scalFac;

  rings = 8;
  sides = 10;
  scalFac=1/(outerRadius*2);

  for(i=0; i<rings; i++) {
    theta1 = (float)i * 2.0 * RAI_PI / rings;
    theta2 = (float)(i + 1) * 2.0 * RAI_PI / rings;
    for(j=0; j<sides; j++) {
      phi1 = (float)j * 2.0 * RAI_PI / sides;
      phi2 = (float)(j + 1) * 2.0 * RAI_PI / sides;

      v0[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[2] = innerRadius * sin(phi1);

      v1[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[2] = innerRadius * sin(phi1);

      v2[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[2] = innerRadius * sin(phi2);

      v3[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[2] = innerRadius * sin(phi2);

      n0[0] = cos(theta1) * (cos(phi1));
      n0[1] =-sin(theta1) * (cos(phi1));
      n0[2] = sin(phi1);

      n1[0] = cos(theta2) * (cos(phi1));
      n1[1] =-sin(theta2) * (cos(phi1));
      n1[2] = sin(phi1);

      n2[0] = cos(theta2) * (cos(phi2));
      n2[1] =-sin(theta2) * (cos(phi2));
      n2[2] = sin(phi2);

      n3[0] = cos(theta1) * (cos(phi2));
      n3[1] =-sin(theta1) * (cos(phi2));
      n3[2] = sin(phi2);

      t0[0] = v0[0]*scalFac + 0.5;
      t0[1] = v0[1]*scalFac + 0.5;
      t0[2] = v0[2]*scalFac + 0.5;

      t1[0] = v1[0]*scalFac + 0.5;
      t1[1] = v1[1]*scalFac + 0.5;
      t1[2] = v1[2]*scalFac + 0.5;

      t2[0] = v2[0]*scalFac + 0.5;
      t2[1] = v2[1]*scalFac + 0.5;
      t2[2] = v2[2]*scalFac + 0.5;

      t3[0] = v3[0]*scalFac + 0.5;
      t3[1] = v3[1]*scalFac + 0.5;
      t3[2] = v3[2]*scalFac + 0.5;

      if((i+j)%2) glColor3f(0., 1., 0.);
      else glColor3f(0., 0., 1.);

      glBegin(GL_POLYGON);
      glNormal3fv(n3); glTexCoord3fv(t3); glVertex3fv(v3);
      glNormal3fv(n2); glTexCoord3fv(t2); glVertex3fv(v2);
      glNormal3fv(n1); glTexCoord3fv(t1); glVertex3fv(v1);
      glNormal3fv(n0); glTexCoord3fv(t0); glVertex3fv(v0);
      glEnd();
    }
  }
  glEndList();
}

uint glImageTexture(const byteA& img) {
  GLuint texName;

  glEnable(GL_TEXTURE_2D);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &texName);

//  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texName);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  switch(img.d2) {
    case 0:
    case 1:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
      break;
    case 2:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
      break;
    case 3:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
      break;
    case 4:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
      break;
    default:
      HALT("no image fomat");
  }

  return texName;
}

void glDrawTexQuad(const byteA& texImg,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) {
  glDisable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //GL_MODULATE);
  if(texImg.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
  if(texImg.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.d1, texImg.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImg.p);

//  glBindTexture(GL_TEXTURE_2D, texture);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0,  mulY); glVertex3f(x1, y1, z1);
  glTexCoord2f(mulX, mulY); glVertex3f(x2, y2, z2);
  glTexCoord2f(mulX, 0.0);  glVertex3f(x3, y3, z3);
  glTexCoord2f(0.0,  0.0);  glVertex3f(x4, y4, z4);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);

}

#ifdef RAI_GLUT
/** @brief return the RGBA-image of scenery drawn just before; the image
  buffer has to have either 2 dimensions [width, height] for a
  gray-scale luminance image or 3 dimensions [width, height, 4] for an
  RGBA-image. */
void glGrabImage(byteA& image) {
  if(!image.N) image.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH), 3);
  CHECK(image.nd==2 || image.nd==3, "not an image format");
  GLint w=image.d1, h=image.d0;
  //CHECK(w<=glutGet(GLUT_WINDOW_WIDTH) && h<=glutGet(GLUT_WINDOW_HEIGHT), "grabbing large image from small window:" <<w <<' ' <<h <<' ' <<glutGet(GLUT_WINDOW_WIDTH) <<' ' <<glutGet(GLUT_WINDOW_HEIGHT));
  if(image.d1%4) {  //necessary: extend the image to have width mod 4
    uint add=4-(image.d1%4);
    if(image.nd==2) image.resize(image.d0, image.d1+add);
    if(image.nd==3) image.resize(image.d0, image.d1+add, image.d2);
  }
  glReadBuffer(GL_FRONT);
//  glReadBuffer(GL_BACK);

  //glPixelStorei(GL_PACK_SWAP_BYTES, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
  switch(image.d2) {
    case 0:
    case 1:
      glPixelTransferf(GL_RED_SCALE, .3333);
      glPixelTransferf(GL_GREEN_SCALE, .3333);
      glPixelTransferf(GL_BLUE_SCALE, .3333);
      glReadPixels(0, 0, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, image.p);
      glPixelTransferf(GL_RED_SCALE, 1.);
      glPixelTransferf(GL_GREEN_SCALE, 1.);
      glPixelTransferf(GL_BLUE_SCALE, 1.);
      break;
//    case 2:
//      //glReadPixels(0, 0, w, h, GL_GA, GL_UNSIGNED_BYTE, image.p);
//      break;
    case 3:
      glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, image.p);
      break;
    case 4:
#if defined RAI_SunOS
      glReadPixels(0, 0, w, h, GL_ABGR_EXT, GL_UNSIGNED_BYTE, image.p);
#else
#if defined RAI_Cygwin
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#else
      //glReadPixels(0, 0, w, h, GL_BGRA_EXT, GL_UNSIGNED_BYTE, image.p);
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#endif
#endif
      break;
    default: HALT("wrong image format");
  }
}
#else
void glGrabImage(byteA& image) { NICO }
#endif

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(byteA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK_EQ(depth.nd, 2, "depth buffer has to be either 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, depth.p);
}

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(floatA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK_EQ(depth.nd, 2, "depth buffer has to be 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth.p);
}

void glRasterImage(float x, float y, byteA& img, float zoom) {
  glRasterPos3f(x, y, 0.); //(int)(y+zoom*img.d0)); (the latter was necessary for other pixel/raster coordinates)
  glPixelZoom(zoom, -zoom);
  if(img.d1%4) {  //necessary: extend the image to have width mod 4
    uint P=img.d2;
    if(!P) P=1;
    uint add=4-(img.d1%4);
    img.reshape(img.d0, img.d1*P);
    img.insColumns(img.d1, add*P);
    if(P>1) img.reshape(img.d0, img.d1/P, P);
  }

  switch(img.d2) {
    case 0:
    case 1:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);        break;
    case 2:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);  break;
    case 3:  glDrawPixels(img.d1, img.d0, GL_RGB, GL_UNSIGNED_BYTE, img.p);              break;
    case 4:  glDrawPixels(img.d1, img.d0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);             break;
    default: HALT("no image format");
  };
}

int OpenGL::watchImage(const floatA& _img, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, _img);
  float x;
  for(uint i=0; i<img.N; i++) {
    x=_img.p[i];
    img.p[i] = (x<0.)?0:((x>255.)?255:x);
  }
  return watchImage(img, wait, _zoom);
}

int OpenGL::watchImage(const byteA& _img, bool wait, float _zoom) {
  background=_img;
  backgroundZoom=_zoom;
  //resize(img->d1*zoom,img->d0*zoom);
  if(wait) return watch();
  return update();
}

/*void glWatchImage(const floatA &x, bool wait, float zoom){
  double ma=x.max();
  double mi=x.min();
  if(wait) cout <<"watched image min/max = " <<mi <<' ' <<ma <<endl;
  byteA img;
  img.resize(x.d0*x.d1);
  img.setZero();
  for(uint i=0;i<x.N;i++){
    img(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  img.reshape(x.d0, x.d1);
  glWatchImage(img, wait, 20);
}*/

int OpenGL::displayGrey(const floatA& x, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, x);
  float mi=x.min(), ma=x.max();
  text.clear() <<"displayGrey" <<" max:" <<ma <<" min:" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  return watchImage(img, wait, _zoom);
}

int OpenGL::displayGrey(const arr& x, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, x);
  double mi=x.min(), ma=x.max();
  text.clear() <<"displayGrey" <<" max:" <<ma <<"min:" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  return watchImage(img, wait, _zoom);
}

int OpenGL::displayRedBlue(const arr& x, bool wait, float _zoom) {
  double mi=x.min(), ma=x.max();
  text.clear() <<"max=" <<ma <<"min=" <<mi <<endl;
//  cout <<"\rdisplay" <<win <<" max=" <<ma <<"min=" <<mi;
  static byteA img;
  img.resize(x.d0*x.d1, 3);
  img.setZero();
  for(uint i=0; i<x.N; i++) {
    if(x.elem(i)>0.) img(i, 0)=(byte)(255.*x.elem(i)/ma);
    if(x.elem(i)<0.) img(i, 2)=(byte)(255.*x.elem(i)/mi);
  }
  img.reshape(x.d0, x.d1, 3);
  return watchImage(img, wait, _zoom);
}

void glDrawUI(void* p, OpenGL& gl) {
  glPushName(0x10);
  ((glUI*)p)->glDraw(gl);
  glPopName();
}

bool glUI::hoverCallback(OpenGL& gl) {
  //bool b=
  checkMouse(gl.mouseposx, gl.mouseposy);
  //if(b) postRedrawEvent(false);
  return true;
}

bool glUI::clickCallback(OpenGL& gl) {
  bool b=checkMouse(gl.mouseposx, gl.mouseposy);
  if(b) gl.postRedrawEvent(true);
  int t=top;
  if(t!=-1) {
    cout <<"CLICK! on button #" <<t <<endl;
    gl.watching.setStatus(0);
    return false;
  }
  return true;
}

#ifdef RAI_FREEGLUT
void glSelectWin(uint win) {
  if(!staticgl[win]) staticgl[win]=new OpenGL;
  glutSetWindow(staticgl[win]->self->windowID);
}
#endif

#else /// RAI_GL
void glColor(int col) { NICO }
void glColor(float, float, float, float) { NICO }
void glDrawDiamond(float, float, float, float, float, float) { NICO }
void glDrawSphere(float radius) { NICO }
void glDrawFloor(float, float, float, float) { NICO }
void glDrawCappedCylinder(float, float) { NICO }
void glStandardLight(void*, OpenGL&) { NICO }
void glDrawAxis(double) { NICO }
void glDrawAxes(double, bool) { NICO }
void glDrawDiamond(float, float, float) { NICO }
void glDrawBox(float, float, float, bool) { NICO }
void glDrawCamera(const rai::Camera&) { NICO }
void glDrawCylinder(float, float, bool) { NICO }

// void glStandardLight(void*) { NICO }   // TOBIAS: das hier wird doch schon ueber opengl_void.cxx definiert
void glStandardScene(void*, OpenGL&) { NICO }
void glStandardOriginAxes(void*, OpenGL&) { NICO }
uint glImageTexture(const byteA& img) { NICO }
void glDrawText(const char* txt, float x, float y, float z, bool largeFont) { NICO }
void glDrawTexQuad(uint texture,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) { NICO }
int OpenGL::watchImage(const floatA& _img, bool wait, float _zoom) { NICO }
int OpenGL::watchImage(const byteA& _img, bool wait, float _zoom) { NICO }
void glDrawUI(void* p) { NICO }
bool glUI::hoverCallback(OpenGL& gl) { NICO }
bool glUI::clickCallback(OpenGL& gl) { NICO }
#endif

//===========================================================================
//
// standalone draw routines for large data structures
//

#ifdef RAI_GL
#endif

//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* _title, int w, int h, bool _offscreen)
  : title(_title), width(w), height(h), offscreen(_offscreen), reportEvents(false), topSelection(nullptr), fboId(0), rboColor(0), rboDepth(0) {
  //RAI_MSG("creating OpenGL=" <<this);
  self = make_unique<sOpenGL>(this); //this might call some callbacks (Reshape/Draw) already!
  init();
}

OpenGL::OpenGL(void* container)
  : width(0), height(0), reportEvents(false), topSelection(nullptr), fboId(0), rboColor(0), rboDepth(0) {
  self = make_unique<sOpenGL>(this); //this might call some callbacks (Reshape/Draw) already!
  init();
}

OpenGL::~OpenGL() {
  clear();
  closeWindow();
}

OpenGL* OpenGL::newClone() const {
  OpenGL* gl=new OpenGL;
  //*(gl->s) = *s; //don't clone internal stuff!
  gl->drawers = drawers;
  gl->camera = camera;
  return gl;
}

void OpenGL::init() {
  drawFocus=false;
  clearR=clearG=clearB=1.; clearA=0.;
  mouseposx=mouseposy=0;
  mouse_button=0;
  mouseIsDown=false;
  mouseView=-1;

  if(width%4) width = 4*(width/4);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);

  reportEvents=false;
  reportSelects=false;
  exitkeys="";

  backgroundZoom=1;
};

struct CstyleDrawer : GLDrawer {
  void* classP;
  void (*call)(void*, OpenGL&);
  CstyleDrawer(void (*call)(void*, OpenGL&), void* classP) : classP(classP), call(call) {}
  void glDraw(OpenGL& gl) { call(classP, gl); }
};

struct LambdaDrawer : GLDrawer {
  std::function<void(OpenGL&)> call;
  LambdaDrawer(std::function<void(OpenGL&)> call) : call(call) {}
  void glDraw(OpenGL& gl) { call(gl); }
};

struct CstyleInitCall : OpenGL::GLInitCall {
  void* classP;
  void (*call)(void*);
  CstyleInitCall(void (*call)(void*), void* classP): classP(classP), call(call) {}
  bool glInit(OpenGL&) { call(classP); return true; }
};

/// add a draw routine
void OpenGL::add(void (*call)(void*, OpenGL&), void* classP) {
  CHECK(call!=0, "OpenGL: nullptr pointer to drawing routine");
  auto _dataLock = dataLock(RAI_HERE);
  toBeDeletedOnCleanup.append(new CstyleDrawer(call, classP));
  drawers.append(toBeDeletedOnCleanup.last());

}

/// add a draw routine
void OpenGL::addInit(void (*call)(void*), void* classP) {
  CHECK(call!=0, "OpenGL: nullptr pointer to drawing routine");
  auto _dataLock = dataLock(RAI_HERE);
  initCalls.append(new CstyleInitCall(call, classP));

}

void OpenGL::add(std::function<void (OpenGL&)> call) {
  CHECK(call, "OpenGL: nullptr std::function to drawing routine");
  auto _dataLock = dataLock(RAI_HERE);
//  toBeDeletedOnCleanup.append(new CstyleDrawer(call, classP));
  drawers.append(new LambdaDrawer(call));

}

/// add a draw routine to a view
void OpenGL::addSubView(uint v, void (*call)(void*, OpenGL&), void* classP) {
  CHECK(call!=0, "OpenGL: nullptr pointer to drawing routine");
  auto _dataLock = dataLock(RAI_HERE);
  if(v>=views.N) views.resizeCopy(v+1);
  toBeDeletedOnCleanup.append(new CstyleDrawer(call, classP));
  views(v).drawers.append(toBeDeletedOnCleanup.last());

}

void OpenGL::addSubView(uint v, GLDrawer& c) {
  auto _dataLock = dataLock(RAI_HERE);
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).drawers.append(&c);

}

void OpenGL::setSubViewTiles(uint cols, uint rows) {
  for(uint i=0; i<cols*rows; i++) {
    double x=i%cols;
    double y=rows - 1 - i/cols;
    setSubViewPort(i, x/cols, (x+1)/cols, y/rows, (y+1)/rows);
  }
}

void OpenGL::setSubViewPort(uint v, double l, double r, double b, double t) {
  auto _dataLock = dataLock(RAI_HERE);
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).le=l;  views(v).ri=r;  views(v).bo=b;  views(v).to=t;

}

void OpenGL::clearSubView(uint v) {
  if(v>=views.N) return;
  auto _dataLock = dataLock(RAI_HERE);
  views(v).drawers.clear();

}

/// remove a draw routine
//void OpenGL::remove(void (*call)(void*), const void* classP) {
//  CHECK(call!=0, "OpenGL: nullptr pointer to drawing routine");
//  uint i;
//  for(i=0; i<drawers.N; i++) if(drawers(i).call==call && drawers(i).classP==classP) break;
//  CHECK(i<drawers.N, "value to remove not found");
//  drawers.remove(i, 1);
//}

/// clear the list of all draw and callback routines
void OpenGL::clear() {
  auto _dataLock = dataLock(RAI_HERE);
  views.clear();
  listDelete(toBeDeletedOnCleanup);
  drawers.clear();
  initCalls.clear();
  hoverCalls.clear();
  clickCalls.clear();
  keyCalls.clear();

  text.clear();
}

void OpenGL::Draw(int w, int h, rai::Camera* cam, bool callerHasAlreadyLocked) {
  if(rai::getDisableGui()) HALT("you should not be here!");

#ifdef RAI_GL
  if(!callerHasAlreadyLocked) {
    singleGLAccess.getMutex().lock(RAI_HERE);
    dataLock.lock(RAI_HERE); //now accessing user data
  }

  //clear bufferer
  GLint viewport[4] = {0, 0, w, h};
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glClearColor(clearR, clearG, clearB, clearA);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //raster an image as background
  if(background.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glOrtho(0, 1., 1., 0., -1., 1.); //only affects the offset - the rest is done with raster zooms
    glDisable(GL_DEPTH_TEST);
    glRasterImage(0, 0, background, backgroundZoom); //.5*w/background.d1);
  }

  //OpenGL initialization
  glEnable(GL_DEPTH_TEST);  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);  glFrontFace(GL_CCW);
  glShadeModel(GL_FLAT);  //glShadeModel(GL_SMOOTH);

  if(drawOptions.pclPointSize>0.) glPointSize(drawOptions.pclPointSize);

  //select mode?
  GLint mode;
  glGetIntegerv(GL_RENDER_MODE, &mode);

  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mode==GL_SELECT) gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
  if(!cam) camera.glSetProjectionMatrix();
  else     cam->glSetProjectionMatrix();
  //glLineWidth(2);

  //**extract the camera projection matrix
#if 0
  //this is the calibration matrix corresponding to OpenGL's ``viewport''
  intA view(4);
  arr Kview(3, 3);
  glGetIntegerv(GL_VIEWPORT, view.p);
  Kview.setZero();
  Kview(0, 0) = .5*view(2); Kview(0, 2) = view(0)+.5*view(2);
  Kview(1, 1) = .5*view(3); Kview(1, 2) = view(1)+.5*view(3);
  Kview(2, 2) = 1.;  //OpenGL's w coordinate is the negative of what we want...
  //the projection matrix (without viewport-calibration) from OpenGL:
  P.resize(4, 4);
  glGetDoublev(GL_PROJECTION_MATRIX, P.p);
  //cout <<"OpenGL's glP=" <<P <<"\nK=" <<Kview <<endl;
  //double sca=P.elem(0);
  P = ~P;      //OpenGL uses transposed matrix storage convention
  P.delRows(2); //We're not interested in OpenGL's ``z-culling-coordinate'', only in the perspective coordinate (divisor) w
  //P[2]() *=-1.;
  //the full camera projection matrix:
  P = Kview*P;
  //cout <<"OpenGL's P=" <<P <<endl;

  /*
  double zn=camera.zNear, zf=camera.zFar, f=1./tan(RAI_PI/180.*camera.heightAngle/2.);
  arr Frust(4, 4); Frust.setZero();
  Frust(0, 0) = Frust(1, 1) = f;
  Frust(2, 2) = (zf+zn)/(zn-zf);
  Frust(3, 2) = -1.;
  Frust(2, 3) = 2.*zf*zn/(zn-zf);
  cout <<"OpenGL P=" <<P <<"K=" <<Kview <<"znear=" <<camera.zNear <<"zfar=" <<camera.zFar <<endl;
  cout <<"Frust=" <<Frust <<endl;;
  Frust.delRows(2); //We're not interested in OpenGL's ``z-coordinate'', only in the perspective coordinate (divisor) w
  cout <<"K=" <<Kview*Frust <<endl;
  */
#endif

  //draw focus?
  if(drawFocus && mode!=GL_SELECT) {
    glColor(1., .7, .3);
    double size = .005 * (camera.X.pos-camera.foc).length();
    glDrawDiamond(camera.foc.x, camera.foc.y, camera.foc.z, size, size, size);
  }
  /*if(topSelection && mode!=GL_SELECT){
    glColor(1., .7, .3);
    double size = .005 * (camera.X.pos-camera.foc).length();
    glDrawDiamond(topSelection->x, topSelection->y, topSelection->z, size, size, size);
  }*/

  //std color: black:
  glColor(.3, .3, .5);

  //draw central view
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if(mode==GL_SELECT) glInitNames();
  for(uint i=0; i<drawers.N; i++) {
    if(mode==GL_SELECT) glLoadName(i);
//    drawers(i)->glDrawerMutex.lock(RAI_HERE);
    drawers(i)->glDraw(*this);
//    drawers(i)->glDrawerMutex.unlock();
    glLoadIdentity();
  }

  //draw text
  if(text.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glOrtho(0., (double)w, (double)h, .0, -1., 1.);
    glDrawText(text, 10, 20, 0);
    glLoadIdentity();
  }

  //draw subviews
  for(uint v=0; v<views.N; v++) {
    GLView* vi=&views(v);
    glViewport(vi->le*w, vi->bo*h, (vi->ri-vi->le)*w+1, (vi->to-vi->bo)*h+1);
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(vi->img) {
      glDisable(GL_DEPTH_TEST);
      glRasterImage(-1., 1., *vi->img, backgroundZoom*(vi->ri-vi->le)*w/vi->img->d1);
      glEnable(GL_DEPTH_TEST);
    }
    vi->camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if(drawFocus) {
      glColor(1., .7, .3);
      double size = .005 * (camera.X.pos-camera.foc).length();
      glDrawDiamond(vi->camera.foc.x, vi->camera.foc.y, vi->camera.foc.z, size, size, size);
    }
    for(uint i=0; i<vi->drawers.N; i++) vi->drawers(i)->glDraw(*this);
    if(vi->text.N) {
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glOrtho(0., (vi->ri-vi->le)*w, (vi->to-vi->bo)*h, .0, -1., 1.);
      glDrawText(vi->text, 10, 20, 0);
//      glDrawText(vi->text, -.95, .85, 0.);
      glLoadIdentity();
    }
  }

  //cout <<"UNLOCK draw" <<endl;

  captureImage.resize(h, w, 3);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);

  captureDepth.resize(h, w);
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, captureDepth.p);

  //check matrix stack
  GLint s;
  glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &s);
  if(s!=1){
    RAI_MSG("OpenGL name stack has not depth 1 (pushs>pops) in DRAW mode:" <<s);
  }
  //CHECK_LE(s, 1, "OpenGL matrix stack has not depth 1 (pushs>pops)");

  if(!callerHasAlreadyLocked) {
    //now de-accessing user data
    dataLock.unlock();
    singleGLAccess.getMutex().unlock();
  }
#endif
}

void OpenGL::Select(bool callerHasAlreadyLocked) {
  if(reportEvents) { LOG(0) <<RAI_HERE <<" Select entry"; }

  if(!callerHasAlreadyLocked) {
    singleGLAccess.getMutex().lock(RAI_HERE);
    dataLock.lock(RAI_HERE);
  }

#ifdef RAI_GL
  uint i, j, k;

  glSelectBuffer(1000, selectionBuffer);
  glRenderMode(GL_SELECT);

#if 1
  GLint w=width, h=height;

  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mouseView==-1) {
    GLint viewport[4] = {0, 0, w, h};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    camera.glSetProjectionMatrix();
  } else {
    GLView* vi=&views(mouseView);
    GLint viewport[4] = { (GLint)(vi->le*w), (GLint)(vi->bo*h), (GLint)((vi->ri-vi->le)*w), (GLint)((vi->to-vi->bo)*h)};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    vi->camera.glSetProjectionMatrix();
  }

  //draw objects
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glInitNames();
  if(mouseView==-1) {
    for(i=0; i<drawers.N; i++) {
      glLoadName(i);
//      drawers(i)->glDrawerMutex.lock(RAI_HERE);
      drawers(i)->glDraw(*this);
//      drawers(i)->glDrawerMutex.unlock();
      GLint s;
      glGetIntegerv(GL_NAME_STACK_DEPTH, &s);
      if(s!=0) RAI_MSG("OpenGL name stack has not depth 1 (pushs>pops) in SELECT mode:" <<s);
    }
  } else {
    GLView* vi=&views(mouseView);
    for(i=0; i<vi->drawers.N; i++) { glLoadName(i); vi->drawers(i)->glDraw(*this); }
  }

#else
  Draw(width, height, nullptr, true);
#endif
  glLoadIdentity();

  GLint n;
  n=glRenderMode(GL_RENDER);
  selection.resize(n);

  GLuint* obj, maxD=(GLuint)(-1);
  topSelection=nullptr;
  for(j=0, i=0; i<(uint)n; i++) {
    obj=selectionBuffer+j;
    j+=3+obj[0];

    //get name as superposition of all names
    selection(i).name = 0;
    for(k=0; k<obj[0]; k++) selection(i).name |= obj[3+k];

    //get dim and dmax
    selection(i).dmin=(double)obj[1]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmin);
    selection(i).dmax=(double)obj[2]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmax);

    //get top-most selection
    if(!topSelection || selection(i).dmin < topSelection->dmin) topSelection = &selection(i);
  }

  if(topSelection) {
    topSelection->x=0; topSelection->y=0; topSelection->z=topSelection->dmin;
    unproject(topSelection->x, topSelection->y, topSelection->z);
  }

  if(reportSelects) reportSelection();
#endif
  if(!callerHasAlreadyLocked) {

    singleGLAccess.getMutex().unlock();
  }
  if(reportEvents) { LOG(0) <<RAI_HERE <<" Select done"; }
}

/** @brief watch in interactive mode and wait for an exiting event
  (key pressed or right mouse) */
int OpenGL::watch(const char* txt) {
  if(rai::getDisableGui()) return 27; //ESC key
  if(offscreen) {
    LOG(0) <<"can't watch an offscreen context";
    return 'q';
  }
#ifdef RAI_GL
  update((txt?STRING(txt <<" - press ENTER to continue"):0), true);
  if(rai::getInteractivity()) {
    watching.setStatus(1);
    watching.waitForStatusEq(0);
  } else {
    rai::wait(.1);
  }
#endif
  return pressedkey;
}

/// update the view (in Qt: also starts displaying the window)
int OpenGL::update(const char* txt, bool nonThreaded) {
  if(rai::getDisableGui()) return 27; //ESC key
  openWindow();
  if(txt) text.clear() <<txt;
#ifdef RAI_GL
#if RAI_FREEGLUT
  if(nonThreaded) isUpdating.waitForStatusEq(0);
  isUpdating.setStatus(1);
  postRedrawEvent(false);
  if(nonThreaded) isUpdating.waitForStatusEq(0);
#else
  if(nonThreaded || offscreen) {
    beginNonThreadedDraw();
    Draw(width, height);
    endNonThreadedDraw();
  } else {
    postRedrawEvent(false);
  }
#endif
#endif
  return pressedkey;
}

/// waits some msecons before updating
int OpenGL::timedupdate(double sec) {
  static double lasttime=-1;
  double now;
  now=rai::realTime();
  if(lasttime>0. && now-lasttime<sec) rai::wait(lasttime+sec-now);
  lasttime=now;
  return update();
#if 0//def RAI_QTGL
  int i;
  quitLoopOnTimer=true;
  i=startTimer(msec);
  Loop();
  killTimer(i);
  return update();
#endif
}

/// set the four clear colors
void OpenGL::setClearColors(float r, float g, float b, float a) {
  clearR=r; clearG=g; clearB=b; clearA=a;
}

/** @brief inverse projection: given a 2D+depth coordinates in the
  camera view (e.g. as a result of selection) computes the world 3D
  coordinates */
void OpenGL::unproject(double& x, double& y, double& z, bool resetCamera, int subView) {
#ifdef RAI_GL
  double _x, _y, _z;
  arr modelMatrix(4, 4), projMatrix(4, 4);
  intA viewPort(4);
  if(resetCamera) {
    GLint viewport[4] = {0, 0, (GLint)width, (GLint)height};
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  if(subView!=-1) {
    GLView* vi=&views(subView);
    glViewport(vi->le*width, vi->bo*height, (vi->ri-vi->le)*width+1, (vi->to-vi->bo)*height+1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    vi->camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix.p);
  glGetDoublev(GL_PROJECTION_MATRIX, projMatrix.p);
  glGetIntegerv(GL_VIEWPORT, viewPort.p);
//  cout <<"\nM=\n" <<modelMatrix <<"\nP=\n" <<projMatrix <<"\nV=\n" <<viewPort <<endl;
  gluUnProject(x, y, z, modelMatrix.p, projMatrix.p, viewPort.p, &_x, &_y, &_z);
  x=_x; y=_y; z=_z;
#else
  NICO
#endif
}

void OpenGL::project(double& x, double& y, double& z, bool resetCamera, int subView) {
#ifdef RAI_GL
  double _x, _y, _z;
  GLdouble modelMatrix[16], projMatrix[16];
  GLint viewPort[4];
  if(resetCamera) {
    GLint viewport[4] = {0, 0, (GLint)width, (GLint)height};
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  if(subView!=-1) {
    GLView* vi=&views(subView);
    glViewport(vi->le*width, vi->bo*height, (vi->ri-vi->le)*width+1, (vi->to-vi->bo)*height+1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    vi->camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
  glGetIntegerv(GL_VIEWPORT, viewPort);
  gluProject(x, y, z, modelMatrix, projMatrix, viewPort, &_x, &_y, &_z);
  x=_x; y=_y; z=_z;
#else
  NICO
#endif
}

/// print some info on the selection buffer
void OpenGL::reportSelection() {
  uint i;
  std::cout <<"selection report: mouse=" <<mouseposx <<" " <<mouseposy <<" -> #selections=" <<selection.N <<std::endl;
  for(i=0; i<selection.N; i++) {
    if(topSelection == &selection(i)) std::cout <<"  TOP: "; else std::cout <<"       ";
    std::cout
        <<"name = 0x" <<std::hex <<selection(i).name <<std::dec
        <<" min-depth:" <<selection(i).dmin <<" max-depth:" <<selection(i).dmax
        <<" 3D: (" <<selection(i).x <<',' <<selection(i).y <<',' <<selection(i).z <<')'
        <<endl;
  }
}

#ifdef RAI_GL2PS
/** @brief generates a ps from the current OpenGL display, using gl2ps */
void OpenGL::saveEPS(const char* filename) {
  FILE* fp = fopen(filename, "wb");
  GLint buffsize = 0, state = GL2PS_OVERFLOW;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  while(state==GL2PS_OVERFLOW) {
    buffsize+=1024*1024;
    gl2psBeginPage("Marc Toussaint", "MT", viewport,
                   GL2PS_EPS, GL2PS_BSP_SORT, GL2PS_SILENT |
                   GL2PS_SIMPLE_LINE_OFFSET | GL2PS_NO_BLENDING |
                   GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT,
                   GL_RGBA, 0, nullptr, 0, 0, 0, buffsize,
                   fp, filename);
    Draw(width, height);
    state = gl2psEndPage();
  }
  fclose(fp);
}
#else
void OpenGL::saveEPS(const char*) {
  RAI_MSG("WARNING: OpenGL::saveEPS was called without RAI_GL2PS configured!");
}
#endif

#ifndef RAI_QTGL
/** @brief report on the OpenGL capabilities (the QGLFormat) */
void OpenGL::about(std::ostream& os) { RAI_MSG("NICO"); }
#endif

//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) { LOG(0) <<x; }
#elif 1
#  define CALLBACK_DEBUG(x) { cout <<RAI_HERE <<s <<':'; x; }
#else
#  define CALLBACK_DEBUG(x)
#endif

void getSphereVector(rai::Vector& vec, int _x, int _y, int le, int ri, int bo, int to) {
  int w=ri-le, h=to-bo;
  int minwh = w<h?w:h;
  double x, y;
  x=(double)_x;  x=x-le-.5*w;  x*= 2./minwh;
  y=(double)_y;  y=y-bo-.5*h;  y*= 2./minwh;
  vec.x=x;
  vec.y=y;
  vec.z=.5-(x*x+y*y);
  if(vec.z<0.) vec.z=0.;
}

void OpenGL::Reshape(int _width, int _height) {
  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG("Reshape Callback: " <<_width <<' ' <<_height);
  width=_width;
  height=_height;
  if(width%4) width = 4*(width/4);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);
  for(uint v=0; v<views.N; v++) views(v).camera.setWHRatio((views(v).ri-views(v).le)*width/((views(v).to-views(v).bo)*height));

  if(!offscreen) {
    postRedrawEvent(true);
  }
}

void OpenGL::Key(unsigned char key, int mods) {
  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG("Keyboard Callback: " <<key <<"('" <<(char)key <<"')");
  pressedkey = key;
  modifiers = mods;

  bool cont=true;
  for(uint i=0; i<keyCalls.N; i++) cont=cont && keyCalls(i)->keyCallback(*this);

  if(key==13 || key==27 || key=='q' || rai::contains(exitkeys, key)) watching.setStatus(0);

}

void OpenGL::MouseButton(int button, int downPressed, int _x, int _y, int mods) {
  auto _dataLock = dataLock(RAI_HERE);
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG("Mouse Click Callback: " <<button <<' ' <<downPressed <<' ' <<_x <<' ' <<_y);
  mouse_button=1+button;
  if(downPressed) mouse_button=-1-mouse_button;
  mouseposx=_x; mouseposy=_y;
  modifiers = mods;
  lastEvent.set(mouse_button, -1, _x, _y, 0., 0.);

  GLView* v=0;
  rai::Camera* cam=&camera;
  rai::Vector vec;
  for(mouseView=views.N; mouseView--;) {
    v=&views(mouseView);
    if(_x<v->ri*w && _x>v->le*w && _y<v->to*h && _y>v->bo*h) {
      getSphereVector(vec, _x, _y, v->le*w, v->ri*w, v->bo*h, v->to*h);
      cam=&views(mouseView).camera;
      break;
    }
  }
  if(mouseView==-1) {
    getSphereVector(vec, _x, _y, 0, w, 0, h);
    v=0;
  }
  CALLBACK_DEBUG("associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);

  if(!downPressed) {  //down press
    if(mouseIsDown) {  return; } //the button is already down (another button was pressed...)
    //CHECK(!mouseIsDown, "I thought the mouse is up...");
    mouseIsDown=true;
    drawFocus = true;
  } else {
    if(!mouseIsDown) {  return; } //the button is already up (another button was pressed...)
    //CHECK(mouseIsDown, "mouse-up event although the mouse is not down???");
    mouseIsDown=false;
    drawFocus = false;
  }
  //store where you've clicked
  downVec=vec;
  downRot=cam->X.rot;
  downPos=cam->X.pos;
  downFoc=cam->foc;

  //check object clicked on
  if(mouse_button==1 && (mods&2)) {
    drawFocus = false;
    if(!downPressed) {
      drawOptions.drawMode_idColor = true;
      Draw(w, h, nullptr, true);
      double x=mouseposx, y=mouseposy, d = captureDepth(mouseposy, mouseposx);
      if(d<.01 || d==1.) {
        cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<camera.glConvertToTrueDepth(d) <<endl;
      } else {
        unproject(x, y, d, true, mouseView);
      }
      cout <<"SELECTION: ID: " <<color2id(&captureImage(mouseposy, mouseposx, 0))
           <<" point: (" <<x <<' ' <<y <<' ' <<d <<")" <<endl;
    }
  } else {
    drawOptions.drawMode_idColor = false;
  }

  //mouse scroll wheel:
  if(mouse_button==4 && !downPressed) cam->X.pos += downRot*Vector_z * (.1 * (downPos-downFoc).length());
  if(mouse_button==5 && !downPressed) cam->X.pos -= downRot*Vector_z * (.1 * (downPos-downFoc).length());

  if(mouse_button==3) {  //focus on selected point
    double d = captureDepth(mouseposy, mouseposx);
    if(d<.001 || d==1.) {
      cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<camera.glConvertToTrueDepth(d) <<endl;
    } else {
      arr x = {(double)mouseposx, (double)mouseposy, d};
      if(v) {
        x(0) -= double(v->le)*width;
        x(1) -= double(v->bo)*height;
        v->camera.unproject_fromPixelsAndGLDepth(x, (v->ri-v->le)*width, (v->to-v->bo)*height);
        v->camera.focus(x);
      } else {
        cam->unproject_fromPixelsAndGLDepth(x, width, height);
        cam->focus(x);
      }
    }
  }

  //step through all callbacks
  if(!downPressed) {
    for(uint i=0; i<clickCalls.N; i++) clickCalls(i)->clickCallback(*this);
  }

  postRedrawEvent(true);
}

void OpenGL::Scroll(int wheel, int direction) {
  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG("Mouse Wheel Callback: " <<wheel <<' ' <<direction);
  rai::Camera* cam=&camera;
  for(mouseView=views.N; mouseView--;) {
    GLView* v = &views(mouseView);
    if(mouseposx<v->ri*width && mouseposx>v->le*width && mouseposy<v->to*height && mouseposy>v->bo*height) {
      cam=&views(mouseView).camera;
      break;
    }
  }

  if(direction>0) cam->X.pos += cam->X.rot*Vector_z * (.1 * (cam->X.pos-cam->foc).length());
  else            cam->X.pos -= cam->X.rot*Vector_z * (.1 * (cam->X.pos-cam->foc).length());

  postRedrawEvent(true);
}

void OpenGL::WindowStatus(int status) {
  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG("WindowStatus Callback: " <<status);
  if(!status) closeWindow();

}

void OpenGL::MouseMotion(int _x, int _y) {
  auto _dataLock = dataLock(RAI_HERE);
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG("Mouse Motion Callback: " <<_x <<' ' <<_y);
  mouseposx=_x; mouseposy=_y;
  rai::Camera* cam;
  rai::Vector vec;
  if(mouseView==-1) {
    cam=&camera;
    getSphereVector(vec, _x, _y, 0, w, 0, h);
  } else {
    cam=&views(mouseView).camera;
    getSphereVector(vec, _x, _y, views(mouseView).le*w, views(mouseView).ri*w, views(mouseView).bo*h, views(mouseView).to*h);
  }
  CALLBACK_DEBUG("associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);
  lastEvent.set(mouse_button, -1, _x, _y, vec.x-downVec.x, vec.y-downVec.y);
  if(!mouseIsDown) {  //passive motion -> hover callbacks
    mouseposx=_x; mouseposy=_y;
    bool ud=false;
    for(uint i=0; i<hoverCalls.N; i++) ud=ud || hoverCalls(i)->hoverCallback(*this);

    if(ud) postRedrawEvent(true);
    return;
  }
  if(mouse_button==1 && !modifiers) {  //rotation
    rai::Quaternion rot;
    if(downVec.z<.1) {
      //margin:
      rot.setDiff(vec, downVec);  //consider imagined sphere rotation of mouse-move
    } else {
      //inside: use starndard xy to rotate
      rot.setVec(2.*(vec-downVec) ^ Vector_z); //consider only xy-mouse-move
    }
    //rotate about focus
    cam->X.rot = downRot * rot;   //rotate camera's direction
    rot = downRot * rot / downRot; //interpret rotation relative to current viewing
    cam->X.pos = downFoc + rot * (downPos - downFoc);   //rotate camera's position
  }
  if(mouse_button==1 && (modifiers&1) && !(modifiers&2)) {  //translation mouse_button==2){
    rai::Vector trans = vec - downVec;
    trans.z = 0.;
    trans *= .1*(downFoc - downPos).length();
    trans = downRot * trans;
    cam->X.pos = downPos - trans;
  }
  if(mouse_button==3 && !modifiers) {  //zooming || (mouse_button==1 && !(modifiers&GLUT_ACTIVE_SHIFT) && (modifiers&GLUT_ACTIVE_CTRL))){
  }

  postRedrawEvent(true);
}

//===========================================================================
//
// offscreen/background rendering
//

struct XBackgroundContext {
#if defined(RAI_GL) && defined(RAI_X11)
  typedef Bool(*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
  typedef GLXContext(*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

  glXCreateContextAttribsARBProc glXCreateContextAttribsARB;
  glXMakeContextCurrentARBProc glXMakeContextCurrentARB;
  Display* dpy;
  int fbcount;
  GLXFBConfig* fbc;
  GLXContext ctx;
  GLXPbuffer pbuf;

  XBackgroundContext()
    : glXCreateContextAttribsARB(0), glXMakeContextCurrentARB(0) {
    static int visual_attribs[] = { None };
    int context_attribs[] = { GLX_CONTEXT_MAJOR_VERSION_ARB, 3, GLX_CONTEXT_MINOR_VERSION_ARB, 0, None };

    dpy = nullptr; //XOpenDisplay(0);
    fbcount = 0;
    fbc = nullptr;

    /* open display */
    if(!(dpy = XOpenDisplay(0))) HALT("Failed to open display");

    /* get framebuffer configs, any is usable (might want to add proper attribs) */
    if(!(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount)))
      HALT("Failed to get FBConfig");

    /* get the required extensions */
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB((const GLubyte*) "glXCreateContextAttribsARB");
    glXMakeContextCurrentARB = (glXMakeContextCurrentARBProc)glXGetProcAddressARB((const GLubyte*) "glXMakeContextCurrent");
    if(!(glXCreateContextAttribsARB && glXMakeContextCurrentARB)) {
      XFree(fbc);
      HALT("missing support for GLX_ARB_create_context");
    }

    /* create a context using glXCreateContextAttribsARB */
    if(!(ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs))) {
      XFree(fbc);
      HALT("Failed to create opengl context");
    }

    /* create temporary pbuffer */
    int pbuffer_attribs[] = { GLX_PBUFFER_WIDTH, 800, GLX_PBUFFER_HEIGHT, 600, None };
    pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);

    XFree(fbc);
    XSync(dpy, False);

    makeCurrent();
  }

  ~XBackgroundContext() {
    XFree(ctx);
    XFree(dpy);
  }

  void makeCurrent() {
    /* try to make it the current context */
    if(!glXMakeContextCurrent(dpy, pbuf, pbuf, ctx)) {
      /* some drivers do not support context without default framebuffer, so fallback on
             * using the default window.
             */
      if(!glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx)) {
        fprintf(stderr, "failed to make current");
        exit(1);
      }
    }
  }
#endif
};

Singleton<XBackgroundContext> xBackgroundContext;

void OpenGL::renderInBack(int w, int h) {
  beginNonThreadedDraw();

#ifdef RAI_GL
  if(w<0) w=width;
  if(h<0) h=height;

  CHECK_EQ(w%4, 0, "should be devidable by 4!!");

  if(!rboColor || !rboDepth) { //need to initialize
    glewInit();
    glGenRenderbuffers(1, &rboColor);  // Create a new renderbuffer unique name.
    glBindRenderbuffer(GL_RENDERBUFFER, rboColor);  // Set it as the current.
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h); // Sets storage type for currently bound renderbuffer.
    // Depth renderbuffer.
    glGenRenderbuffers(1, &rboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
    // Framebuffer.
    // Create a framebuffer and a renderbuffer object.
    // You need to delete them when program exits.
    glGenFramebuffers(1, &fboId);
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    //from now on, operate on the given framebuffer
    //GL_FRAMEBUFFER        read write
    //GL_READ_FRAMEBUFFER   read
    //GL_FRAMEBUFFER        write

    // Adds color renderbuffer to currently bound framebuffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rboColor);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_RENDERBUFFER, rboDepth);

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    //glReadBuffer(GL_BACK);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE) {
      cout << "framebuffer error:" << endl;
      switch(status) {
        case GL_FRAMEBUFFER_UNDEFINED: {
          cout << "GL_FRAMEBUFFER_UNDEFINED" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << endl;
          break;
        }
        case GL_FRAMEBUFFER_UNSUPPORTED: {
          cout << "GL_FRAMEBUFFER_UNSUPPORTED" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE" << endl;
          break;
        }
        case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: {
          cout << "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS" << endl;
          break;
        }
        case 0: {
          cout << "0" << endl;
          break;
        }
      }
      HALT("couldn't create framebuffer");
    }
  }

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboId);
  Draw(w, h, nullptr, true);
  glFlush();
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
#endif

  endNonThreadedDraw();
}

//===========================================================================
//
// GUI implementation
//

void glUI::addButton(uint x, uint y, const char* name, const char* img1, const char* img2) {
  Button& b = buttons.append();
  byteA img;
  b.hover=false;
  b.x=x; b.y=y; b.name=name;
  //read_png(img, tex1);
  if(img1) {
    read_ppm(img, img1, true);
  } else {
    img.resize(18, strlen(name)*9+10, 3);
    img=255;
  }
  b.w=img.d1; b.h=img.d0;
  b.img1=img;    add_alpha_channel(b.img1, 100);
  if(img2) {
    read_ppm(img, img1, true);
    CHECK(img.d1==b.w && img.d0==b.h, "mismatched size");
  }
  b.img2=img;    add_alpha_channel(b.img2, 200);
}

void glUI::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLint viewPort[4];
  glGetIntegerv(GL_VIEWPORT, viewPort);
  glOrtho(0., viewPort[2], viewPort[3], .0, -1., 1.);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  Button* b;
  float x, y, h;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    //w=b->w;
    h=b->h;
    glColor(0, 0, 0, 1);
    glDrawText(b->name, x+5, y+h-5, 0.);
    if((int)i==top) glRasterImage((int)x, (int)y, b->img2);
    else      glRasterImage((int)x, (int)y, b->img1);
  }
#else
  NICO
#endif
}

bool glUI::checkMouse(int _x, int _y) {
  float x, y, w, h;
  Button* b;
  int otop=top;
  top=-1;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    w=b->w;
    h=b->h;
    if(_x>=x && _x <=x+w && _y>=y && _y<=y+h) top = i;
  }
  if(otop==top) return false;
  //postRedrawEvent(false);
  return true;
}

#ifdef RAI_QTGL
#if   defined RAI_MSVC
#  include"opengl_MSVC.moccpp"
#elif defined RAI_SunOS
#  include"opengl_SunOS.moccpp"
#elif defined RAI_Linux
#  include"opengl_qt_moc.cxx"
#elif defined RAI_Cygwin
#  include"opengl_Cygwin.moccpp"
#endif
#endif

void read_png(byteA& img, const char* file_name, bool swap_rows) {
#ifdef RAI_PNG
  if(access(file_name, F_OK) == -1) {
    HALT("png file '" <<file_name <<"' does not exist");
  }

  FILE* fp = fopen(file_name, "rb");

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  CHECK(png, "");

  png_infop info = png_create_info_struct(png);
  CHECK(info, "");

  if(setjmp(png_jmpbuf(png))) abort();

  png_init_io(png, fp);
  png_read_info(png, info);

  uint width      = png_get_image_width(png, info);
  uint height     = png_get_image_height(png, info);
  png_byte color_type = png_get_color_type(png, info);
  png_byte bit_depth  = png_get_bit_depth(png, info);

  // Read any color_type into 8bit depth, RGBA format.
  // See http://www.libpng.org/pub/png/libpng-manual.txt

  if(bit_depth == 16)
    png_set_strip_16(png);

  if(color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png);

  // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand_gray_1_2_4_to_8(png);

  if(png_get_valid(png, info, PNG_INFO_tRNS))
    png_set_tRNS_to_alpha(png);

  // These color_type don't have an alpha channel then fill it with 0xff.
  if(color_type == PNG_COLOR_TYPE_RGB ||
      color_type == PNG_COLOR_TYPE_GRAY ||
      color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

  if(color_type == PNG_COLOR_TYPE_GRAY ||
      color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_gray_to_rgb(png);

  png_read_update_info(png, info);

  img.resize(height, png_get_rowbytes(png, info));
  rai::Array<byte*> cpointers = img.getCarray();
  //    row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  //    for(int y = 0; y < height; y++) {
  //      row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
  //    }

  png_read_image(png, cpointers.p);

  img.resize(height, width, img.N/(height*width));

  fclose(fp);

  if(swap_rows) flip_image(img);
#else
  LOG(-2) <<"libpng not linked";
#endif
}

