/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
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
#include "../Geo/geo.h"

#include <math.h>

#ifdef RAI_GLFW
#  include <GLFW/glfw3.h>
#endif

#ifdef RAI_PNG
#  include <png.h>
#  include <unistd.h>
#endif

#define _SHIFT(mod) (mod&GLFW_MOD_SHIFT)
#define _CTRL(mod) (mod&GLFW_MOD_CONTROL)
#define _NONE(mod) ((!hideCameraControls && !mod) || (hideCameraControls && _SHIFT(mod) && _CTRL(mod)))

#if 1
#  define CALLBACK_DEBUG(gl, x) if(gl->reportEvents) { LOG(0) <<x; }
#elif 1
#  define CALLBACK_DEBUG(gl, x) { cout <<RAI_HERE <<':' <<x <<endl; }
#else
#  define CALLBACK_DEBUG(gl, x)
#endif

OpenGL& NoOpenGL = *((OpenGL*)(nullptr));

OpenGLDrawOptions& GLDrawer::glDrawOptions(OpenGL& gl) { return gl.drawOptions; }

//===========================================================================

struct SingleGLAccess {
};

Singleton<SingleGLAccess> singleGLAccess;

//===========================================================================

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
  int newWinX=-50, newWinY=50;

  GlfwSpinner() : Thread("GlfwSpinnerSpinner", .01) {
    if(rai::getDisableGui()) { HALT("you must not be here with -disableGui"); }

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

  void delLists(OpenGL* gl){
    auto _mux = mutex(RAI_HERE);
    //  LOG(0) <<"# lists before delete:" <<gl->listMap.size();
    glfwMakeContextCurrent(gl->self->window);
    for(auto& entry:gl->listMap) if(entry.second.listId){
      glDeleteLists(entry.second.listId, 1);
    }
    gl->listMap.clear();
    glfwMakeContextCurrent(nullptr);
    //  LOG(0) <<"# lists after delete:" <<gl->listMap.size();
  }

  void delGL(OpenGL* gl) {
//    bool stop=false;
    {
      auto _mux = mutex(RAI_HERE);
      glwins.removeValue(gl);
      //if(!glwins.N) stop=true; //stop looping
      //    if(stop) threadStop(); //stop looping
    }

    delLists(gl);
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
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
//    gl->modifiers=mods;
    CALLBACK_DEBUG(gl, key <<' ' <<action <<' ' <<mods);
    if(action == GLFW_PRESS) {
      if(key==256) key=27;
      if(key==257) key=13;
      if(key==GLFW_KEY_LEFT_CONTROL) { mods |= GLFW_MOD_CONTROL; key='%'; }
      if(key==GLFW_KEY_LEFT_SHIFT) { mods |= GLFW_MOD_SHIFT; key='%'; }
      if(key>0xff) key='%';
      if(key>='A' && key<='Z') key += 'a' - 'A';
      gl->Key(key, mods, true);
    } else if(action==GLFW_RELEASE) {
      if(key==GLFW_KEY_LEFT_CONTROL) { mods &= ~GLFW_MOD_CONTROL; key='%'; }
      if(key==GLFW_KEY_LEFT_SHIFT) { mods &= ~GLFW_MOD_SHIFT; key='%'; }
      if(key>0xff) key='%';
      gl->Key(key, mods, false);
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
    gl->watching.setStatus(0);
//    gl->closeWindow();
  }

  static void _Scroll(GLFWwindow* window, double xoffset, double yoffset) {
    OpenGL* gl=(OpenGL*)glfwGetWindowUserPointer(window);
    gl->Scroll(0, yoffset);
  }

  static void _Refresh(GLFWwindow* window) {
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
    if(fullscreen) {
      GLFWmonitor* monitor = glfwGetPrimaryMonitor();
      const GLFWvidmode* mode = glfwGetVideoMode(monitor);
      //glfwSetWindowMonitor( _wnd, _monitor, 0, 0, mode->width, mode->height, 0 );
      self->window = glfwCreateWindow(mode->width, mode->height, title.p, monitor, nullptr);
    } else {
      self->window = glfwCreateWindow(width, height, title.p, nullptr, nullptr);
      if(fg->newWinX<0){
        fg->newWinX += glfwGetVideoMode( glfwGetPrimaryMonitor() )->width - width;
      }
      glfwSetWindowPos(self->window, fg->newWinX, fg->newWinY);
      fg->newWinY += height+50;
      if(fg->newWinY>1000){ fg->newWinY = 0; fg->newWinX -= width+20; }
    }
    if(!offscreen) {
      glfwMakeContextCurrent(self->window);
      glfwSetWindowUserPointer(self->window, this);
      glfwSetMouseButtonCallback(self->window, GlfwSpinner::_MouseButton);
      glfwSetCursorPosCallback(self->window, GlfwSpinner::_MouseMotion);
      glfwSetKeyCallback(self->window, GlfwSpinner::_Key);
      glfwSetScrollCallback(self->window, GlfwSpinner::_Scroll);
      glfwSetWindowSizeCallback(self->window, GlfwSpinner::_Resize);
      glfwSetWindowCloseCallback(self->window, GlfwSpinner::_Close);
      glfwSetWindowRefreshCallback(self->window, GlfwSpinner::_Refresh);

      if(noCursor) {
        glfwSetInputMode(self->window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//        if (glfwRawMouseMotionSupported()) glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
      }
      //glfwSetWindowAttrib(self->window, GLFW_FOCUS_ON_SHOW, GL_FALSE);

      glfwSwapInterval(1);
      glfwMakeContextCurrent(nullptr);
    }
    glfwGetCursorPos(self->window, &mouseposx, &mouseposy);
    mouseposy = height-mouseposy;

    fg->mutex.unlock();

    fg->addGL(this);
  } else {
    auto fg = singletonGlSpinner();
    auto lock = fg->mutex(RAI_HERE);
    if(!offscreen && !glfwGetWindowAttrib(self->window, GLFW_VISIBLE)) {
      glfwShowWindow(self->window);
    }
  }
}

void OpenGL::closeWindow() {
  self->needsRedraw=0;
  if(self->window) {
    auto fg = singletonGlSpinner();
    isUpdating.setStatus(0);
    watching.setStatus(0);
    fg->delGL(this);
    fg->mutex.lock(RAI_HERE);
    glfwGetWindowPos(self->window, &fg->newWinX, &fg->newWinY);
    glfwDestroyWindow(self->window);
    fg->mutex.unlock();
  }
}

void OpenGL::raiseWindow() {
  if(self->window) {
    auto fg = singletonGlSpinner();
    auto lock = fg->mutex(RAI_HERE);
    glfwFocusWindow(self->window);
  }
}

bool OpenGL::hasWindow() {
  return self->window;
}

void OpenGL::setTitle(const char* _title) {
  if(_title) title = _title;
  if(self->window) {
    glfwSetWindowTitle(self->window, title.p);
  }
}

void OpenGL::beginNonThreadedDraw(bool fromWithinCallback) {
  if(rai::getDisableGui()) return;
  openWindow();
  auto fg = singletonGlSpinner();
  if(!fromWithinCallback) fg->mutex.lock(RAI_HERE);
  glfwMakeContextCurrent(self->window);
}

void OpenGL::endNonThreadedDraw(bool fromWithinCallback) {
  if(rai::getDisableGui()) return;
  auto fg = singletonGlSpinner();
  glfwSwapBuffers(self->window);
  glfwMakeContextCurrent(nullptr);
  if(!fromWithinCallback) fg->mutex.unlock();
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
  {
    auto fg = singletonGlSpinner();
    auto lock = fg->mutex(RAI_HERE);
    glfwSetWindowSize(self->window, width, height);
  }
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
// static objects
//

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
  return arr{rgb(0)/256., rgb(1)/256., rgb(2)/256.};
}

#ifdef RAI_GL
void glStandardLight(void*, OpenGL& gl) {
  glEnable(GL_LIGHTING);
  if(!gl.drawOptions.enableLighting) {
    glDisable(GL_LIGHTING);
    return;
  }
  static GLfloat ambient[]   = { .5, .5, .5, 1.0 };
  static GLfloat diffuse[]   = { .3, .3, .3, 1.0 };
  static GLfloat specular[]  = { .4, .4, .4, 1.0 };
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
  {
    static GLfloat ambient2[]   = { .0, .0, .0, 1.0 };
    static GLfloat diffuse2[]   = { .3, .3, .3, 1.0 };
    static GLfloat specular2[]  = { .4, .4, .4, 1.0 };
    static GLfloat position2[]  = { 0.0, 0.0, -100.0, 1.0 };
    static GLfloat direction2[] = { .0, .0, 1.0 };
    glLightfv(GL_LIGHT1, GL_POSITION, position2);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction2);
    glLighti(GL_LIGHT1, GL_SPOT_CUTOFF,   90);
    glLighti(GL_LIGHT1, GL_SPOT_EXPONENT, 10);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  ambient2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  diffuse2);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular2);
    glEnable(GL_LIGHT1);
  }
}

void glStandardScene(void*, OpenGL& gl) {
  glPushAttrib(GL_CURRENT_BIT);
  glStandardLight(nullptr, gl);
  glDrawFloor(10, .4, .45, .5);
//   glDrawFloor(10, 1.5, 0.83, .0);
//   glDrawFloor(10., 108./255., 123./255., 139./255.);
  if(!gl.drawOptions.drawVisualsOnly) {
    glDrawAxes(.1);
  }
  glPopAttrib();
  glColor(.8, .8, .8);
}

void glStandardOriginAxes(void*, OpenGL&) {
  glDrawAxes(.1, true);
}

void glColor(int col) {
  static const GLfloat colorsTab[6][4] = {
    {0.2, 0.2, 1.0, 1.0}, // blue
    {1.0, 0.8, 0.0, 1.0}, // gold
    {1.0, 0.0, 0.0, 1.0}, // red
    {0.7, 0.7, 0.7, 1.0}, // gray
    {1.0, 1.0, 1.0, 1.0}, // white
    {0.2, 1.0, 0.2, 1.0}  // green
  };

  col = col%6; //if(col<0) col=0; if(col>5) col=5;
  glColor(colorsTab[col][0], colorsTab[col][1], colorsTab[col][2], colorsTab[col][3]);
}

void glColor(float r, float g, float b, float alpha, GLboolean lightingEnabled) {
  if(lightingEnabled>1) {
    glGetBooleanv(GL_LIGHTING, &lightingEnabled);
  }
  if(lightingEnabled==1) {
    float diff=1.f;
    GLfloat diffuse[4]  = { r*diff, g*diff, b*diff, alpha };
#if 1
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
#else
    float amb=1.f, spec=.25f;
    GLfloat ambient[4]  = { r*amb, g*amb, b*amb, alpha };
    GLfloat specular[4] = { spec* (1.f+r), spec* (1.f+g), spec* (1.f+b), alpha };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0f);
#endif
  } else {
    glColor4f(r, g, b, alpha);
  }
}

void glColor(float* rgb) { glColor(rgb[0], rgb[1], rgb[2], 1.); }

void glColor(const arr& col) {
  if(col.N==1) glColor(col.p[0], col.p[0], col.p[0], 1.);
  if(col.N==3) glColor(col.p[0], col.p[1], col.p[2], 1.);
  if(col.N==4) glColor(col.p[0], col.p[1], col.p[2], col.p[3]);
}

void glColorId(uint id) {
  byte rgb[3];
  glDisable(GL_LIGHTING);
  id2color(rgb, id);
  glColor3ubv(rgb);
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
        if(largeFont) y+=4;
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

#if 1
  glColor(r+.15f, g+.15f, b+.15f);
  float eps=.002;
  for(int i=-5; i<=5; i++) {
    glBegin(GL_LINES);
    glVertex3f(i*x/5., -x, eps);
    glVertex3f(i*x/5., x, eps);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, i*x/5., eps);
    glVertex3f(x, i*x/5., eps);
    glEnd();
  }

//  glColor(.25, .25, .25);
//  glBegin(GL_LINE_STRIP);
//  glVertex3f(-x, -x, eps);
//  glVertex3f(-x, x, eps);
//  glVertex3f(x, x, eps);
//  glVertex3f(x, -x, eps);
//  glVertex3f(-x, -x, eps);
//  glEnd();
#endif

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
  glDisable(GL_CULL_FACE);
  if(scale>=0) glPushMatrix();
  if(scale>=0) glScalef(scale, scale, scale);
  GLUquadric* style=gluNewQuadric();
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(.95, 0, 0);
  glEnd();
  glTranslatef(.9, 0, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(style, .04, 0, .1, 16, 1);
  gluDeleteQuadric(style);
  if(scale>=0) glPopMatrix();
  glEnable(GL_CULL_FACE);
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
  glDrawAxes(.1);

  double dxFar, dyFar, zFar;
  double dxNear, dyNear, zNear;
  zNear = cam.zNear;
  zFar = cam.zFar;
  if(zFar-zNear > 1.) zFar = 0.; //zNear + .1;
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
  glVertex3f(-dxNear, -dyNear, zNear);
  glVertex3f(-dxNear, dyNear, zNear);
  glVertex3f(dxNear, dyNear, zNear);
  glVertex3f(dxNear, -dyNear, zNear);
  glVertex3f(-dxNear, -dyNear, zNear);
  glEnd();
  if(zFar) {
    glBegin(GL_LINE_STRIP);
    glVertex3f(-dxFar, -dyFar, zFar);
    glVertex3f(-dxFar, dyFar, zFar);
    glVertex3f(dxFar, dyFar, zFar);
    glVertex3f(dxFar, -dyFar, zFar);
    glVertex3f(-dxFar, -dyFar, zFar);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxFar, -dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxFar, dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(dxFar, -dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(dxFar, dyFar, zFar);
    glEnd();
    glEnd();
  } else {
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxNear, -dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxNear, dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(dxNear, -dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(dxNear, dyNear, zNear);
    glEnd();
  }
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
    img.insColumns(-1, add*P);
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

void glDrawAsList(GLDrawer& drawer, OpenGL& gl) {
  OpenGL::ListIdVersion& entry = gl.listMap[&drawer];

  if(!entry.listId) { //first time -> allocate list
    entry.listId = glGenLists(1);
    CHECK_GE(entry.listId, 1, "I expected id>=1");
    entry.version = -1;
  }

  if(entry.version < drawer.version) { //needs rebuilding
    glNewList(entry.listId, GL_COMPILE_AND_EXECUTE);
    drawer.glDraw(gl);
    glEndList();
    entry.version = drawer.version;
  } else {
    glCallList(entry.listId);
  }
}

void glClearList(GLDrawer& drawer, OpenGL& gl){
  auto entry = gl.listMap.find(&drawer);

  if(entry!=gl.listMap.end()){
    glDeleteLists(entry->second.listId, 1);
    gl.listMap.erase(entry);
  }
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
  if(!self->window) resize(_img.d1*_zoom, _img.d0*_zoom);
  background=_img;
  backgroundZoom=_zoom;
  //resize(img->d1*zoom,img->d0*zoom);
  if(wait) return watch();
  return update();
}

/*void glWatchImage(const floatA &x, bool wait, float zoom){
  double ma=max(x);
  double mi=min(x);
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

int OpenGL::displayGrey(const arr& x, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, x);
  double mi=min(x), ma=max(x);
  text.clear() <<"displayGrey" <<" max:" <<ma <<"min:" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  return watchImage(img, wait, _zoom);
}

int OpenGL::displayRedBlue(const arr& x, bool wait, float _zoom) {
  double mi=min(x), ma=max(x);
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
#endif


//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* _title, int w, int h, bool _offscreen, bool _fullscreen, bool _hideCameraControls, bool _noCursor)
  : title(_title), width(w), height(h), offscreen(_offscreen),
    fullscreen(_fullscreen), hideCameraControls(_hideCameraControls), noCursor(_noCursor) {
  //RAI_MSG("creating OpenGL=" <<this);
  self = make_unique<sOpenGL>(this); //this might call some callbacks (Reshape/Draw) already!
  clearColor= {1., 1., 1.};
  if(width%4) width = 4*(width/4);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);
}

OpenGL::~OpenGL() {
  clear();
  closeWindow();
}

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

/// add a draw routine
void OpenGL::add(void (*call)(void*, OpenGL&), void* classP) {
  CHECK(call!=0, "OpenGL: nullptr pointer to drawing routine");
  auto _dataLock = dataLock(RAI_HERE);
  toBeDeletedOnCleanup.append(new CstyleDrawer(call, classP));
  drawers.append(toBeDeletedOnCleanup.last());
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

void OpenGL::clearLists(){
  auto fg = singletonGlSpinner();
  fg->delLists(this);
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
  background.clear();
  views.clear();
  for(CstyleDrawer* d:toBeDeletedOnCleanup) delete d;
  toBeDeletedOnCleanup.clear();
  drawers.clear();
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
  glClearColor(clearColor(0), clearColor(1), clearColor(2), 1.);
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
    glColor(1., 1., .0);
    double size = .02 * (camera.X.pos-camera.foc).length()/camera.focalLength;
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
    if(clearColor(0)+clearColor(1)+clearColor(2)>1.) glColor3d(0., 0., 0.); else glColor3d(1., 1., 1.);
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
      if(clearColor(0)+clearColor(1)+clearColor(2)>1.) glColor3d(0., 0., 0.); else glColor3d(1., 1., 1.);
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
  if(s!=1) {
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
  if(txt) text.clear() <<txt;
  rai::String textCopy = text;
  text <<" - press ENTER to continue";
  update(0, true);
  if(rai::getInteractivity()) {
    watching.setStatus(1);
    watching.waitForStatusEq(0);
  } else {
    rai::wait(.1);
  }
  text = textCopy;
#endif
  return pressedkey;
}

/// update the view (in Qt: also starts displaying the window)
int OpenGL::update(const char* txt, bool nonThreaded) {
  if(rai::getDisableGui()) return 27; //ESC key
  openWindow();
  if(txt) text.clear() <<txt;
#ifdef RAI_GL
  if(nonThreaded || offscreen) {
    beginNonThreadedDraw();
    Draw(width, height);
    endNonThreadedDraw();
  } else {
    postRedrawEvent(false);
  }
#endif
  int key=pressedkey;
//  pressedkey=0;
//  if(key) LOG(0) <<"KEY! " <<key;
  return key;
}

/// waits some msecons before updating
int OpenGL::timedupdate(double sec) {
  static double lasttime=-1;
  double now;
  now=rai::realTime();
  if(lasttime>0. && now-lasttime<sec) rai::wait(lasttime+sec-now);
  lasttime=now;
  return update();
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

//===========================================================================
//
// callbacks
//

void getSphereVector(rai::Vector& vec, double x, double y, int le, int ri, int bo, int to) {
  int w=ri-le, h=to-bo;
  int minwh = w<h?w:h;
  x=x-le-.5*w;  x*= 2./minwh;
  y=y-bo-.5*h;  y*= -2./minwh;
  vec.set(x, y, .5-(x*x+y*y));
  if(vec.z<0.) vec.z=0.;
  vec.isZero=false;
}

bool OpenGL::modifiersNone() { return _NONE(modifiers); }
bool OpenGL::modifiersShift() { return _SHIFT(modifiers); }
bool OpenGL::modifiersCtrl() { return _CTRL(modifiers); }

arr OpenGL::get3dMousePos(arr& normal) {
  double d = 0;
  if(mouseposy>=0. && mouseposy<=height-1 && mouseposx>=0. && mouseposx<=width-1)
    d = captureDepth(mouseposy, mouseposx);
  arr x = {mouseposx, mouseposy, d};
  if(d<.01 || d==1.) {
    cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<camera.glConvertToTrueDepth(d) <<endl;
  } else {
    camera.unproject_fromPixelsAndGLDepth(x, width, height);
  }
  if(!!normal) {
    arr x1 = {mouseposx-1., mouseposy, captureDepth(mouseposy, mouseposx-1.)};
    camera.unproject_fromPixelsAndGLDepth(x1, width, height);
    arr x2 = {mouseposx+1., mouseposy, captureDepth(mouseposy, mouseposx+1.)};
    camera.unproject_fromPixelsAndGLDepth(x2, width, height);
    arr y1 = {mouseposx, mouseposy-1., captureDepth(mouseposy-1., mouseposx)};
    camera.unproject_fromPixelsAndGLDepth(y1, width, height);
    arr y2 = {mouseposx, mouseposy+1., captureDepth(mouseposy+1., mouseposx)};
    camera.unproject_fromPixelsAndGLDepth(y2, width, height);

    normal = crossProduct(x2-x1, y2-y1);
    normal /= length(normal);
  }
  return x;
}

uint OpenGL::get3dMouseObjID() {
  drawOptions.drawMode_idColor = true;
  drawOptions.drawColors = false;
  beginNonThreadedDraw(true);
  Draw(width, height, nullptr, true);
  endNonThreadedDraw(true);
  drawOptions.drawMode_idColor = false;
  drawOptions.drawColors = true;
  uint id = color2id(&captureImage(mouseposy, mouseposx, 0));
  LOG(1) <<"SELECTION: ID: " <<id;
  return id;
}

void OpenGL::Reshape(int _width, int _height) {
  //  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG(this, "Reshape Callback: " <<_width <<' ' <<_height);
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

void OpenGL::Key(unsigned char key, int mods, bool _keyIsDown) {
//  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG(this, "Keyboard Callback: " <<key <<"('" <<(char)key <<"') mods:" <<mods <<" down:" <<_keyIsDown);
  if(_keyIsDown) pressedkey = key;
  modifiers = mods;
  keyIsDown = _keyIsDown;

  bool cont=true;
  for(uint i=0; i<keyCalls.N; i++) cont=cont && keyCalls(i)->keyCallback(*this);

//  if(key==13 || key==27 || key=='q' || rai::contains(exitkeys, key)) watching.setStatus(0);
  if(_keyIsDown && !mods && key!='%') watching.setStatus(0);
}

void OpenGL::MouseButton(int button, int buttonIsUp, int _x, int _y, int mods) {
//  auto _dataLock = dataLock(RAI_HERE);
  int w=width, h=height;
  bool needsUpdate=false;
  _y = h-_y;
  CALLBACK_DEBUG(this, "Mouse Click Callback: " <<button <<' ' <<_x <<' ' <<_y <<" up:" <<buttonIsUp <<" mods:" <<mods);
  mouse_button=1+button;
  if(buttonIsUp) mouse_button=-1-mouse_button;
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
  CALLBACK_DEBUG(this, "associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);

  if(!buttonIsUp) {  //down press
    if(mouseIsDown) { return; } //the button is already down (another button was pressed...)
    //CHECK(!mouseIsDown, "I thought the mouse is up...");
    mouseIsDown=true;
    if(_NONE(modifiers)) drawFocus = true;
  } else {
    if(!mouseIsDown) return; //the button is already up (another button was pressed...)
    //CHECK(mouseIsDown, "mouse-up event although the mouse is not down???");
    mouseIsDown=false;
    drawFocus = false;
    needsUpdate=true;
  }
  //store where you've clicked
  downVec=vec;
  downRot=cam->X.rot;
  downPos=cam->X.pos;
  downFoc=cam->foc;
  downModifiers = modifiers;

  //-- shift-ctrl-LEFT -> check object clicked on
  if(mouse_button==1 && !hideCameraControls && _SHIFT(modifiers) && _CTRL(modifiers)) {
    drawFocus = false;
    if(!buttonIsUp) {
      drawOptions.drawMode_idColor = true;
      drawOptions.drawColors = false;
      beginNonThreadedDraw(true);
      Draw(w, h, nullptr, true);
      endNonThreadedDraw(true);
      double d = 0.;
      if(mouseposy>=0. && mouseposy<=height-1 && mouseposx>=0. && mouseposx<=width-1)
        d = captureDepth(mouseposy, mouseposx);
      arr x = {double(mouseposx), double(mouseposy), d};
//      cout <<" image coords: " <<x;
      if(d<.01 || d==1.) {
        cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<camera.glConvertToTrueDepth(d) <<endl;
      } else {
        camera.unproject_fromPixelsAndGLDepth(x, width, height);
      }
      LOG(1) <<"SELECTION: ID: " <<color2id(&captureImage(mouseposy, mouseposx, 0))
             <<" world coords: " <<x;
    }
  } else {
    drawOptions.drawMode_idColor = false;
    drawOptions.drawColors = true;
  }

  //mouse scroll wheel:
  if(mouse_button==4 && !hideCameraControls && !buttonIsUp) cam->X.pos += downRot*Vector_z * (.1 * (downPos-downFoc).length());
  if(mouse_button==5 && !hideCameraControls && !buttonIsUp) cam->X.pos -= downRot*Vector_z * (.1 * (downPos-downFoc).length());

  //-- RIGHT -> focus on selected point
  if(mouse_button==3 && (_NONE(modifiers))) {
    double d = 0.;
    if(mouseposy>=0. && mouseposy<=height-1 && mouseposx>=0. && mouseposx<=width-1)
      d = captureDepth(mouseposy, mouseposx);
    if(d<.001 || d==1.) {
      cout <<"NO SELECTION: SELECTION DEPTH = " <<d <<' ' <<camera.glConvertToTrueDepth(d) <<endl;
    } else {
      arr x = {(double)mouseposx, (double)mouseposy, d};
      if(v) {
        x(0) -= double(v->le)*width;
        x(1) -= double(v->bo)*height;
        v->camera.unproject_fromPixelsAndGLDepth(x, (v->ri-v->le)*width, (v->to-v->bo)*height);
        v->camera.focus(x(0), x(1), x(2));
      } else {
        cam->unproject_fromPixelsAndGLDepth(x, width, height);
        cam->focus(x(0), x(1), x(2));
      }
      //LOG(1) <<"FOCUS: world coords: " <<x;
    }
    needsUpdate=true;
  }

  //step through all callbacks
  for(uint i=0; i<clickCalls.N; i++) needsUpdate = needsUpdate || clickCalls(i)->clickCallback(*this);

  if(needsUpdate) postRedrawEvent(true);
}

void OpenGL::Scroll(int wheel, int direction) {
  CALLBACK_DEBUG(this, "Mouse Wheel Callback: " <<wheel <<' ' <<direction);

  rai::Camera* cam=&camera;
  for(mouseView=views.N; mouseView--;) {
    GLView* v = &views(mouseView);
    if(mouseposx<v->ri*width && mouseposx>v->le*width && mouseposy<v->to*height && mouseposy>v->bo*height) {
      cam=&views(mouseView).camera;
      break;
    }
  }

  //-- user provided callbacks first
  bool cont=true;
  for(uint i=0; cont && i<scrollCalls.N; i++) cont = cont && scrollCalls(i)->scrollCallback(*this, direction);

  if(cont) {
    double dz = (direction>0? -.1:.1);

    //-- SCROLL -> zoom
    if(_NONE(modifiers)) {
      cam->X.pos += cam->X.rot.getZ() * (dz * (cam->X.pos-cam->foc).length());
    }

    //-- shift -> translation
    if(_SHIFT(modifiers) && !_CTRL(modifiers)) {
#if 0
      cam->X.pos += cam->X.rot.getZ() * (dz * (cam->X.pos-cam->foc).length());
      cam->foc += cam->X.rot.getZ() * (dz * (cam->X.pos-cam->foc).length());
#else
      if(direction>0) scrollCounter++;
      else scrollCounter--;
#endif
    }

    //-- ctrl -> focal length
    if(!_SHIFT(modifiers) && _CTRL(modifiers)) {
      if(direction<0.) cam->focalLength *= 1.1;
      else cam->focalLength /= 1.1;
    }
  }

  postRedrawEvent(true);
}

void OpenGL::WindowStatus(int status) {
//  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG(this, "WindowStatus Callback: " <<status);
  if(!status) closeWindow();
}

void OpenGL::MouseMotion(double _x, double _y) {
//  auto _dataLock = dataLock(RAI_HERE);
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG(this, "Mouse Motion Callback: " <<_x <<' ' <<_y);
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
  CALLBACK_DEBUG(this, "associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);
  lastEvent.set(mouse_button, -1, _x, _y, vec.x-downVec.x, vec.y-downVec.y);

  bool needsUpdate=false;

  //-- LEFT -> rotation
  if(mouse_button==1 && _NONE(downModifiers) && !downVec.isZero) {
    rai::Quaternion rot;
    if(downVec.z<.01) {
      //at the margin:
      downVec.z=0; downVec.normalize();
      vec.z = 0.; vec.normalize();
      rot.setDiff(vec, downVec);  //consider imagined sphere rotation of mouse-move
    } else {
      //not at margin: use starndard xy to rotate
      rai::Vector diff = vec - downVec;
      diff.set(-diff.y, diff.x, 0); //no rotation about
      rot.setVec(3.*diff); //consider only xy-mouse-move
    }
    //rotate about focus
    cam->X.rot = downRot * rot;   //rotate camera's direction
    rot = downRot * rot / downRot; //interpret rotation relative to current viewing
    cam->X.pos = downFoc + rot * (downPos - downFoc);   //rotate camera's position
    drawFocus=true;
    needsUpdate=true;
  }

  //-- shift-LEFT -> translation
  if(mouse_button==1 && (!hideCameraControls && _SHIFT(downModifiers) && !_CTRL(downModifiers)) && !downVec.isZero) {
    rai::Vector diff = vec - downVec;
    diff.z = 0.;
    diff *= .5*(downFoc - downPos).length()/cam->focalLength;
    diff = downRot * diff;
    cam->X.pos = downPos - diff;
    drawFocus=true;
    needsUpdate=true;
  }

  //step through all callbacks
  for(uint i=0; i<hoverCalls.N; i++) needsUpdate = needsUpdate || hoverCalls(i)->hoverCallback(*this);

  if(needsUpdate) postRedrawEvent(true);
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

void OpenGL::renderInBack(int w, int h, bool fromWithinCallback) {
  beginNonThreadedDraw(fromWithinCallback);

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

  endNonThreadedDraw(fromWithinCallback);
}

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
  rai::Array<byte*> cpointers = getCarray(img);
  if(swap_rows) cpointers.reverse();

  png_read_image(png, cpointers.p);

  img.resize(height, width, img.N/(height*width));

  fclose(fp);
#else
  THROW("libpng not linked")
#endif
}

void write_png(const byteA& img, const char* file_name, bool swap_rows) {
#ifdef RAI_PNG
  FILE* fp = fopen(file_name, "wb");
  if(!fp) abort();

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if(!png) abort();

  png_infop info = png_create_info_struct(png);
  if(!info) abort();

  if(setjmp(png_jmpbuf(png))) abort();

  png_init_io(png, fp);

  // Output is 8bit depth, RGBA format.
  png_set_IHDR(
    png,
    info,
    img.d1, img.d0,
    8,
    img.d2==4?PNG_COLOR_TYPE_RGBA:PNG_COLOR_TYPE_RGB,
    PNG_INTERLACE_NONE,
    PNG_COMPRESSION_TYPE_DEFAULT,
    PNG_FILTER_TYPE_DEFAULT
  );
  png_write_info(png, info);

  // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
  // Use png_set_filler().
  //png_set_filler(png, 0, PNG_FILLER_AFTER);

  byteA imgRef = img.ref();
  imgRef.reshape(img.d0, -1);
  rai::Array<byte*> cpointers = getCarray(imgRef);
  if(swap_rows) cpointers.reverse();

  png_write_image(png, cpointers.p);
  png_write_end(png, NULL);

  fclose(fp);

  png_destroy_write_struct(&png, &info);
#else
  THROW("libpng not linked")
#endif

}
RUN_ON_INIT_BEGIN(opengl)
rai::Array<GLDrawer*>::memMove=1;
RUN_ON_INIT_END(opengl)
