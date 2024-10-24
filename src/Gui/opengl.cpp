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

#define RAI_OPENGL_3

#include "opengl.h"
#include "RenderData.h"
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

//===========================================================================

#ifdef RAI_GLFW

//===========================================================================

struct GlfwSingleton : Thread {
  rai::Array<OpenGL*> glwins;
  Mutex mutex;
  int newWinX=-50, newWinY=50;

  GlfwSingleton() : Thread("GlfwSpinnerSpinner", .01) {
    if(rai::getDisableGui()) { HALT("you must not be here with -disableGui"); }

    glfwSetErrorCallback(error_callback);
    if(!glfwInit()) exit(EXIT_FAILURE);
#ifndef RAI_OPENGL_3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#else
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    int argc=1;
    char* argv[1]= {(char*)"x"};
    glutInit(&argc, argv);

    threadLoop(true);
  }
  ~GlfwSingleton() {
    threadClose();
    glfwTerminate();
  }
  void open() {}
  void step() {
    mutex.lock(RAI_HERE);
    glfwPollEvents();
    for(OpenGL* gl: glwins) if(!gl->offscreen && gl->window && gl->needsRedraw) {
        gl->isUpdating.setStatus(1);

        glfwMakeContextCurrent(gl->window);
        gl->Render(gl->width, gl->height);
        glfwSwapBuffers(gl->window);
        glfwMakeContextCurrent(nullptr);

        gl->needsRedraw=false;
        gl->isUpdating.setStatus(0);
      }
    mutex.unlock();
  }
  void close() {}

  void addGL(OpenGL* gl) {
    mutex.lock(RAI_HERE);
    glwins.append(gl);
#if 1
    gl->needsRedraw = true;
#else
    glfwMakeContextCurrent(gl->window);
    gl->Draw(gl->width, gl->height);
    glfwSwapBuffers(gl->window);
    glfwMakeContextCurrent(nullptr);
#endif
    mutex.unlock();
  }

  void delGL(OpenGL* gl) {
    auto _mux = mutex(RAI_HERE);
    gl->needsRedraw = false;
    glwins.removeValue(gl);
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
//      if(key>0xff) key='%';
      if(key>='A' && key<='Z') key += 'a' - 'A';
      gl->Key(key, mods, true);
    } else if(action==GLFW_RELEASE) {
      if(key==GLFW_KEY_LEFT_CONTROL) { mods &= ~GLFW_MOD_CONTROL; key='%'; }
      if(key==GLFW_KEY_LEFT_SHIFT) { mods &= ~GLFW_MOD_SHIFT; key='%'; }
//      if(key>0xff) key='%';
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

static GlfwSingleton* glfwSingleton() {
  static GlfwSingleton instance;
  return &instance;
}

//===========================================================================

void OpenGL::openWindow() {
  if(rai::getDisableGui()) return;

  if(!window) {
    auto _glfw = glfwSingleton();
    _glfw->mutex.lock(RAI_HERE);

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
      window = glfwCreateWindow(mode->width, mode->height, title.p, monitor, nullptr);
    } else {
      window = glfwCreateWindow(width, height, title.p, nullptr, nullptr);
      if(_glfw->newWinX<0){
        _glfw->newWinX += glfwGetVideoMode( glfwGetPrimaryMonitor() )->width - width;
      }
      glfwSetWindowPos(window, _glfw->newWinX, _glfw->newWinY);
      _glfw->newWinY += height+50;
      if(_glfw->newWinY>1000){ _glfw->newWinY = 0; _glfw->newWinX -= width+20; }
    }
    /*if(!offscreen) */{
      glfwMakeContextCurrent(window);
      glfwSetWindowUserPointer(window, this);
      glfwSetMouseButtonCallback(window, GlfwSingleton::_MouseButton);
      glfwSetCursorPosCallback(window, GlfwSingleton::_MouseMotion);
      glfwSetKeyCallback(window, GlfwSingleton::_Key);
      glfwSetScrollCallback(window, GlfwSingleton::_Scroll);
      glfwSetWindowSizeCallback(window, GlfwSingleton::_Resize);
      glfwSetWindowCloseCallback(window, GlfwSingleton::_Close);
      glfwSetWindowRefreshCallback(window, GlfwSingleton::_Refresh);

      if(noCursor) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//        if (glfwRawMouseMotionSupported()) glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
      }
      //glfwSetWindowAttrib(window, GLFW_FOCUS_ON_SHOW, GL_FALSE);

//      glfwSwapInterval(1);
      glfwMakeContextCurrent(nullptr);
    }
    glfwGetCursorPos(window, &mouseposx, &mouseposy);
    mouseposy = height-mouseposy;

    _glfw->mutex.unlock();

    _glfw->addGL(this);
  } else {
    auto _glfw = glfwSingleton();
    auto lock = _glfw->mutex(RAI_HERE);
    if(!offscreen && !glfwGetWindowAttrib(window, GLFW_VISIBLE)) {
      glfwShowWindow(window);
    }
  }
}

void OpenGL::closeWindow() {
  if(window) {
    auto _glfw = glfwSingleton();
//    isUpdating.setStatus(0);
//    watching.setStatus(0);
    _glfw->delGL(this);
    _glfw->mutex.lock(RAI_HERE);
    glfwGetWindowPos(window, &_glfw->newWinX, &_glfw->newWinY);
    glfwDestroyWindow(window);
    _glfw->mutex.unlock();
  }
}

void OpenGL::raiseWindow() {
  if(window) {
    auto _glfw = glfwSingleton();
    auto lock = _glfw->mutex(RAI_HERE);
    glfwFocusWindow(window);
  }
}

bool OpenGL::hasWindow() {
  return window;
}

void OpenGL::setTitle(const char* _title) {
  if(_title) title = _title;
  if(window) {
    glfwSetWindowTitle(window, title.p);
  }
}

void OpenGL::beginContext(bool fromWithinCallback) {
  if(rai::getDisableGui()) return;
  if(!fromWithinCallback) openWindow();
  auto _glfw = glfwSingleton();
  if(!fromWithinCallback) _glfw->mutex.lock(RAI_HERE);
  glfwMakeContextCurrent(window);
}

void OpenGL::endContext(bool fromWithinCallback) {
  if(rai::getDisableGui()) return;
  auto _glfw = glfwSingleton();
  //glfwSwapBuffers(window);
  glfwMakeContextCurrent(nullptr);
  if(!fromWithinCallback) _glfw->mutex.unlock();
}

void OpenGL::postRedrawEvent(bool fromWithinCallback) {
  auto _glfw = glfwSingleton();
  if(!fromWithinCallback) _glfw->mutex.lock(RAI_HERE);
  if(!needsRedraw) needsRedraw=true;
  if(!fromWithinCallback) _glfw->mutex.unlock();
}

void OpenGL::resize(int w, int h) {
  openWindow();
  {
    auto _glfw = glfwSingleton();
    auto lock = _glfw->mutex(RAI_HERE);
    Reshape(w, h);
    glfwSetWindowSize(window, width, height);
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

int OpenGL::watchImage(const floatA& _img, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, _img);
  float x;
  for(uint i=0; i<img.N; i++) {
    x=_img.p[i];
    img.p[i] = (x<0.)?0:((x>255.)?255:x);
  }
  if(img.nd==2) make_RGB(img);
  return watchImage(img, wait, _zoom);
}

int OpenGL::watchImage(const byteA& img, bool wait, float _zoom) {
  resize(img.d1*_zoom, img.d0*_zoom);
  data().clear();
  data().addStandardScene();
  data().addQuad(img, 0, 0, img.d1*_zoom, -1);
  return update(wait);
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
//  text.clear() <<"displayGrey" <<" max:" <<ma <<"min:" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  return watchImage(img, wait, _zoom);
}

int OpenGL::displayRedBlue(const arr& x, bool wait, float _zoom) {
  double mi=min(x), ma=max(x);
//  text.clear() <<"max=" <<ma <<"min=" <<mi <<endl;
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
  clearColor= {1., 1., 1.};
  if(width%4) width = 4*(width/4);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);
}

OpenGL::~OpenGL() {
  closeWindow();
  clear();
}

void OpenGL::add(rai::RenderData* c) {
//  beginContext();
//  c.glInitialize(*this);
//  endContext();
  {
    auto _dataLock = dataLock(RAI_HERE);
    drawers.append(c);
  }
}

void OpenGL::remove(rai::RenderData* s) {
  {
    auto _dataLock = dataLock(RAI_HERE);
    drawers.removeValue(s);
  }
  beginContext();
  s->glDeinitialize(*this);
  endContext();
}

void OpenGL::addSubView(uint v, rai::RenderData* c) {
  auto _dataLock = dataLock(RAI_HERE);
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).drawers.append(c);
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

rai::RenderData& OpenGL::data(){
  if(!_data){
    _data = std::make_shared<rai::RenderData>();
    add(_data.get());
  }
  return *_data;
}

void OpenGL::clear() {
  auto _dataLock = dataLock(RAI_HERE);
  for (auto& drawer: drawers) {
    drawer->glDeinitialize(*this);
  }
  views.clear();
  drawers.clear();
  hoverCalls.clear();
  clickCalls.clear();
  keyCalls.clear();
}

void OpenGL::Render(int w, int h, rai::Camera* cam, bool callerHasAlreadyLocked) {
  if(rai::getDisableGui()) HALT("you should not be here!");

#ifdef RAI_GL
  if(!callerHasAlreadyLocked) {
    dataLock.lock(RAI_HERE); //now accessing user data
  }

  //clear bufferer
  GLint viewport[4] = {0, 0, w, h};
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glClearColor(clearColor(0), clearColor(1), clearColor(2), 1.);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(drawOptions.pclPointSize>0.) glPointSize(drawOptions.pclPointSize);

  //draw central view
  activeView=0;
  for(rai::RenderData* d:drawers) {
    d->ensureInitialized(*this);
    d->glDraw(*this);
  }

  //draw subviews
  for(uint v=0; v<views.N; v++) {
    activeView=&views(v);
    glViewport(activeView->le*w, activeView->bo*h, (activeView->ri-activeView->le)*w+1, (activeView->to-activeView->bo)*h+1);
    for(rai::RenderData* d:activeView->drawers){
      d->ensureInitialized(*this);
      d->glDraw(*this);
    }
  }
  activeView=0;

  captureImage.resize(h, w, 3);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);

  captureDepth.resize(h, w);
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, captureDepth.p);

  if(!callerHasAlreadyLocked) {
    dataLock.unlock();
  }
#endif
}

/// update the view (in Qt: also starts displaying the window)
int OpenGL::update(bool wait, bool nonThreaded) {
  if(rai::getDisableGui()) return 27; //ESC key
  openWindow();
#ifdef RAI_GL
  if(nonThreaded || offscreen) {
//    HALT("no");
    beginContext();
    Render(width, height);
    glfwSwapBuffers(window);
    endContext();
  } else {
    postRedrawEvent(false);
  }
#endif
  if(wait && rai::getInteractivity()) {
    if(offscreen) {
      LOG(0) <<"can't pause an offscreen context";
    }else{
      watching.setStatus(1);
      watching.waitForStatusEq(0);
    }
  }
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
}

#if 0
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
#endif

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
  beginContext(true);
  Render(width, height, nullptr, true);
  endContext(true);
  drawOptions.drawMode_idColor = false;
  drawOptions.drawColors = true;
  selectID = color2id(&captureImage(mouseposy, mouseposx, 0));
  LOG(1) <<"SELECTION: ID: " <<selectID;
  return selectID;
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

void OpenGL::Key(int key, int mods, bool _keyIsDown) {
//  auto _dataLock = dataLock(RAI_HERE);
  CALLBACK_DEBUG(this, "Keyboard Callback: " <<key <<"('" <<(char)key <<"') mods:" <<mods <<" down:" <<_keyIsDown);
  if(_keyIsDown) pressedkey = key;
  modifiers = mods;
  keyIsDown = _keyIsDown;

  bool cont=true;
  for(uint i=0; i<keyCalls.N; i++) cont=cont && keyCalls(i)->keyCallback(*this);

  if(key==263 && keyIsDown){ scrollCounter++; pressedkey=0; postRedrawEvent(true); }
  if(key==262 && keyIsDown){ scrollCounter--; pressedkey=0; postRedrawEvent(true); }

//  if(key==13 || key==27 || key=='q' || rai::contains(exitkeys, key)) watching.setStatus(0);
  if(keyIsDown && !modifiers && pressedkey && pressedkey!='%') watching.setStatus(0);
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
  } else {
    if(!mouseIsDown) return; //the button is already up (another button was pressed...)
    //CHECK(mouseIsDown, "mouse-up event although the mouse is not down???");
    mouseIsDown=false;
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
    if(!buttonIsUp) {
      drawOptions.drawMode_idColor = true;
      drawOptions.drawColors = false;
      beginContext(true);
      Render(w, h, nullptr, true);
      endContext(true);
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
      selectID = color2id(&captureImage(mouseposy, mouseposx, 0));
      LOG(1) <<"SELECTION: ID: " <<selectID
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
  CALLBACK_DEBUG(this, "Mouse Wheel Callback: " <<wheel <<' ' <<direction <<' ' <<modifiers);

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
//    if(!_SHIFT(modifiers) && _CTRL(modifiers)) {
//      if(direction<0.) cam->focalLength *= 1.1;
//      else cam->focalLength /= 1.1;
//    }
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
    needsUpdate=true;
  }

  //-- shift-LEFT -> translation
  if(mouse_button==1 && (!hideCameraControls && _SHIFT(downModifiers) && !_CTRL(downModifiers)) && !downVec.isZero) {
    rai::Vector diff = vec - downVec;
    diff.z = 0.;
    diff *= .5*(downFoc - downPos).length()/cam->focalLength;
    diff = downRot * diff;
    cam->X.pos = downPos - diff;
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
  bool org_offscreen=offscreen;
  offscreen = true;

  beginContext(fromWithinCallback);

  uint org_width=width, org_height=height;

#ifdef RAI_GL
  if(w>0) width=w;
  if(h>0) height=h;

  Reshape(width, height);

  CHECK_EQ(width%4, 0, "should be devidable by 4!!");

  if(!offscreenFramebuffer) { //need to initialize
    glewInit();
    //create color render buffer
    glGenRenderbuffers(1, &offscreenColor);  // Create a new renderbuffer unique name.
    glBindRenderbuffer(GL_RENDERBUFFER, offscreenColor);  // Set it as the current.
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height); // Sets storage type for currently bound renderbuffer.
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    //create depth render buffer
    glGenRenderbuffers(1, &offscreenDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, offscreenDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    //create framebuffer and attach both renderbuffers
    glGenFramebuffers(1, &offscreenFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, offscreenFramebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, offscreenColor);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_RENDERBUFFER, offscreenDepth);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) HALT("failed");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, offscreenFramebuffer);
  Render(width, height, nullptr, true);
  glFlush();
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
#endif

  Reshape(org_width, org_height);
  offscreen = org_offscreen;

  endContext(fromWithinCallback);
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
rai::Array<rai::RenderData*>::memMove=1;
RUN_ON_INIT_END(opengl)
