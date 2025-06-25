/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/thread.h"
#include "../Geo/geo.h"

#include <functional>
#include <map>

#ifdef RAI_GL
#  include <GL/glew.h>
#  include <GL/gl.h>
#  include <GL/glu.h>
#  ifdef RAI_CUDA
#    undef APIENTRY
#  endif
#  include <GL/glut.h>
#endif

#undef Success

typedef struct GLFWwindow GLFWwindow;

namespace rai {
struct Vector;
struct Quaternion;
struct Transformation;
struct RenderData;
}

//===========================================================================
//
// utility functions
//

byteA id2color_b(uint id);
arr id2color(uint id);
uint color2id(::byte rgb[3]);

//void glStandardLight(void*, OpenGL& gl);
//void glStandardScene(void*, OpenGL&);
//void glStandardOriginAxes(void*, OpenGL&);

//void glColor(float r, float g, float b, float a=1.f, unsigned char lightingEnabled=2);
//void glColor(int col);
//void glColor(const arr& col);
//void glColorId(uint id);
//void glDrawText(const char* txt, float x=0., float y=0., float z=0., bool largeFont=false);
////void glShadowTransform();
//void glTransform(const rai::Transformation& t);
//void glTransform(const double pos[3], const double R[12]);
//void glRotate(const rai::Quaternion& rot);
//void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
//                float x3, float y3, float z3, float x4, float y4, float z4,
//                float r, float g, float b);
//void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
//                float x3, float y3, float z3, float x4, float y4, float z4);
//void glDrawRect(float x, float y, float z, float rad);
//void glDrawPolygon(const arr& P);
//void glDrawFloor(float x, float r, float g, float b);
//void glDrawBox(float x, float y, float z, bool linesOnly=false);
//void glDrawDiamond(float dx, float dy, float dz);
//void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz);
//void glDrawSphere(float radius);
//void glDrawDisk(float radius);
//void glDrawProxy(const arr& p1, const arr& p2, double diskSize=.02, int colorCode=0, const arr& norm=NoArr, double _rad1=0., double _rad2=0.);
//void glDrawCylinder(float radius, float length, bool closed=true);
//void glDrawCappedCylinder(float radius, float length);
//void glDrawAxis(double scale=-1.);
//void glDrawAxes(double scale, bool colored=true);
//void glDrawCamera(const rai::Camera& cam);
//void glDrawGridBox(float x);
//void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2);
//void glDrawKhepera();
//void glMakeSquare(int num);
//void glMakeStdSimplex(int num);
//void glMakeTorus(int num);
//void glDrawRobotArm(float a, float b, float c, float d, float e, float f);
//uint glImageTexture(const byteA& img);
//void glDrawTexQuad(const byteA& img,
//                   float x1, float y1, float z1, float x2, float y2, float z2,
//                   float x3, float y3, float z3, float x4, float y4, float z4,
//                   float mulX=1., float mulY=1.);
////grabImage: use OpenGL::capture instead!
//void glRasterImage(float x, float y, byteA& img, float zoom=1.);

void read_png(byteA& img, const char* file_name, bool swap_rows);
void write_png(const byteA& img, const char* file_name, bool swap_rows=true);

//===========================================================================
//
// OpenGL class
//

/** @brief A class to display and control 3D scenes using OpenGL and Qt.

    Minimal use: call \ref add to add routines or objects to be drawn
    and \ref update or \ref watch to start the display. */
struct OpenGL {
  /// @name little structs to store objects and callbacks
  struct GLHoverCall { virtual bool hoverCallback(OpenGL&) = 0; };
  struct GLClickCall { virtual bool clickCallback(OpenGL&, int button, int buttonIsDown) = 0; };
  struct GLKeyCall  { virtual bool keyCallback(OpenGL&, int key, int mods, bool keyIsDown) = 0; };
  struct GLScrollCall { virtual bool scrollCallback(OpenGL&, int) = 0; };
  struct GLEvent    { int button, key, x, y; float dx, dy; void set(int b, int k, int _x, int _y, float _dx, float _dy) { button=b; key=k; x=_x; y=_y; dx=_dx; dy=_dy; } };
  struct GLView     { double le, ri, bo, to;  rai::Array<rai::RenderData*> drawers;  rai::Camera camera;  GLView() { le=bo=0.; ri=to=1.; }  str text; };

  /// @name data fields
  rai::Array<GLView> views;            ///< list of subviews
  rai::Array<rai::RenderData*> drawers;
  rai::Array<GLHoverCall*> hoverCalls; ///< list of hover callbacks
  rai::Array<GLClickCall*> clickCalls; ///< list of click callbacks
  rai::Array<GLKeyCall*> keyCalls;     ///< list of click callbacks
  rai::Array<GLScrollCall*> scrollCalls;     ///< list of click callbacks

  rai::String title;     ///< the window title
  uint width, height;
  str text;
  bool offscreen;
  rai::Camera camera;     ///< the camera used for projection
  floatA clearColor;  ///< colors of the beackground (called in glClearColor(...))
  bool reportEvents=false;    ///< flags for verbosity
  int pressedkey=0;         ///< stores the key pressed
  bool keyIsDown=false;
  int modifiers=0;          ///< stores modifier keys
  int mouse_button=0;       ///< stores which button was pressed
  double mouseposx=0, mouseposy=0;  ///< current x- and y-position of mouse
  int mouseView=-1;
  GLView *activeView=0;
  bool mouseIsDown=false;
  int scrollCounter=0;
  byteA captureImage;
  floatA captureDepth;
  rai::Mutex dataLock; //'data' means anything: member fields (camera, variables), drawers, data the drawers access
//  uint fbo, render_buf;
  uint offscreenFramebuffer=0;
  uint offscreenColor=0;
  uint offscreenDepth=0;
  Signaler isUpdating;
  Signaler watching;
  rai::OpenGLDrawOptions drawOptions;
  int selectID=-1;
  std::shared_ptr<rai::RenderData> _data;

  bool fullscreen=false; ///<window starts in fullscreenmode on the primary screen
  bool hideCameraControls=false; ///<camera can be tilted, rotated, zoomed in/out if controls are enabled
  bool noCursor=false;

  /// @name constructors & destructors
  OpenGL(const char* title="rai::OpenGL", int w=400, int h=400, bool _offscreen=false, bool _fullscreen=false, bool _hideCameraControls=false, bool _noCursor=false);
  //OpenGL(void *parent, const char* title="rai::OpenGL", int w=400, int h=400, int posx=-1, int posy=-1);

  ~OpenGL();

  /// @name adding drawing routines and callbacks
  void clear();
  void add(rai::RenderData* c);
  void remove(rai::RenderData* s);
  void addHoverCall(GLHoverCall* c) { hoverCalls.append(c); }
  void addClickCall(GLClickCall* c) { clickCalls.append(c); }
  void addKeyCall(GLKeyCall* c) { keyCalls.append(c); }
  void addSubView(uint view, rai::RenderData* c);
  void setSubViewTiles(uint cols, uint rows);
  void setSubViewPort(uint view, double l, double r, double b, double t);
  void clearSubView(uint view);
  rai::RenderData& data();

  /// @name the core draw routines (actually only for internal use)
  void Render(int w, int h, rai::Camera* cam=nullptr, bool callerHasAlreadyLocked=false);
  void renderInBack(int w=-1, int h=-1, bool fromWithinCallback=false);

  /// @name showing, updating, and watching
  int update(bool wait=false, bool nonThreaded=false);
  int timedupdate(double sec);
  void resize(int w, int h);
//  void unproject(double& x, double& y, double& z, bool resetCamera=false, int subView=-1);
//  void project(double& x, double& y, double& z, bool resetCamera=false, int subView=-1);

  /// @name info & I/O
  bool modifiersNone();
  bool modifiersShift();
  bool modifiersCtrl();
  arr get3dMousePos(arr& normal=NoArr);
  uint get3dMouseObjID();

  /// @name to display image data (kind of misuse)
  int watchImage(const byteA& img, bool wait, float backgroundZoom=1.);
  int watchImage(const floatA& img, bool wait, float backgroundZoom=1.);
  int displayGrey(const arr& x, bool wait, float backgroundZoom=1.);
  int displayRedBlue(const arr& x, bool wait, float backgroundZoom=1.);

 public: //driver dependent methods
  GLFWwindow *window=0;
  bool needsRedraw=false;

  void openWindow();
  void closeWindow();
  void raiseWindow();
  bool hasWindow();
  void setTitle(const char* _title=0);
  void beginContext(bool fromWithinCallback=false);
  void endContext(bool fromWithinCallback=false);
  void postRedrawEvent(bool fromWithinCallback);

 public:
  //general callbacks (used by all implementations)
  rai::Vector downVec, downPos, downFoc;
  int downModifiers=0;
  rai::Quaternion downRot;
 protected:
  void Key(int key, int mods, bool _keyIsDown);
  void MouseButton(int button, int buttonIsUp, int x, int y, int mods);
  void MouseMotion(double x, double y);
  void Reshape(int w, int h);
  void Scroll(int wheel, int direction);
  void WindowStatus(int status);

  friend struct sOpenGL;
  friend struct GlfwSingleton;
};
