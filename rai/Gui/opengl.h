/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/thread.h"
#include "../Geo/geo.h"
#include <functional>

#ifdef RAI_FLTK
#  include <FL/glut.H>
#endif

#ifdef RAI_FREEGLUT
#  ifndef RAI_MSVC
#    define FREEGLUT_STATIC
#  endif
#  include <GL/freeglut.h>
#endif

#ifdef RAI_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#  ifdef RAI_CUDA
#    undef APIENTRY
#  endif
#  include <GL/glut.h>
#endif

#ifdef RAI_GL2PS
#  include<gl2ps.h>
#endif

#undef Success

namespace rai {
struct Vector;
struct Quaternion;
struct Transformation;
}

//===========================================================================
//
// utility functions
//

void glStandardLight(void*, OpenGL&);
void glStandardScene(void*, OpenGL&);
void glStandardOriginAxes(void*, OpenGL&);

void glColor(float r, float g, float b, float a=1.f);
void glColor(int col);
void glColor(const arr& col);
void id2color(byte rgb[3], uint id);
arr id2color(uint id);
uint color2id(byte rgb[3]);
void glDrawText(const char* txt, float x=0., float y=0., float z=0., bool largeFont=false);
//void glShadowTransform();
void glTransform(const rai::Transformation& t);
void glTransform(const double pos[3], const double R[12]);
void glRotate(const rai::Quaternion& rot);
void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4,
                float r, float g, float b);
void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4);
void glDrawRect(float x, float y, float z, float rad);
void glDrawPolygon(const arr& P);
void glDrawFloor(float x, float r, float g, float b);
void glDrawBox(float x, float y, float z, bool linesOnly=false);
void glDrawDiamond(float dx, float dy, float dz);
void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz);
void glDrawSphere(float radius);
void glDrawDisk(float radius);
void glDrawProxy(const arr& p1, const arr& p2, double diskSize=.02, int colorCode=0, const arr& norm=NoArr, double _rad1=0., double _rad2=0.);
void glDrawCylinder(float radius, float length, bool closed=true);
void glDrawCappedCylinder(float radius, float length);
void glDrawAxis(double scale=-1.);
void glDrawAxes(double scale, bool colored=true);
void glDrawCamera(const rai::Camera& cam);
void glDrawGridBox(float x);
void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2);
void glDrawKhepera();
void glMakeSquare(int num);
void glMakeStdSimplex(int num);
void glMakeTorus(int num);
void glDrawRobotArm(float a, float b, float c, float d, float e, float f);
uint glImageTexture(const byteA& img);
void glDrawTexQuad(const byteA& img,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX=1., float mulY=1.);
//grabImage: use OpenGL::capture instead!
void glRasterImage(float x, float y, byteA& img, float zoom=1.);

void read_png(byteA& img, const char* file_name, bool swap_rows);


//===========================================================================
//
// OpenGL class
//

/** @brief A class to display and control 3D scenes using OpenGL and Qt.

    Minimal use: call \ref add to add routines or objects to be drawn
    and \ref update or \ref watch to start the display. */
struct OpenGL {
  unique_ptr<struct sOpenGL> self;

  /// @name little structs to store objects and callbacks
  struct GLInitCall { virtual bool glInit(OpenGL&) = 0; };
  struct GLHoverCall { virtual bool hoverCallback(OpenGL&) = 0; };
  struct GLClickCall { virtual bool clickCallback(OpenGL&) = 0; };
  struct GLKeyCall  { virtual bool keyCallback(OpenGL&) = 0; };
  struct GLEvent    { int button, key, x, y; float dx, dy; void set(int b, int k, int _x, int _y, float _dx, float _dy) { button=b; key=k; x=_x; y=_y; dx=_dx; dy=_dy; } };
  struct GLSelect   { int name; double dmin, dmax, x, y, z; };
  struct GLView     { double le, ri, bo, to;  rai::Array<GLDrawer*> drawers;  rai::Camera camera;  byteA* img;  rai::String text;  GLView() { img=nullptr; le=bo=0.; ri=to=1.; } };

  /// @name data fields
  rai::Array<GLView> views;            ///< list of subviews
  rai::Array<GLDrawer*> drawers;       ///< list of draw routines
  rai::Array<GLInitCall*> initCalls;   ///< list of initialization routines
  rai::Array<GLHoverCall*> hoverCalls; ///< list of hover callbacks
  rai::Array<GLClickCall*> clickCalls; ///< list of click callbacks
  rai::Array<GLKeyCall*> keyCalls;     ///< list of click callbacks
  rai::Array<struct CstyleDrawer*> toBeDeletedOnCleanup;

  rai::String title;     ///< the window title
  uint width, height;
  bool offscreen;
  rai::Camera camera;     ///< the camera used for projection
  rai::String text;        ///< the text to be drawn as title within the opengl frame
  float clearR, clearG, clearB, clearA;  ///< colors of the beackground (called in glClearColor(...))
  bool reportEvents, reportSelects;    ///< flags for verbosity
  int pressedkey;         ///< stores the key pressed
  const char* exitkeys;   ///< additional keys to exit watch mode
  int modifiers;          ///< stores modifier keys
  int mouse_button;       ///< stores which button was pressed
  int mouseposx, mouseposy;  ///< current x- and y-position of mouse
  int mouseView;
  bool mouseIsDown;
  rai::Array<GLSelect> selection; ///< list of all selected objects
  GLSelect* topSelection;        ///< top selected object
  bool drawFocus;
  byteA background, captureImage;
  floatA captureDepth;
  double backgroundZoom;
  arr P; //camera projection matrix
  Mutex dataLock; //'data' means anything: member fields (camera, variables), drawers, data the drawers access
//  uint fbo, render_buf;
  uint fboId;
  uint rboColor;
  uint rboDepth;
  Signaler isUpdating;
  Signaler watching;
  OpenGLDrawOptions drawOptions;

  /// @name constructors & destructors
  OpenGL(const char* title="rai::OpenGL", int w=400, int h=400, bool _offscreen=false);
  //OpenGL(void *parent, const char* title="rai::OpenGL", int w=400, int h=400, int posx=-1, int posy=-1);
  OpenGL(void* container); //special constructor: used when the underlying system-dependent class exists already

  OpenGL* newClone() const;
  ~OpenGL();

  /// @name adding drawing routines and callbacks
  void clear();
  void add(void (*call)(void*, OpenGL&), void* classP=nullptr);
  void addInit(void (*call)(void*), void* classP=nullptr);
  void add(std::function<void(OpenGL&)> drawer);
  void add(GLDrawer& c) { auto _dataLock = dataLock(RAI_HERE); drawers.append(&c); }
  template<class T> void add(Var<T>& c) { add(c.set()); }
  void addDrawer(GLDrawer* c) { auto _dataLock = dataLock(RAI_HERE); drawers.append(c); }
  void remove(GLDrawer& c) { auto _dataLock = dataLock(RAI_HERE); drawers.removeValue(&c); }
  //template<class T> void add(const T& x) { add(x.staticDraw, &x); } ///< add a class or struct with a staticDraw routine
  void addHoverCall(GLHoverCall* c) { hoverCalls.append(c); }
  void addClickCall(GLClickCall* c) { clickCalls.append(c); }
  void addKeyCall(GLKeyCall* c) { keyCalls.append(c); }
  void addSubView(uint view, void (*call)(void*, OpenGL&), void* classP=0);
  void addSubView(uint view, GLDrawer& c);
  void setSubViewTiles(uint cols, uint rows);
  void setSubViewPort(uint view, double l, double r, double b, double t);
  void clearSubView(uint view);

  /// @name the core draw routines (actually only for internal use)
  void Draw(int w, int h, rai::Camera* cam=nullptr, bool callerHasAlreadyLocked=false);
  void Select(bool callerHasAlreadyLocked=false);
  void renderInBack(int w=-1, int h=-1);

  /// @name showing, updating, and watching
  int update(const char* text=nullptr, bool nonThreaded=false);
  int watch(const char* text=nullptr);
  int timedupdate(double sec);
  void resize(int w, int h);
  void setClearColors(float r, float g, float b, float a);
  void unproject(double& x, double& y, double& z, bool resetCamera=false, int subView=-1);
  void project(double& x, double& y, double& z, bool resetCamera=false, int subView=-1);

  /// @name info & I/O
  void reportSelection();
  void saveEPS(const char* filename);
  void about(std::ostream& os=std::cout);

  /// @name to display image data (kind of misuse)
  int watchImage(const byteA& img, bool wait, float backgroundZoom=1.);
  int watchImage(const floatA& img, bool wait, float backgroundZoom=1.);
  int displayGrey(const floatA& x, bool wait, float backgroundZoom=1.);
  int displayGrey(const arr& x, bool wait, float backgroundZoom=1.);
  int displayRedBlue(const arr& x, bool wait, float backgroundZoom=1.);

  void drawId(uint id);

 public: //driver dependent methods
  void openWindow();
  void closeWindow();
  void setTitle(const char* _title=0);
  void beginNonThreadedDraw();
  void endNonThreadedDraw();
  void postRedrawEvent(bool fromWithinCallback);

 public:
  GLEvent lastEvent;
  static uint selectionBuffer[1000];

  void init(); //initializes camera etc

  //general callbacks (used by all implementations)
 protected:
  rai::Vector downVec, downPos, downFoc;
  rai::Quaternion downRot;
  void Key(unsigned char key, int mods);
  void MouseButton(int button, int updown, int x, int y, int mods);
  void MouseMotion(int x, int y);
  void Reshape(int w, int h);
  void Scroll(int wheel, int direction);
  void WindowStatus(int status);

  friend struct sOpenGL;
  friend struct GlfwSpinner;
  friend bool glClickUI(void* p, OpenGL* gl);
};

//===========================================================================

struct SingleGLAccess {
};

extern Singleton<SingleGLAccess> singleGLAccess;

//===========================================================================

//
// simplest UI
//

struct glUI:OpenGL::GLHoverCall, OpenGL::GLClickCall {
  int top;
  struct Button { byteA img1, img2; bool hover; uint x, y, w, h; const char* name; };
  rai::Array<Button> buttons;

  glUI() { top=-1; }

  void addButton(uint x, uint y, const char* name, const char* img1=0, const char* img2=0);
  void glDraw(OpenGL& gl);
  bool checkMouse(int _x, int _y);

  bool hoverCallback(OpenGL&);
  bool clickCallback(OpenGL&);
};

void glDrawUI(void* p, OpenGL&);

extern OpenGL& NoOpenGL;
