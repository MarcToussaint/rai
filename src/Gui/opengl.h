/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


/// @file
/// @ingroup group_Gui
/// @addtogroup group_Gui
/// @{

#ifndef MLR_opengl_h
#define MLR_opengl_h

#include <Core/array.h>
#include <Core/thread.h>
#include <Geo/geo.h>

#ifndef MLR_QTGL
#  include <X11/Xlib.h>
#endif

#ifdef MLR_FLTK
#  include <FL/glut.H>
#endif

#ifdef MLR_FREEGLUT
#  ifndef MLR_MSVC
#    define FREEGLUT_STATIC
#  endif
#  include <GL/freeglut.h>
#endif

#ifdef MLR_GL
#  include <GL/gl.h>
#  include <GL/glu.h>
#  ifdef MLR_CUDA
#    undef APIENTRY
#  endif
#  include <GL/glut.h>
#endif

#ifdef MLR_GL2PS
#  include<gl2ps.h>
#endif

#undef Success

namespace mlr {
struct Vector;
struct Quaternion;
struct Transformation;
}

//===========================================================================
//
// utility functions
//

void glStandardLight(void*);
void glStandardScene(void*);
void glColor(float r, float g, float b, float a=1.f);
void glColor(int col);
void glColor(const arr& col);
void glDrawText(const char* txt, float x, float y, float z, bool largeFont=false);
//void glShadowTransform();
void glTransform(const mlr::Transformation& t);
void glTransform(const double pos[3], const double R[12]);
void glRotate(const mlr::Quaternion& rot);
void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4,
                float r, float g, float b);
void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4);
void glDrawRect(float x, float y, float z, float rad);
void glDrawFloor(float x, float r, float g, float b);
void glDrawBox(float x, float y, float z, bool linesOnly=false);
void glDrawDiamond(float dx, float dy, float dz);
void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz);
void glDrawSphere(float radius);
void glDrawDisk(float radius);
void glDrawCylinder(float radius, float length, bool closed=true);
void glDrawCappedCylinder(float radius, float length);
void glDrawAxis();
void glDrawAxes(double scale);
void glDrawGridBox(float x);
void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2);
void glDrawKhepera();
void glMakeSquare(int num);
void glMakeStdSimplex(int num);
void glMakeTorus(int num);
void glDrawRobotArm(float a, float b, float c, float d, float e, float f);
uint glImageTexture(const byteA &img);
void glDrawTexQuad(const byteA &img,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX=1., float mulY=1.);
//grabImage: use OpenGL::capture instead!
void glRasterImage(float x, float y, byteA &img, float zoom=1.);


//===========================================================================
//
// OpenGL class
//

/** @brief A class to display and control 3D scenes using OpenGL and Qt.

    Minimal use: call \ref add to add routines or objects to be drawn
    and \ref update or \ref watch to start the display. */
struct OpenGL {
  struct sOpenGL *s;
  
  /// @name little structs to store objects and callbacks
  struct GLInitCall { virtual bool glInit(OpenGL&) = 0; };
  struct GLHoverCall { virtual bool hoverCallback(OpenGL&) = 0; };
  struct GLClickCall { virtual bool clickCallback(OpenGL&) = 0; };
  struct GLKeyCall  { virtual bool keyCallback(OpenGL&) = 0; };
  struct GLEvent    { int button, key, x, y; float dx, dy; void set(int b, int k, int _x, int _y, float _dx, float _dy) { button=b; key=k; x=_x; y=_y; dx=_dx; dy=_dy; } };
  struct GLSelect   { int name; double dmin, dmax, x,y,z; };
  struct GLView     { double le, ri, bo, to;  mlr::Array<GLDrawer*> drawers;  mlr::Camera camera;  byteA *img;  mlr::String text;  GLView() { img=NULL; le=bo=0.; ri=to=1.; } };
  
  /// @name data fields
  mlr::Array<GLView> views;            ///< list of draw routines
  mlr::Array<GLDrawer*> drawers;        ///< list of draw routines
  mlr::Array<GLInitCall*> initCalls;    ///< list of initialization routines
  mlr::Array<GLHoverCall*> hoverCalls; ///< list of hover callbacks
  mlr::Array<GLClickCall*> clickCalls; ///< list of click callbacks
  mlr::Array<GLKeyCall*> keyCalls;     ///< list of click callbacks

  mlr::String title;     ///< the window title
  uint width, height;
  mlr::Camera camera;     ///< the camera used for projection
  mlr::String text;        ///< the text to be drawn as title within the opengl frame
  float clearR, clearG, clearB, clearA;  ///< colors of the beackground (called in glClearColor(...))
  bool reportEvents, reportSelects;    ///< flags for verbosity
  int pressedkey;         ///< stores the key pressed
  const char *exitkeys;   ///< additional keys to exit watch mode
  int mouse_button;       ///< stores which button was pressed
  int mouseposx, mouseposy;  ///< current x- and y-position of mouse
  int mouseView;
  bool mouseIsDown;
  mlr::Array<GLSelect> selection; ///< list of all selected objects
  GLSelect *topSelection;        ///< top selected object
  bool immediateExitLoop;
  bool drawFocus;
  bool captureImg, captureDep;
  byteA background, captureImage;
  floatA captureDepth;
  double backgroundZoom;
  arr P; //camera projection matrix
  RWLock dataLock; //'data' means anything: member fields (camera, variables), drawers, data the drawers access
//  uint fbo, render_buf;
  uint fboId;
  uint rboColor;
  uint rboDepth;
  Signaler isUpdating;
  Signaler watching;

  /// @name constructors & destructors
  OpenGL(const char* title="mlr::OpenGL", int w=400, int h=400, int posx=-1, int posy=-1);
  //OpenGL(void *parent, const char* title="mlr::OpenGL", int w=400, int h=400, int posx=-1, int posy=-1);
  OpenGL(void *container); //special constructor: used when the underlying system-dependent class exists already
  
  OpenGL *newClone() const;
  ~OpenGL();
  
  /// @name adding drawing routines and callbacks
  void clear();
  void add(void (*call)(void*), void* classP=NULL);
  void addInit(void (*call)(void*), void* classP=NULL);
  void add(GLDrawer& c){ dataLock.writeLock(); drawers.append(&c); dataLock.unlock(); }
  void addDrawer(GLDrawer *c){ dataLock.writeLock(); drawers.append(c); dataLock.unlock(); }
  void remove(void (*call)(void*), const void* classP=0);
  //template<class T> void add(const T& x) { add(x.staticDraw, &x); } ///< add a class or struct with a staticDraw routine
  void addHoverCall(GLHoverCall *c){ hoverCalls.append(c); }
  void addClickCall(GLClickCall *c){ clickCalls.append(c); }
  void addKeyCall(GLKeyCall *c){ keyCalls.append(c); }
  void addView(uint view, void (*call)(void*), void* classP=0);
  void addSubView(uint view, GLDrawer& c);
  void setSubViewTiles(uint cols, uint rows);
  void setViewPort(uint view, double l, double r, double b, double t);
  
  /// @name the core draw routines (actually only for internal use)
  void Draw(int w, int h, mlr::Camera *cam=NULL, bool callerHasAlreadyLocked=false);
  void Select(bool callerHasAlreadyLocked=false);
  void renderInBack(bool captureImg=true, bool captureDepth=false, int w=-1, int h=-1);

  /// @name showing, updating, and watching
  int update(const char *text=NULL, bool captureImg=false, bool captureDepth=false, bool waitForCompletedDraw=true);
  int watch(const char *text=NULL);
  int timedupdate(double sec);
  void resize(int w, int h);
  void setClearColors(float r, float g, float b, float a);
  void unproject(double &x, double &y, double &z, bool resetCamera=false);
  
  /// @name info & I/O
  void reportSelection();
  void saveEPS(const char *filename);
  void about(std::ostream& os=std::cout);
  
  /// @name to display image data (kind of misuse)
  void watchImage(const byteA &img, bool wait, float backgroundZoom);
  void watchImage(const floatA &img, bool wait, float backgroundZoom);
  void displayGrey(const arr &x, bool wait, float backgroundZoom);
  void displayRedBlue(const arr &x, bool wait, float backgroundZoom);
  
public: //driver dependent methods
  void openWindow();
  void closeWindow();
  void postRedrawEvent(bool fromWithinCallback);
#if !defined MLR_MSVC && !defined MLR_QTGL
  Display* xdisplay();
  Drawable xdraw();
#endif

protected:
  GLEvent lastEvent;
  static uint selectionBuffer[1000];
  
  void init(); //initializes camera etc

  //general callbacks (used by all implementations)
  void Key(unsigned char key, int x, int y);
  void Mouse(int button, int updown, int x, int y);
  void Motion(int x, int y);
public:
  void Reshape(int w, int h);
protected:
  void MouseWheel(int wheel, int direction, int x, int y);
  
  friend struct sOpenGL;
  friend bool glClickUI(void *p, OpenGL *gl);
};


//===========================================================================

struct SingleGLAccess{
//  Mutex openglMutex;
//  void lock(){ openglMutex.lock(); }
//  void unlock(){ openglMutex.unlock(); }
};

extern Singleton<SingleGLAccess> singleGLAccess;

//===========================================================================

//
// simplest UI
//

struct glUI:OpenGL::GLHoverCall,OpenGL::GLClickCall {
  int top;
  struct Button { byteA img1, img2; bool hover; uint x, y, w, h; const char* name; };
  mlr::Array<Button> buttons;
  
  glUI() { top=-1; }
  
  void addButton(uint x, uint y, const char *name, const char *img1=0, const char *img2=0);
  void glDraw();
  bool checkMouse(int _x, int _y);
  
  bool hoverCallback(OpenGL&);
  bool clickCallback(OpenGL&);
};

void glDrawUI(void *p);

extern OpenGL& NoOpenGL;

/// @}

#endif
