/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "opengl.h"
#include "../Geo/geo.h"

#include <GL/glut.h>
#undef scroll
#undef border
// #  define QT3_SUPPORT
// #  include <Qt/Qt3Support>
// #  include <Qt/qtimer.h>
#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
// #  include <Qt/qobject.h>
// #  include <Qt/qevent.h>
#include <QtOpenGL/QtOpenGL>
#if defined RAI_Cygwin //|| defined RAI_Linux
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#  else
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#endif
#define RAI_GLUT

#ifdef RAI_MSVC
#  include<windows.h>
#  undef min //I hate it that windows defines these macros!
#  undef max
#endif

struct sOpenGL: QGLWidget {
  Q_OBJECT
 public:
  OpenGL* gl;
  rai::Vector downVec, downPos, downFoc;
  rai::Quaternion downRot;
  bool quitLoopOnTimer;

  sOpenGL(OpenGL* _gl, const char* title, int w, int h, int posx, int posy);
  sOpenGL(OpenGL* gl, void* container);
  sOpenGL(QWidget* container);
  ~sOpenGL();
  void init();
  void beginGlContext() {};
  void endGlContext() {};

  //hooks for Qt (overloading virtuals of QGLWidget)
  void paintGL() { gl->Draw(width(), height()); }
  void initializeGL() { }
  void resizeGL(int w, int h) { gl->Reshape(w, h); }
  void keyPressEvent(QKeyEvent* e) { gl->pressedkey=e->text().toAscii()[0]; gl->Key(gl->pressedkey, gl->mouseposx, gl->mouseposy); }
  void timerEvent(QTimerEvent*) { if(quitLoopOnTimer) gl->exitEventLoop(); }
  void mouseMoveEvent(QMouseEvent* e) {
    if(!gl->mouseIsDown) gl->MouseMotion(e->x(), e->y()); else gl->MouseMotion(e->x(), e->y());
  }
  void mousePressEvent(QMouseEvent* e) {
    if(e->button()==Qt::LeftButton) { gl->MouseButton(0, 0, e->x(), e->y()); }
    if(e->button()==Qt::MidButton)  { gl->MouseButton(1, 0, e->x(), e->y()); }
    if(e->button()==Qt::RightButton) { gl->MouseButton(2, 0, e->x(), e->y()); }
  }
  void mouseReleaseEvent(QMouseEvent* e) {
    if(e->button()==Qt::LeftButton) { gl->MouseButton(0, 1, e->x(), e->y()); }
    if(e->button()==Qt::MidButton)  { gl->MouseButton(1, 1, e->x(), e->y()); }
    if(e->button()==Qt::RightButton) { gl->MouseButton(2, 1, e->x(), e->y()); }
  }

#if 0
  /* OLD offscrean code*/
  QPixmap* osPixmap;      // the paint device for off-screen rendering
  QGLContext* osContext;  //the GL context for off-screen rendering
  void createOffscreen(int width, int height);
  void offscreenGrab(byteA& image);
  void offscreenGrab(byteA& image, byteA& depth);
  void offscreenGrabDepth(byteA& depth);
  void offscreenGrabDepth(floatA& depth);
  void setOffscreen(int width, int height);
#endif
};
