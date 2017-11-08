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


#ifndef MLR_opengl_qt_h
#define MLR_opengl_qt_h

#include "opengl.h"
#include <Geo/geo.h>

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
#if defined MLR_Cygwin //|| defined MLR_Linux
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#  else
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#endif
#define MLR_GLUT

#ifdef MLR_MSVC
#  include<windows.h>
#  undef min //I hate it that windows defines these macros!
#  undef max
#endif

struct sOpenGL: QGLWidget {
  Q_OBJECT
public:
  OpenGL *gl;
  mlr::Vector downVec,downPos,downFoc;
  mlr::Quaternion downRot;
  bool quitLoopOnTimer;
  
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy);
  sOpenGL(OpenGL *gl, void *container);
  sOpenGL(QWidget *container);
  ~sOpenGL();
  void init();
  void beginGlContext() {};
  void endGlContext() {};
  
  //hooks for Qt (overloading virtuals of QGLWidget)
  void paintGL() { gl->Draw(width(),height()); }
  void initializeGL() { }
  void resizeGL(int w,int h) { gl->Reshape(w,h); }
  void keyPressEvent(QKeyEvent *e) { gl->pressedkey=e->text().toAscii()[0]; gl->Key(gl->pressedkey,gl->mouseposx,gl->mouseposy); }
  void timerEvent(QTimerEvent*) { if(quitLoopOnTimer) gl->exitEventLoop(); }
  void mouseMoveEvent(QMouseEvent* e) {
    if(!gl->mouseIsDown) gl->Motion(e->x(),e->y()); else gl->Motion(e->x(),e->y());
  }
  void mousePressEvent(QMouseEvent* e) {
    if(e->button()==Qt::LeftButton) { gl->Mouse(0,0,e->x(),e->y()); }
    if(e->button()==Qt::MidButton)  { gl->Mouse(1,0,e->x(),e->y()); }
    if(e->button()==Qt::RightButton) { gl->Mouse(2,0,e->x(),e->y()); }
  }
  void mouseReleaseEvent(QMouseEvent* e) {
    if(e->button()==Qt::LeftButton) { gl->Mouse(0,1,e->x(),e->y()); }
    if(e->button()==Qt::MidButton)  { gl->Mouse(1,1,e->x(),e->y()); }
    if(e->button()==Qt::RightButton) { gl->Mouse(2,1,e->x(),e->y()); }
  }
  
  
#if 0
  /* OLD offscrean code*/
  QPixmap *osPixmap;      // the paint device for off-screen rendering
  QGLContext *osContext;  //the GL context for off-screen rendering
  void createOffscreen(int width,int height);
  void offscreenGrab(byteA& image);
  void offscreenGrab(byteA& image,byteA& depth);
  void offscreenGrabDepth(byteA& depth);
  void offscreenGrabDepth(floatA& depth);
  void setOffscreen(int width,int height);
#endif
};


#endif
