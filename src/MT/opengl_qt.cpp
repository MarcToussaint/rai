#ifdef MT_QTGLUT
#  include <GL/glut.h>
#  undef scroll
#  undef border
// #  define QT3_SUPPORT
// #  include <Qt/Qt3Support>
// #  include <Qt/qtimer.h>
#  include <QtGui/QApplication>
#  include <QtOpenGL/QGLWidget>
// #  include <Qt/qobject.h>
// #  include <Qt/qevent.h>
#  include <QtOpenGL/QtOpenGL>
#  if defined MT_Cygwin //|| defined MT_Linux
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#  else
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#  endif
#  define MT_GLUT
#endif

#ifdef MT_MSVC
#  include<windows.h>
#  undef min //I hate it that windows defines these macros!
#  undef max
#endif


#ifdef MT_QTGLUT
  //hooks for Qt (overloading virtuals)
    void paintGL(){ Draw(width(),height()); }
    void initializeGL(){ }
    void resizeGL(int w,int h){ Reshape(w,h); }
    void keyPressEvent(QKeyEvent *e){ pressedkey=e->text().toAscii()[0]; Key(pressedkey,mouseposx,mouseposy); }
    void timerEvent(QTimerEvent*){ if(quitLoopOnTimer) MTexitLoop(); }
    void mouseMoveEvent(QMouseEvent* e){
      if(!mouseIsDown) PassiveMotion(e->x(),e->y()); else Motion(e->x(),e->y());
    }
    void mousePressEvent(QMouseEvent* e){
      if(e->button()==Qt::LeftButton) { Mouse(0,0,e->x(),e->y()); }
      if(e->button()==Qt::MidButton)  { Mouse(1,0,e->x(),e->y()); }
      if(e->button()==Qt::RightButton){ Mouse(2,0,e->x(),e->y()); }
    }
    void mouseReleaseEvent(QMouseEvent* e){
      if(e->button()==Qt::LeftButton) { Mouse(0,1,e->x(),e->y()); }
      if(e->button()==Qt::MidButton)  { Mouse(1,1,e->x(),e->y()); }
      if(e->button()==Qt::RightButton){ Mouse(2,1,e->x(),e->y()); }
    }
#endif

#ifdef MT_QTGLUT
    bool quitLoopOnTimer;
    QPixmap *osPixmap;      // the paint device for off-screen rendering
    QGLContext *osContext;  //the GL context for off-screen rendering
#endif

//! resize the window
void OpenGL::resize(int w,int h){
#ifdef MT_FREEGLUT
  glutSetWindow(s->windowID);
  glutReshapeWindow(w,h);
#elif defined MT_QTGLUT
  QGLWidget::resize(w,h);
#endif
  MTprocessEvents();
}
