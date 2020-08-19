/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "opengl_qt.h"

//===========================================================================
//
// sOpenGL implementations
//

bool qtInitialized=false;
QApplication* qtApp;

void initGlEngine() {
  if(!qtInitialized) {
    int argc=1;
    char** argv = new char* [1];
    argv[0] = (char*)"x.exe";
    glutInit(&argc, argv);

    qtApp = new QApplication(argc, argv);
    qtInitialized = true;
  }
}

sOpenGL::sOpenGL(OpenGL* _gl, void* container)
  :QGLWidget(QGLFormat(GLformat), (QWidget*)container) {
  gl=_gl;
  //ownWin = false;
  init();
}

sOpenGL::sOpenGL(QWidget* container)
  :QGLWidget(QGLFormat(GLformat), container) {
  gl = new OpenGL;
  //ownWin = false;
  init();
}

sOpenGL::sOpenGL(OpenGL* _gl, const char* title, int width, int height, int posx, int posy)
  :QGLWidget(QGLFormat(GLformat)) {
  gl = _gl;
  QGLWidget::move(posx, posy);
  QGLWidget::resize(width, height);
  QWidget::setWindowTitle(title);
  init();
}

void sOpenGL::init() {
  QWidget::setMouseTracking(true);
  QGLWidget::show();
  //osPixmap=0;
  //osContext=0;
  quitLoopOnTimer=gl->reportEvents=false;
}

sOpenGL::~sOpenGL() {
  //if(osContext) delete osContext;
  //if(osPixmap) delete osPixmap;
};

//===========================================================================
//
// OpenGL implementations
//

void OpenGL::postRedrawEvent(bool fromWithinCallback) { self->QGLWidget::update(); }
void OpenGL::processEvents() {  qtApp->processEvents(); }
void OpenGL::sleepForEvents()() { qtApp->exec(); }
void OpenGL::exitEventLoop() {  qtApp->exit(); }

//int OpenGL::width(){  return self->QGLWidget::width(); }

//int OpenGL::height(){ return self->QGLWidget::height(); }

/// resize the window
void OpenGL::resize(int w, int h) {
  self->QGLWidget::resize(w, h);
  processEvents();
}

void OpenGL::about(std::ostream& os) {
  os <<"Widget's OpenGL capabilities:\n";
  QGLFormat f=self->format();
  os <<"direct rendering: " <<f.directRendering() <<"\n"
     <<"double buffering: " <<f.doubleBuffer()  <<"\n"
     <<"depth:            " <<f.depth() <<"\n"
     <<"rgba:             " <<f.rgba() <<"\n"
     <<"alpha:            " <<f.alpha() <<"\n"
     <<"accum:            " <<f.accum() <<"\n"
     <<"stencil:          " <<f.stencil() <<"\n"
     <<"stereo:           " <<f.stereo() <<"\n"
     <<"overlay:          " <<f.hasOverlay() <<"\n"
     <<"plane:            " <<f.plane() <<std::endl;

#if 0
  if(!self->osContext) {
    os <<"no off-screen context created yet" <<std::endl;
  } else {
    os <<"Off-screen pixmaps's OpenGL capabilities:\n";
    f=self->osContext->format();
    os <<"direct rendering: " <<f.directRendering() <<"\n"
       <<"double buffering: " <<f.doubleBuffer()  <<"\n"
       <<"depth:            " <<f.depth() <<"\n"
       <<"rgba:             " <<f.rgba() <<"\n"
       <<"alpha:            " <<f.alpha() <<"\n"
       <<"accum:            " <<f.accum() <<"\n"
       <<"stencil:          " <<f.stencil() <<"\n"
       <<"stereo:           " <<f.stereo() <<"\n"
       <<"overlay:          " <<f.hasOverlay() <<"\n"
       <<"plane:            " <<f.plane() <<std::endl;
  }
#endif
}

#if 0 //OLD offscrean code
/** @brief creates a off-screen rendering context for future backround
    rendering routines -- the off-screen context cannot be
    resized... */
void OpenGL::createOffscreen(int width, int height) {
  if(self->osContext && (width>self->osPixmap->width() || height>self->osPixmap->height())) {
    delete self->osContext;
    delete self->osPixmap;
    self->osContext=nullptr;
  }
  if(!self->osContext) {
    self->osPixmap=new QPixmap(width, height);
    if(!self->osPixmap) RAI_MSG("can't create off-screen Pixmap");
    self->osContext=new QGLContext(QGLFormat(GLosformat), self->osPixmap);
    if(!self->osContext->create()) RAI_MSG("can't create off-screen OpenGL context");
  }
}

/** @brief return the RGBA-image of the given perspective; rendering is done
    off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrab(byteA& image) {
  if(image.nd==3) { CHECK_EQ(image.d2, 4, "3rd dim of image has to be 4 for RGBA");} else { CHECK_EQ(image.nd, 2, "image has to be either 2- or 3(for RGBA)-dimensional");}
  setOffscreen(image.d1, image.d0);
  Draw(image.d1, image.d0);
  glGrabImage(image);
}

/** @brief return the RGBA-image of the given perspective; rendering
    is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrab(byteA& image, byteA& depth) {
  if(image.nd==3) { CHECK_EQ(image.d2, 4, "3rd dim of image has to be 4 for RGBA");} else { CHECK_EQ(image.nd, 2, "image has to be either 2- or 3(for RGBA)-dimensional");}
  CHECK_EQ(depth.nd, 2, "depth buffer has to be either 2-dimensional");
  setOffscreen(image.d1, image.d0);
  Draw(image.d1, image.d0);
  glGrabImage(image);
  glGrabDepth(depth);
}

/** @brief return only the depth gray-scale map of given perspective;
    rendering is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrabDepth(byteA& depth) {
  CHECK_EQ(depth.nd, 2, "depth buffer has to be either 2-dimensional");
  setOffscreen(depth.d1, depth.d0);
  Draw(depth.d1, depth.d0);
  glGrabDepth(depth);
}

/** @brief return only the depth gray-scale map of given perspective;
    rendering is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrabDepth(floatA& depth) {
  CHECK_EQ(depth.nd, 2, "depth buffer has to be either 2-dimensional");
  setOffscreen(depth.d1, depth.d0);
  Draw(depth.d1, depth.d0);
  glGrabDepth(depth);
}

void OpenGL::setOffscreen(int width, int height) {
  createOffscreen(width, height);
  CHECK(width<=self->osPixmap->width() && height<=self->osPixmap->height(),
        "width (" <<width <<") or height (" <<height
        <<") too large for the created pixmap - create and set size earlier!");
  self->osContext->makeCurrent();
  //if(initRoutine) (*initRoutine)();
}
#endif
