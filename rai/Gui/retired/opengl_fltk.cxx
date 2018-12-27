/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include <FL/glut.H>
#include <FL/fl_draw.H>

#include "opengl.h"
#include "kin.h"

//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL:public Fl_Gl_Window {
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy)
    :Fl_Gl_Window(posx, posy, w, h, title) {
    gl=_gl;
  };
  
  OpenGL *gl;
  int w_old,h_old;
  
  rai::Vector downVec,downPos,downFoc;
  rai::Quaternion downRot;
  
  void draw();
  int handle(int event);
};

//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy) {
  s = new sOpenGL(this,title,w,h,posx,posy);
  s->w_old=w; s->h_old=h;
  init();
  s->size_range(100,50);
  s->show();
}

OpenGL::~OpenGL() {
  delete s;
}

void OpenGL::postRedrawEvent() { s->redraw(); }
void OpenGL::processEvents() {  Fl::check(); }
void OpenGL::sleepForEvents()() { loopExit=false; while(!loopExit) Fl::wait(); }

/// resize the window
void OpenGL::resize(int w,int h) {
  s->size(w,h);
}

int OpenGL::width() {  return s->w(); }
int OpenGL::height() { return s->h(); }

void sOpenGL::draw() {
  Fl::lock();
  if(w_old!=w() || h_old!=h()) { //resized
    w_old=w();  h_old=h();
    gl->Reshape(w_old,h_old);
  }
  gl->Draw(w_old,h_old);
  Fl::unlock();
}

int sOpenGL::handle(int event) {
  switch(event) {
    case FL_PUSH:    gl->MouseButton(Fl::event_button()-1, false, Fl::event_x(), Fl::event_y());  break;
    case FL_DRAG:    gl->MouseMotion(Fl::event_x(), Fl::event_y());  break;
    case FL_RELEASE: gl->MouseButton(Fl::event_button()-1, true, Fl::event_x(), Fl::event_y());  break;
    case FL_MOVE:    gl->PassiveMotion(Fl::event_x(), Fl::event_y());  break;
    case FL_MOUSEWHEEL:  break; // MouseWheel(int wheel, int direction, Fl::event_x(), Fl::event_y());  break;
    
    case FL_FOCUS: return 1;
    case FL_HIDE:  gl->loopExit=true;  break;
    
    case FL_KEYDOWN: gl->Key(Fl::event_key(), Fl::event_x(), Fl::event_y());  break;
  }
  return 0;
}
