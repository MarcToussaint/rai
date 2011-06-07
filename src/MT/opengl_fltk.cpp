/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

#include <FL/glut.H>
#include <FL/fl_draw.H>

#include "opengl.h"
#include "ors.h"


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL:public Fl_Gl_Window{
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy)
  :Fl_Gl_Window(posx, posy, w, h, title){
    gl=_gl;
  };

  OpenGL *gl;
  int w_old,h_old;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  
  void draw();
  int handle(int event);
};


//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy){
  s = new sOpenGL(this,title,w,h,posx,posy);
  s->w_old=w; s->h_old=h;
  init();
  s->size_range(100,50);
  s->show();
}

OpenGL::~OpenGL(){
  delete s;
}

void OpenGL::redrawEvent(){    s->redraw(); } 
void OpenGL::processEvents(){  Fl::check(); }
void OpenGL::enterEventLoop(){ loopExit=false; while(!loopExit) Fl::wait(); }
void OpenGL::exitEventLoop(){  loopExit=true; }

//! resize the window
void OpenGL::resize(int w,int h){
  s->size(w,h);
}

int OpenGL::width(){  return s->w(); }
int OpenGL::height(){ return s->h(); }

void sOpenGL::draw(){
  if(w_old!=w() || h_old!=h()){ //resized
    w_old=w();  h_old=h();
    gl->Reshape(w_old,h_old);
  }
  gl->Draw(w_old,h_old);
}

int sOpenGL::handle(int event){
  switch(event){
    case FL_PUSH:    gl->Mouse(Fl::event_button()-1, false, Fl::event_x(), Fl::event_y());  break;
    case FL_DRAG:    gl->Motion(Fl::event_x(), Fl::event_y());  break;
    case FL_RELEASE: gl->Mouse(Fl::event_button()-1, true, Fl::event_x(), Fl::event_y());  break;
    case FL_MOVE:    gl->PassiveMotion(Fl::event_x(), Fl::event_y());  break;
    case FL_MOUSEWHEEL:  break; // MouseWheel(int wheel, int direction, Fl::event_x(), Fl::event_y());  break;

    case FL_FOCUS: return 1;
    case FL_HIDE:  gl->loopExit=true;  break;
    
    case FL_KEYDOWN: gl->Key(Fl::event_key(), Fl::event_x(), Fl::event_y());  break;
  }
  return 0;
}
