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

#include "opengl.h"
#include "ors.h"

#include <FL/fl_draw.H>

extern uint nrWins;

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
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  
  void draw();
  int handle(int event);
};


//===========================================================================
//
// special trick for the event loop
//



//===========================================================================
//
// OpenGL implementations
//

void fltk_callback(Fl_Widget*, void*);

//! constructor
OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy){
  s = new sOpenGL(this,title,w,h,posx,posy);

  init();
  s->show();
  
  nrWins++;

  windowID = nrWins;
      
  if(glwins.N<(uint)windowID+1) glwins.resizeCopy(windowID+1);
  glwins(windowID) = this;

  /*glutDisplayFunc( _Draw );
  glutKeyboardFunc( _Key );
  glutMouseFunc ( _Mouse ) ;
  glutMotionFunc ( _Motion ) ;
  glutPassiveMotionFunc ( _PassiveMotion ) ;
  glutCloseFunc ( _Close ) ;
  glutReshapeFunc( _Reshape );
  glutSpecialFunc( _Special );
  glutMouseWheelFunc ( _MouseWheel ) ;
  */

  //  glutVisibilityFunc( Visibility );
  //  glutKeyboardUpFunc( KeyUp );
  //  glutSpecialUpFunc( SpecialUp );
  //  glutJoystickFunc( Joystick, 100 );
  //  glutEntryFunc ( Entry ) ;
}

// freeglut destructor
OpenGL::~OpenGL(){
  glwins(windowID)=0;
  nrWins--;
  delete s;
}

void OpenGL::redrawEvent(){  s->redraw(); } 
void OpenGL::processEvents(){ Fl::check(); }
void OpenGL::enterEventLoop(){     loopExit=false; while(!loopExit){ Fl::check(); MT::wait(.1); } }
void OpenGL::exitEventLoop(){      loopExit=true; }

//! resize the window
void OpenGL::resize(int w,int h){
  NIY;
}

int OpenGL::width(){  return s->w(); }
int OpenGL::height(){ return s->h(); }

void sOpenGL::draw(){
  gl->Draw(w(),h());
}

int sOpenGL::handle(int event){
  switch(event){
    case FL_PUSH:    gl->Mouse(Fl::event_button()-1, false, Fl::event_x(), Fl::event_y());  break;
    case FL_DRAG:    gl->Motion(Fl::event_x(), Fl::event_y());  break;
    case FL_RELEASE: gl->Mouse(Fl::event_button()-1, true, Fl::event_x(), Fl::event_y());  break;
    case FL_MOVE:    gl->PassiveMotion(Fl::event_x(), Fl::event_y());  break;
    case FL_MOUSEWHEEL:  break; // MouseWheel(int wheel, int direction, Fl::event_x(), Fl::event_y());  break;

    case FL_FOCUS: return 1;
    
    case FL_KEYDOWN: gl->Key(Fl::event_key(), Fl::event_x(), Fl::event_y());  break;
  }
}


//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) x
#else
#  define CALLBACK_DEBUG(x)
#endif

