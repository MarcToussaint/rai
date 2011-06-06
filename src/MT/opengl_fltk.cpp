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


//===========================================================================
//
// special trick for the event loop
//


bool loopExit;
void MTprocessEvents(){ Fl::check(); }
void MTenterLoop(){     loopExit=false; while(!loopExit){ Fl::check(); MT::wait(.1); } }
void MTexitLoop(){      loopExit=true; }


//===========================================================================
//
// OpenGL implementations
//

//! constructor
OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy)
  : Fl_Gl_Window(posx, posy, w, h, title){
  init();
  show();
  
  if(!nrWins){
    int argc=1;
    char *argv[1]={(char*)"x"};
  }
  nrWins++;

  //OpenGL initialization
  //two optional thins:
  /*glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE); glFrontFace(GL_CCW);
  //glDisable(GL_CULL_FACE);
  glDepthFunc(GL_LESS);
  glShadeModel(GL_SMOOTH);
  glShadeModel(GL_FLAT);*/
      
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
  delete WS;
}

void OpenGL::draw(){
  Draw(width(),height());
}

//! update the view (in Qt: also starts displaying the window)
bool OpenGL::update(const char *txt){
  pressedkey=0;
  if(txt) text.clr() <<txt;
  redraw();
  Fl::check();
  return !pressedkey;
}

//! resize the window
void OpenGL::resize(int w,int h){
  NIY;
}

int OpenGL::width(){  return glutGet(GLUT_WINDOW_WIDTH); }
int OpenGL::height(){ return glutGet(GLUT_WINDOW_HEIGHT); }


//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) x
#else
#  define CALLBACK_DEBUG(x)
#endif

