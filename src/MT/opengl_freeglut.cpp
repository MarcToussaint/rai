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

#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#include <X11/Xlib.h>

#include "opengl.h"
#include "ors.h"


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL{
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy){
    gl=_gl;
  };

  static uint nrWins;
  static MT::Array<OpenGL*> glwins;    //!< global window list
  int windowID;                        //!< id of this window in the global glwins list

  OpenGL *gl;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  
  void draw();
  int handle(int event);
  
  //hooks for FREEGLUT (static callbacks)
  static void _Void(){ }
  static void _Draw(){ OpenGL *gl=glwins(glutGetWindow()); gl->Draw(gl->width(),gl->height()); glutSwapBuffers(); }
  static void _Key(unsigned char key, int x, int y){ glwins(glutGetWindow())->Key(key,x,y); }
  static void _Mouse(int button, int updown, int x, int y){ glwins(glutGetWindow())->Mouse(button,updown,x,y); }
  static void _Motion(int x, int y){ glwins(glutGetWindow())->Motion(x,y); }
  static void _PassiveMotion(int x, int y){ glwins(glutGetWindow())->PassiveMotion(x,y); }
  static void _Close(){ glwins(glutGetWindow())->Close(); }
  static void _Reshape(int w,int h){ glwins(glutGetWindow())->Reshape(w,h); }
  static void _Special(int key, int x, int y){ glwins(glutGetWindow())->Special(key,x,y); }
  static void _MouseWheel(int wheel, int direction, int x, int y){ glwins(glutGetWindow())->MouseWheel(wheel,direction,x,y); }
};

uint sOpenGL::nrWins=0;
MT::Array<OpenGL*> sOpenGL::glwins;


//===========================================================================
//
// special trick for the event loop
//

extern "C"{
  void fgDeinitialize( void );
}
struct SFG_Display_dummy{
    _XDisplay*        Display;            /* The display we are being run in.  */
};
extern SFG_Display_dummy fgDisplay;

static void sleepForEvents( void ){
#ifdef MT_Linux
  if( ! XPending( fgDisplay.Display ) ){
    fd_set fdset;
    int err;
    int socket;
    struct timeval wait;

    socket = ConnectionNumber( fgDisplay.Display );
    FD_ZERO( &fdset );
    FD_SET( socket, &fdset );
    wait.tv_sec = 10000 / 1000;
    wait.tv_usec = (10000 % 1000) * 1000;
    err = select( socket+1, &fdset, NULL, NULL, &wait );

#if HAVE_ERRNO
  if( ( -1 == err ) && ( errno != EINTR ) )
    fgWarning ( "freeglut select() error: %d", errno );
#endif
  }
#elif defined MT_MSVC
  MsgWaitForMultipleObjects( 0, NULL, FALSE, msec, QS_ALLINPUT );
#endif
}


//===========================================================================
//
// OpenGL implementations
//

//! constructor
OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy){
  s=new sOpenGL(this,title,w,h,posx,posy);
  init();

  if(!s->nrWins){
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  s->nrWins++;

  glutInitWindowSize(w,h);
  glutInitWindowPosition(posx,posy);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  s->windowID = glutCreateWindow(title);

  //OpenGL initialization
  //two optional thins:
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE); glFrontFace(GL_CCW);
  //glDisable(GL_CULL_FACE);
  glDepthFunc(GL_LESS);
  glShadeModel(GL_SMOOTH);
  glShadeModel(GL_FLAT);
      
  if(s->glwins.N<(uint)s->windowID+1) s->glwins.resizeCopy(s->windowID+1);
  s->glwins(s->windowID) = this;

  glutDisplayFunc( s->_Draw );
  glutKeyboardFunc( s->_Key );
  glutMouseFunc ( s->_Mouse ) ;
  glutMotionFunc ( s->_Motion ) ;
  glutPassiveMotionFunc ( s->_PassiveMotion ) ;
  glutCloseFunc ( s->_Close ) ;
  glutReshapeFunc( s->_Reshape );
  glutSpecialFunc( s->_Special );
  glutMouseWheelFunc ( s->_MouseWheel ) ;

  //  glutVisibilityFunc( Visibility );
  //  glutKeyboardUpFunc( KeyUp );
  //  glutSpecialUpFunc( SpecialUp );
  //  glutJoystickFunc( Joystick, 100 );
  //  glutEntryFunc ( Entry ) ;
}

// freeglut destructor
OpenGL::~OpenGL(){
  glutDestroyWindow(s->windowID);
  s->glwins(s->windowID)=0;
  s->nrWins--;
  if(!s->nrWins) fgDeinitialize();
  delete s;
}

void OpenGL::redrawEvent(){  glutSetWindow(s->windowID);  glutPostRedisplay(); } 
void OpenGL::processEvents(){  glutMainLoopEvent(); }
void OpenGL::enterEventLoop(){ loopExit=false; while(!loopExit){ glutMainLoopEvent(); sleepForEvents(); } }
void OpenGL::exitEventLoop(){  loopExit=true; }

//! resize the window
void OpenGL::resize(int w,int h){
  glutSetWindow(s->windowID);
  glutReshapeWindow(w,h);
  processEvents();
}

int OpenGL::width(){  glutSetWindow(s->windowID); return glutGet(GLUT_WINDOW_WIDTH); }
int OpenGL::height(){ glutSetWindow(s->windowID); return glutGet(GLUT_WINDOW_HEIGHT); }

