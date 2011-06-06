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

#include <X11/Xlib.h>


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
    /*
     * Possibly due to aggressive use of XFlush() and friends,
     * it is possible to have our socket drained but still have
     * unprocessed events.  (Or, this may just be normal with
     * X, anyway?)  We do non-trivial processing of X events
     * after the event-reading loop, in any case, so we
     * need to allow that we may have an empty socket but non-
     * empty event queue.
     */
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

bool loopExit;
void MTprocessEvents(){ glutMainLoopEvent(); }
void MTenterLoop(){     loopExit=false; while(!loopExit){ glutMainLoopEvent(); sleepForEvents(); } }
void MTexitLoop(){      loopExit=true; }


//===========================================================================
//
// OpenGL implementations
//

//! constructor
OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy){
  init();

  if(!nrWins){
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  nrWins++;

  glutInitWindowSize(w,h);
  glutInitWindowPosition(posx,posy);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  windowID = glutCreateWindow(title);

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
      
  if(glwins.N<(uint)windowID+1) glwins.resizeCopy(windowID+1);
  glwins(windowID) = this;

  glutDisplayFunc( _Draw );
  glutKeyboardFunc( _Key );
  glutMouseFunc ( _Mouse ) ;
  glutMotionFunc ( _Motion ) ;
  glutPassiveMotionFunc ( _PassiveMotion ) ;
  glutCloseFunc ( _Close ) ;
  glutReshapeFunc( _Reshape );
  glutSpecialFunc( _Special );
  glutMouseWheelFunc ( _MouseWheel ) ;

  //  glutVisibilityFunc( Visibility );
  //  glutKeyboardUpFunc( KeyUp );
  //  glutSpecialUpFunc( SpecialUp );
  //  glutJoystickFunc( Joystick, 100 );
  //  glutEntryFunc ( Entry ) ;
}

// freeglut destructor
OpenGL::~OpenGL(){
  glutDestroyWindow(windowID);
  glwins(windowID)=0;
  nrWins--;
  if(!nrWins) fgDeinitialize();
  delete WS;
}

//! update the view (in Qt: also starts displaying the window)
bool OpenGL::update(const char *txt){
  pressedkey=0;
  if(txt) text.clr() <<txt;
#ifdef MT_FREEGLUT
  glutSetWindow(windowID);
  glutPostRedisplay();
#endif
#ifdef MT_QTGLUT
  show();
  QGLWidget::update();
#endif
  MTprocessEvents();
  return !pressedkey;
}

//! resize the window
void OpenGL::resize(int w,int h){
#ifdef MT_FREEGLUT
  glutSetWindow(windowID);
  glutReshapeWindow(w,h);
#elif defined MT_QTGLUT
  QGLWidget::resize(w,h);
#endif
  MTprocessEvents();
}

int OpenGL::width(){  glutSetWindow(windowID); return glutGet(GLUT_WINDOW_WIDTH); }
int OpenGL::height(){ glutSetWindow(windowID); return glutGet(GLUT_WINDOW_HEIGHT); }


//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) x
#else
#  define CALLBACK_DEBUG(x)
#endif

