/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include <Core/array_t.h>
#include <Core/geo.h>
#include <GL/glew.h>
#include "opengl.h"

//===========================================================================

struct OpenGLEngineAccess{
  Mutex openglMutex;
  void lock(){ openglMutex.lock(); }
  void unlock(){ openglMutex.unlock(); }
};

Singleton<OpenGLEngineAccess> openglAccess;

//===========================================================================

#ifdef MT_FREEGLUT
#  include "opengl_freeglut.cxx"
#endif

#ifdef MT_GTKGL
#  include "opengl_gtk.cxx"
#endif

#ifdef MT_FLTK
#  include "opengl_fltk.cxx"
#endif

#ifdef MT_QTGL
#  include "opengl_qt.cxx"
#endif

#if !defined MT_FREEGLUT && !defined MT_GTKGL && !defined MT_FLTK && !defined MT_QTGL
#  include "opengl_void.cxx"
#else
#  define MT_GLUT
#endif


//===========================================================================
//
// force instantiations
//

template MT::Array<glUI::Button>::Array();
template MT::Array<glUI::Button>::~Array();


//===========================================================================
//
// camera class
//

/** @brief constructor; specify a frame if the camera is to be attached
   to an existing frame. Otherwise the camera creates its own
      frame */
ors::Camera::Camera() {
  X = new Transformation;
  //if(!frame){ f=new Frame; ownFrame=true; }else{ f=frame; ownFrame=false; }
  foc = new ors::Vector;
  setZero();
  
  setPosition(0., 0., 10.);
  focus(0, 0, 0);
  setZRange(.1, 1000.);
  setHeightAngle(12.);
}

ors::Camera::~Camera() {
  delete X;
  delete foc;
}

ors::Camera& ors::Camera::operator=(const ors::Camera& c) {
  //MEM_COPY_OPERATOR(c);
  memmove(this, &c, sizeof(*this));
  foc = new ors::Vector(*c.foc);
  X = new ors::Transformation(*c.X);
  return *this;
}

void ors::Camera::setZero() {
  X->setZero();
  foc->setZero();
  heightAngle=90.;
  heightAbs=10.;
  focalLength=1.;
  whRatio=1.;
  zNear=.1;
  zFar=1000.;
}

/// the height angle (in degrees) of the camera perspective; set it 0 for orthogonal projection
void ors::Camera::setHeightAngle(float a) { heightAngle=a; }
/// the absolute height of the camera perspective (automatically also sets heightAngle=0)
void ors::Camera::setHeightAbs(float h) { heightAngle=0; heightAbs=h; }
/// the z-range (depth range) visible for the camera
void ors::Camera::setZRange(float znear, float zfar) { zNear=znear; zFar=zfar; }
/// set the width/height ratio of your viewport to see a non-distorted picture
void ors::Camera::setWHRatio(float ratio) { whRatio=ratio; }
/// the frame's position
void ors::Camera::setPosition(float x, float y, float z) { X->pos.set(x, y, z); }
/// rotate the frame to focus the absolute coordinate origin (0, 0, 0)
void ors::Camera::focusOrigin() { foc->setZero(); focus(); }
/// rotate the frame to focus the point (x, y, z)
void ors::Camera::focus(float x, float y, float z) { foc->set(x, y, z); focus(); }
/// rotate the frame to focus the point given by the vector
void ors::Camera::focus(const Vector& v) { *foc=v; focus(); }
/// rotate the frame to focus (again) the previously given focus
void ors::Camera::focus() { watchDirection((*foc)-X->pos); } //X->Z=X->pos; X->Z-=foc; X->Z.normalize(); upright(); }
/// rotate the frame to watch in the direction vector D
void ors::Camera::watchDirection(const Vector& d) {
  if(d.x==0. && d.y==0.) {
    X->rot.setZero();
    if(d.z>0) X->rot.setDeg(180, 1, 0, 0);
    return;
  }
  Quaternion r;
  r.setDiff(-X->rot.getZ(), d);
  X->rot=r*X->rot;
}
/// rotate the frame to set it upright (i.e. camera's y aligned with 's z)
void ors::Camera::upright() {
#if 1
  //construct desired X:
  Vector v(0, 0, -1), x(1, 0, 0), dx, up;
  x=X->rot*x; //true X
  v=X->rot*v;
  if(fabs(v.z)<1.) up.set(0, 0, 1); else up.set(0, 1, 0);
  dx=up^v; //desired X
  if(dx*x<=0) dx=-dx;
  Quaternion r;
  r.setDiff(x, dx);
  X->rot=r*X->rot;
#else
  if(X->Z[2]<1.) X->Y.set(0, 0, 1); else X->Y.set(0, 1, 0);
  X->X=X->Y^X->Z; X->X.normalize();
  X->Y=X->Z^X->X; X->Y.normalize();
#endif
}

//}

void ors::Camera::setCameraProjectionMatrix(const arr& P) {
  //P is in standard convention -> computes fixedProjectionMatrix in OpenGL convention from this
  cout <<"desired P=" <<P <<endl;
  arr Kview=ARR(200, 0, 200, 0, 200, 200, 0, 0, 1); //OpenGL's calibration matrix
  Kview.reshape(3, 3);
  //arr glP=inverse(Kview)*P;
  arr glP=P;
  //glP[2]()*=-1.;
  glP.append(glP[2]);
  glP[2]()*=.99; glP(2, 2)*=1.02; //some hack to invent a culling coordinate (usually determined via near and far...)
  glP = ~glP;
  glP *= 1./glP(3, 3);
  cout <<"glP=" <<glP <<endl;
  //glLoadMatrixd(glP.p);
  //fixedProjectionMatrix = glP;
}

/** sets OpenGL's GL_PROJECTION matrix accordingly -- should be
    called in an opengl draw routine */
void ors::Camera::glSetProjectionMatrix() {
#ifdef MT_GL
//  if(fixedProjectionMatrix.N) {
//    glLoadMatrixd(fixedProjectionMatrix.p);
//  } else {
  if(heightAngle==0) {
    if(heightAbs==0) {
      arr P(4,4);
      P.setZero();
      P(0,0) = 2.*focalLength/whRatio;
      P(1,1) = 2.*focalLength;
      P(2,2) = (zFar + zNear)/(zNear-zFar);
      P(2,3) = -1.;
      P(3,2) = 2. * zFar * zNear / (zNear-zFar);
      glLoadMatrixd(P.p);
    }else{
      glOrtho(-whRatio*heightAbs/2, whRatio*heightAbs/2,
              -heightAbs/2, heightAbs/2, zNear, zFar);
    }
  } else
    gluPerspective(heightAngle, whRatio, zNear, zFar);
  double m[16];
  glMultMatrixd(X->getInverseAffineMatrixGL(m));
#else
  NICO
#endif
}

/// convert from gluPerspective's non-linear [0, 1] depth to the true [zNear, zFar] depth
void ors::Camera::glConvertToTrueDepth(double &d) {
  d = zNear + (zFar-zNear)*d/(zFar/zNear*(1.-d)+1.);
}

/// convert from gluPerspective's non-linear [0, 1] depth to the linear [0, 1] depth
void ors::Camera::glConvertToLinearDepth(double &d) {
  d = d/(zFar/zNear*(1.-d)+1.);
}




//===========================================================================
//
// static objects
//

OpenGL *staticgl [10]; //ten pointers to be potentially used as display windows
uint OpenGL::selectionBuffer[1000];


//===========================================================================
//
// utility implementations
//

#ifdef MT_GL
void glStandardLight(void*) {
  glEnable(GL_LIGHTING);
#if 1
  static GLfloat ambient[]   = { .5, .5, .5, 1.0 };
  static GLfloat diffuse[]   = { .2, .2, .2, 1.0 };
  static GLfloat specular[]  = { .3, .3, .3, 1.0 };
  static GLfloat position[]  = { 100.0, -100.0, 100.0, 1.0 };
  static GLfloat direction[] = { -1.0, 1.0, -1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, direction);
  glLighti(GL_LIGHT0, GL_SPOT_CUTOFF,   90);
  glLighti(GL_LIGHT0, GL_SPOT_EXPONENT, 10);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  glEnable(GL_LIGHT0);
#endif
#if 0
  static GLfloat diffuse1[]   = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat specular1[]  = { 0.1, 0.1, 0.1, 1.0 };
  static GLfloat position1[]  = { -100.0, 20.0, -100.0, 1.0 };
  static GLfloat direction1[] = { 1.0, -.2, 1.0 };
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
  glLighti(GL_LIGHT1, GL_SPOT_CUTOFF,   90);
  glLighti(GL_LIGHT1, GL_SPOT_EXPONENT, 10);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  diffuse1);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);
  glEnable(GL_LIGHT1);
#endif
}

void glStandardScene(void*) {
  glStandardLight(NULL);
//   glDrawFloor(10, .8, .8, .8);
//  glDrawFloor(10, 1.5, 0.83, .0);
  glDrawFloor(10., 108./255., 123./255., 139./255.); //Tobias' beautiful colors ;-)
}

void glColor(int col) {
  static const GLfloat colorsTab[6][4] = {
    {0.2, 0.2, 1.0, 1.0}, // blue
    {1.0, 0.8, 0.0, 1.0}, // gold
    {1.0, 0.0, 0.0, 1.0}, // red
    {0.7, 0.7, 0.7, 1.0}, // gray
    {1.0, 1.0, 1.0, 1.0}, // white
    {0.2, 1.0, 0.2, 1.0}
  }; // green
  
  if(col<0) col=0; if(col>5) col=5;
  glColor(colorsTab[col][0], colorsTab[col][1], colorsTab[col][2], colorsTab[col][3]);
}

void glColor(float r, float g, float b, float alpha) {
  float amb=1.f, diff=1.f, spec=1.f;
  GLfloat ambient[4], diffuse[4], specular[4];
  ambient[0] = r*amb;
  ambient[1] = g*amb;
  ambient[2] = b*amb;
  ambient[3] = alpha;
  diffuse[0] = r*diff;
  diffuse[1] = g*diff;
  diffuse[2] = b*diff;
  diffuse[3] = alpha;
  specular[0] = spec*.5*(1.+r);
  specular[1] = spec*.5*(1.+g);
  specular[2] = spec*.5*(1.+b);
  specular[3] = alpha;
#if 0
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
#else
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0f);
#endif
  glColor4f(r, g, b, alpha);
}

void glColor(float *rgb) { glColor(rgb[0], rgb[1], rgb[2], 1.); }

/* // shadows do not work with a light source;
   // thus, we need to leave this out. 4. Mar 06 (hh)
void glShadowTransform()
{
  GLfloat matrix[16];
  for(int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-1;  //light_x
  matrix[9]=-1;  //light_y
  matrix[14]=.02; //ground offset
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}
*/

void glTransform(const ors::Transformation& t){
  double GLmatrix[16];
  t.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
}

void glTransform(const double pos[3], const double R[12]) {
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf(matrix);
}

static GLboolean glLightIsOn = false;
void glPushLightOff() { glGetBooleanv(GL_LIGHTING, &glLightIsOn); glDisable(GL_LIGHTING); }
void glPopLight() { if(glLightIsOn) glEnable(GL_LIGHTING); }

void glDrawText(const char* txt, float x, float y, float z) {
  if(!txt) return;
#if 1 //defined MT_FREEGLUT
  glDisable(GL_DEPTH_TEST);
  glPushLightOff();
  glRasterPos3f(x, y, z);
  void *font=GLUT_BITMAP_HELVETICA_12;
  while(*txt) {
    switch(*txt) {
      case '\n':
        y+=15;
        glRasterPos3f(x, y, z);
        break;
      case '\b':
        if(font==GLUT_BITMAP_HELVETICA_12) font=GLUT_BITMAP_HELVETICA_18;
        else font=GLUT_BITMAP_HELVETICA_12;
        break;
      default:
        glutBitmapCharacter(font, *txt);
    }
    txt++;
  }
  glPopLight();
  glEnable(GL_DEPTH_TEST);
#endif
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x1, float y1, float z1, float x2, float y2, float z2,
                float x3, float y3, float z3, float x4, float y4, float z4,
                float r, float g, float b) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  glColor(r, g, b);
  glVertex3f(x1, y1, z1);
  glVertex3f(x2, y2, z2);
  glVertex3f(x3, y3, z3);
  glVertex3f(x4, y4, z4);
  glVertex3f(x1, y1, z1);
  glEnd();
}

void glDrawRect(float x, float y, float z, float r) {
  glDrawRect(x-r, y-r, z, x-r, y+r, z, x+r, y+r, z, x+r, y-r, z);
}

void glDrawFloor(float x, float r, float g, float b) {
  x/=2.;
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor(r, g, b);
  glBegin(GL_POLYGON);
  glNormal3f(0, 0, 1);
  glVertex3f(-x, -x, 0.);
  glVertex3f(x, -x, 0.);
  glVertex3f(x, x, 0.);
  glVertex3f(-x, x, 0.);
  glVertex3f(-x, -x, 0.);
  glEnd();
#if 1
  glColor(.75, .75, .75);
  for(int i=-4; i<=4; i++) {
    glBegin(GL_LINES);
    glVertex3f(i*x/5., -x, 0.001);
    glVertex3f(i*x/5., x, 0.001);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, i*x/5., 0.001);
    glVertex3f(x, i*x/5., 0.001);
    glEnd();
  }
  
  glColor(.25, .25, .25);
  glBegin(GL_LINE_STRIP);
  glVertex3f(-x, -x, 0.002);
  glVertex3f(-x, x, 0.002);
  glVertex3f(x, x, 0.002);
  glVertex3f(x, -x, 0.002);
  glVertex3f(-x, -x, 0.002);
  glEnd();
#endif
}

void glDrawBox(float x, float y, float z) {
  static GLfloat n[6][3] = {
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0}
  };
  static GLint faces[6][4] = {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  GLfloat v[8][3];
  GLint i;
  
  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -x / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] =  x / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] =  -y / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] =  y / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] =  -z / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] =  z / 2;
  
  for(i = 5; i >= 0; i--) {
    glBegin(GL_QUADS);
    glNormal3fv(&n[i][0]);
    glVertex3fv(&v[faces[i][0]][0]);
    glVertex3fv(&v[faces[i][1]][0]);
    glVertex3fv(&v[faces[i][2]][0]);
    glVertex3fv(&v[faces[i][3]][0]);
    glEnd();
  }
}

void glDrawDiamond(float x, float y, float z) {
  glDisable(GL_CULL_FACE);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(.0, .0, -z);
  glVertex3f(x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, -y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
  glEnable(GL_CULL_FACE);
}

void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz) {
  glPushMatrix();
  glTranslated(x, y, z);
  glDrawDiamond(dx, dy, dz);
  glPopMatrix();
}

void glDrawAxes(double scale) {
  GLUquadric *style=gluNewQuadric();
  
  for(uint i=0; i<3; i++) {
    glPushMatrix();
    glScalef(scale, scale, scale);
    switch(i) {
      case 0:  glColor(1, 0, 0);  break;
      case 1:  glColor(0, 1, 0);  glRotatef(90, 0, 0, 1);  break;
      case 2:  glColor(0, 0, 1);  glRotatef(90, 0, -1, 0);  break;
    }
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(.95, 0, 0);
    glEnd();
    glTranslatef(.8, 0, 0);
    glRotatef(90, 0, 1, 0);
    glDisable(GL_CULL_FACE);
    gluCylinder(style, .08, 0, .2, 20, 1);
    glEnable(GL_CULL_FACE);
    glPopMatrix();
  }
  
  gluDeleteQuadric(style);
}

void drawCoordinateFrame() {
  // x-axis = green
  glBegin(GL_LINE_STRIP);
  glColor(4.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.001);
  glVertex3f(1.0, 0.0, 0.001);
  glVertex3f(0.0, 0.0, 0.002);
  glVertex3f(1.0, 0.0, 0.002);
  glVertex3f(0.0, 0.0, 0.003);
  glVertex3f(1.0, 0.0, 0.003);
  glVertex3f(0.0, 0.0, 0.004);
  glVertex3f(1.0, 0.0, 0.004);
  glEnd();
  
  // y-axis = green
  glBegin(GL_LINE_STRIP);
  glColor(0.0, 4.0, 0.0);
  glVertex3f(0.0, 0.0, 0.001);
  glVertex3f(0.0, 1.0, 0.001);
  glVertex3f(0.0, 0.0, 0.002);
  glVertex3f(0.0, 1.0, 0.002);
  glVertex3f(0.0, 0.0, 0.003);
  glVertex3f(0.0, 1.0, 0.003);
  glVertex3f(0.0, 0.0, 0.004);
  glVertex3f(0.0, 1.0, 0.004);
  glEnd();
  
  // z-axis = blue
  glBegin(GL_LINE_STRIP);
  glColor(0.0, 0.0, 4.0);
  glVertex3f(0.001, 0.0, 0.0);
  glVertex3f(0.001, 0.0, 1.0);
  glVertex3f(0.002, 0.0, 0.0);
  glVertex3f(0.002, 0.0, 1.0);
  glVertex3f(0.003, 0.0, 0.0);
  glVertex3f(0.003, 0.0, 1.0);
  glVertex3f(0.004, 0.0, 0.0);
  glVertex3f(0.004, 0.0, 1.0);
  glEnd();
}

void glDrawSphere(float radius) {
  GLUquadric *style=gluNewQuadric();
  gluSphere(style, radius, 10, 10); // last two value for detail
  gluDeleteQuadric(style);
}

void glDrawDisk(float radius) {
  GLUquadric *style=gluNewQuadric();
  gluDisk(style, 0, radius, 10, 1);
  gluDeleteQuadric(style);
}

void glDrawCylinder(float radius, float length, bool closed) {
  GLUquadric *style=gluNewQuadric();
  glTranslatef(0, 0, -length/2);
  gluCylinder(style, radius, radius, length, 20, 1);
  if(closed) {
    glScalef(-1, 1, 1);  //flip orientation of triangles...
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, length);
    glScalef(-1, 1, 1);
    gluDisk(style, 0, radius, 20, 1);
    glTranslatef(0, 0, -length/2);
  } else {
    glTranslatef(0, 0, length/2);
  }
  gluDeleteQuadric(style);
}

void glDrawCappedCylinder(float radius, float length) {
  GLUquadric *style1=gluNewQuadric();
  GLUquadric *style2=gluNewQuadric();
  GLUquadric *style3=gluNewQuadric();
  
  glTranslatef(0, 0, -length/2);
  gluCylinder(style1, radius, radius, length, 20, 1);
  glTranslatef(0, 0, length);
  gluSphere(style2, radius, 10, 10);
  glTranslatef(0, 0, -length);
  gluSphere(style3, radius, 10, 10);
  
  gluDeleteQuadric(style1);
  gluDeleteQuadric(style2);
  gluDeleteQuadric(style3);
}

void glDrawGridBox(float x=10.) {
  x/=2.;
  glBegin(GL_LINE_LOOP);
  glVertex3f(-x, -x, -x);
  glVertex3f(-x, -x, x);
  glVertex3f(-x, x, x);
  glVertex3f(-x, x, -x);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(x, -x, -x);
  glVertex3f(x, -x, x);
  glVertex3f(x, x, x);
  glVertex3f(x, x, -x);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(x, x, x);
  glVertex3f(-x, x, x);
  glVertex3f(x, -x, x);
  glVertex3f(-x, -x, x);
  glVertex3f(x, x, -x);
  glVertex3f(-x, x, -x);
  glVertex3f(x, -x, -x);
  glVertex3f(-x, -x, -x);
  glEnd();
}

void glDrawGridBox(float x1, float y1, float z1, float x2, float y2, float z2) {
  glBegin(GL_LINE_STRIP);
  glVertex3f(x1, y1, z1+0.001);
  glVertex3f(x2, y1, z1+0.001);
  glVertex3f(x2, y2, z1+0.001);
  glVertex3f(x1, y2, z1+0.001);
  glVertex3f(x1, y1, z1+0.001);
  glEnd();
  
  glBegin(GL_LINES);
  glVertex3f(x2, y2, z1 +0.001);
  glVertex3f(x2, y2, z2 +0.001);
  glEnd();
}

void glDrawKhepera() {
  GLUquadric *style=gluNewQuadric();
  glPushMatrix();
  glRotatef(-90, 1, 0, 0);
  glColor3f(.3, .3, .3);
  gluCylinder(style, 1.5, 1.5, 2, 20, 1);
  glPopMatrix();
  
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 2, 0);
  glVertex3f(0, 2, -2.5);
  glEnd();
  gluDeleteQuadric(style);
}

void glMakeSquare(int num) {
  glNewList(num, GL_COMPILE);
  glColor3f(1., 0., 0.);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-1 , -1 , 0.);
  glVertex3f(-1 , +1 , 0.);
  glVertex3f(+1 , +1 , 0.);
  glVertex3f(+1 , -1 , 0.);
  glEnd();
  glEndList();
}

void glMakeStdSimplex(int num) {
  glNewList(num, GL_COMPILE);
  //glPolygonMode(GL_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glBegin(GL_TRIANGLE_FAN);
  glColor3f(1., 1., 1.);
  glVertex3f(0., 0., 0.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glColor3f(0., 1., 0.);
  glVertex3f(0., 1., 0.);
  glColor3f(0., 0., 1.);
  glVertex3f(0., 0., 1.);
  glColor3f(1., 0., 0.);
  glVertex3f(1., 0., 0.);
  glEnd();
  /*
    glColor4f(.5, .5, .5, .9);
    glBegin(GL_POLYGON);
    glVertex3f( 1. , 0. , 0. );
    glVertex3f( 0. , 1. , 0. );
    glVertex3f( 0. , 0. , 1. );
    glEnd();
  */
  glEndList();
}

void glMakeTorus(int num) {
  glNewList(num, GL_COMPILE);
  
  GLint i, j, rings, sides;
  float theta1, phi1, theta2, phi2;
  float v0[03], v1[3], v2[3], v3[3];
  float t0[03], t1[3], t2[3], t3[3];
  float n0[3], n1[3], n2[3], n3[3];
  float innerRadius=0.4;
  float outerRadius=0.8;
  float scalFac;
  
  rings = 8;
  sides = 10;
  scalFac=1/(outerRadius*2);
  
  for(i=0; i<rings; i++) {
    theta1 = (float)i * 2.0 * MT_PI / rings;
    theta2 = (float)(i + 1) * 2.0 * MT_PI / rings;
    for(j=0; j<sides; j++) {
      phi1 = (float)j * 2.0 * MT_PI / sides;
      phi2 = (float)(j + 1) * 2.0 * MT_PI / sides;
      
      v0[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi1));
      v0[2] = innerRadius * sin(phi1);
      
      v1[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi1));
      v1[2] = innerRadius * sin(phi1);
      
      v2[0] = cos(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[1] =-sin(theta2) * (outerRadius + innerRadius * cos(phi2));
      v2[2] = innerRadius * sin(phi2);
      
      v3[0] = cos(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[1] =-sin(theta1) * (outerRadius + innerRadius * cos(phi2));
      v3[2] = innerRadius * sin(phi2);
      
      n0[0] = cos(theta1) * (cos(phi1));
      n0[1] =-sin(theta1) * (cos(phi1));
      n0[2] = sin(phi1);
      
      n1[0] = cos(theta2) * (cos(phi1));
      n1[1] =-sin(theta2) * (cos(phi1));
      n1[2] = sin(phi1);
      
      n2[0] = cos(theta2) * (cos(phi2));
      n2[1] =-sin(theta2) * (cos(phi2));
      n2[2] = sin(phi2);
      
      n3[0] = cos(theta1) * (cos(phi2));
      n3[1] =-sin(theta1) * (cos(phi2));
      n3[2] = sin(phi2);
      
      t0[0] = v0[0]*scalFac + 0.5;
      t0[1] = v0[1]*scalFac + 0.5;
      t0[2] = v0[2]*scalFac + 0.5;
      
      t1[0] = v1[0]*scalFac + 0.5;
      t1[1] = v1[1]*scalFac + 0.5;
      t1[2] = v1[2]*scalFac + 0.5;
      
      t2[0] = v2[0]*scalFac + 0.5;
      t2[1] = v2[1]*scalFac + 0.5;
      t2[2] = v2[2]*scalFac + 0.5;
      
      t3[0] = v3[0]*scalFac + 0.5;
      t3[1] = v3[1]*scalFac + 0.5;
      t3[2] = v3[2]*scalFac + 0.5;
      
      if((i+j)%2) glColor3f(0., 1., 0.);
      else glColor3f(0., 0., 1.);
      
      glBegin(GL_POLYGON);
      glNormal3fv(n3); glTexCoord3fv(t3); glVertex3fv(v3);
      glNormal3fv(n2); glTexCoord3fv(t2); glVertex3fv(v2);
      glNormal3fv(n1); glTexCoord3fv(t1); glVertex3fv(v1);
      glNormal3fv(n0); glTexCoord3fv(t0); glVertex3fv(v0);
      glEnd();
    }
  }
  glEndList();
}

uint glImageTexture(const byteA &img) {
  GLuint texName;
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &texName);
  
  glBindTexture(GL_TEXTURE_2D, texName);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  switch(img.d2) {
    case 0:
    case 1:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
      break;
    case 2:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
      break;
    case 3:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
      break;
    case 4:
      glTexImage2D(GL_TEXTURE_2D, 0, 4, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
      break;
    default:
      HALT("no image fomat");
  }
  return texName;
}

void glDrawTexQuad(uint texture,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) {
  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //GL_MODULATE);
  glBindTexture(GL_TEXTURE_2D, texture);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0,  0.0);  glVertex3f(x1, y1, z1);
  glTexCoord2f(0.0,  mulY); glVertex3f(x2, y2, z2);
  glTexCoord2f(mulX, mulY); glVertex3f(x3, y3, z3);
  glTexCoord2f(mulX, 0.0);  glVertex3f(x4, y4, z4);
  glEnd();
  glFlush();
  glDisable(GL_TEXTURE_2D);
}

#ifdef MT_GLUT
/** @brief return the RGBA-image of scenery drawn just before; the image
  buffer has to have either 2 dimensions [width, height] for a
  gray-scale luminance image or 3 dimensions [width, height, 4] for an
  RGBA-image. */
void glGrabImage(byteA& image) {
  if(!image.N) image.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH), 3);
  CHECK(image.nd==2 || image.nd==3, "not an image format");
  GLint w=image.d1, h=image.d0;
  //CHECK(w<=glutGet(GLUT_WINDOW_WIDTH) && h<=glutGet(GLUT_WINDOW_HEIGHT), "grabbing large image from small window:" <<w <<' ' <<h <<' ' <<glutGet(GLUT_WINDOW_WIDTH) <<' ' <<glutGet(GLUT_WINDOW_HEIGHT));
  if(image.d1%4) {  //necessary: extend the image to have width mod 4
    uint add=4-(image.d1%4);
    if(image.nd==2) image.resize(image.d0, image.d1+add);
    if(image.nd==3) image.resize(image.d0, image.d1+add, image.d2);
  }
  glReadBuffer(GL_FRONT);
//  glReadBuffer(GL_BACK);

  //glPixelStorei(GL_PACK_SWAP_BYTES, 0);
  glPixelStorei(GL_PACK_ALIGNMENT,4);
  switch(image.d2) {
    case 0:
    case 1:
      glPixelTransferf(GL_RED_SCALE, .3333);
      glPixelTransferf(GL_GREEN_SCALE, .3333);
      glPixelTransferf(GL_BLUE_SCALE, .3333);
      glReadPixels(0, 0, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, image.p);
      glPixelTransferf(GL_RED_SCALE, 1.);
      glPixelTransferf(GL_GREEN_SCALE, 1.);
      glPixelTransferf(GL_BLUE_SCALE, 1.);
      break;
//    case 2:
//      //glReadPixels(0, 0, w, h, GL_GA, GL_UNSIGNED_BYTE, image.p);
//      break;
    case 3:
      glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, image.p);
    break;
    case 4:
#if defined MT_SunOS
      glReadPixels(0, 0, w, h, GL_ABGR_EXT, GL_UNSIGNED_BYTE, image.p);
#else
#if defined MT_Cygwin
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#else
      //glReadPixels(0, 0, w, h, GL_BGRA_EXT, GL_UNSIGNED_BYTE, image.p);
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.p);
#endif
#endif
      break;
    default: HALT("wrong image format");
  }
}
#else
void glGrabImage(byteA& image) { NICO }
#endif

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(byteA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK(depth.nd==2, "depth buffer has to be either 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, depth.p);
}

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(floatA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK(depth.nd==2, "depth buffer has to be 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth.p);
}

void glRasterImage(float x, float y, byteA &img, float zoom) {
  glRasterPos3f(x, y, 0.); //(int)(y+zoom*img.d0)); (the latter was necessary for other pixel/raster coordinates)
  glPixelZoom(zoom, -zoom);
  if(img.d1%4) {  //necessary: extend the image to have width mod 4
    uint P=img.d2;
    if(!P) P=1;
    uint add=4-(img.d1%4);
    img.reshape(img.d0, img.d1*P);
    img.insColumns(img.d1, add*P);
    if(P>1) img.reshape(img.d0, img.d1/P, P);
  }
  
  switch(img.d2) {
    case 0:
    case 1:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);        break;
    case 2:  glDrawPixels(img.d1, img.d0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);  break;
    case 3:  glDrawPixels(img.d1, img.d0, GL_RGB, GL_UNSIGNED_BYTE, img.p);              break;
    case 4:  glDrawPixels(img.d1, img.d0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);             break;
    default: HALT("no image format");
  };
}

void OpenGL::watchImage(const floatA &_img, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, _img);
  float x;
  for(uint i=0; i<img.N; i++) {
    x=_img.elem(i);
    img.elem(i) = (x<0.)?0:((x>255.)?255:x);
  }
  watchImage(img, wait, _zoom);
}

void OpenGL::watchImage(const byteA &_img, bool wait, float _zoom) {
  background=_img;
  backgroundZoom=_zoom;
  //resize(img->d1*zoom,img->d0*zoom);
  if(wait) watch(); else update();
}

/*void glWatchImage(const floatA &x, bool wait, float zoom){
  double ma=x.max();
  double mi=x.min();
  if(wait) cout <<"watched image min/max = " <<mi <<' ' <<ma <<endl;
  byteA img;
  img.resize(x.d0*x.d1);
  img.setZero();
  for(uint i=0;i<x.N;i++){
    img(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  img.reshape(x.d0, x.d1);
  glWatchImage(img, wait, 20);
}*/

void OpenGL::displayGrey(const arr &x, bool wait, float _zoom) {
  static byteA img;
  resizeAs(img, x);
  double mi=x.min(), ma=x.max();
  text.clear() <<"displayGrey" <<" max=" <<ma <<"min=" <<mi <<endl;
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  watchImage(img, wait, _zoom);
}

void OpenGL::displayRedBlue(const arr &x, bool wait, float _zoom) {
  double mi=x.min(), ma=x.max();
  text.clear() <<"max=" <<ma <<"min=" <<mi <<endl;
//  cout <<"\rdisplay" <<win <<" max=" <<ma <<"min=" <<mi;
  static byteA img;
  img.resize(x.d0*x.d1, 3);
  img.setZero();
  for(uint i=0; i<x.N; i++) {
    if(x.elem(i)>0.) img(i, 0)=(byte)(255.*x.elem(i)/ma);
    if(x.elem(i)<0.) img(i, 2)=(byte)(255.*x.elem(i)/mi);
  }
  img.reshape(x.d0, x.d1, 3);
  watchImage(img, wait, _zoom);
}

void glDrawUI(void *p) {
  glPushName(0x10);
  ((glUI*)p)->glDraw();
  glPopName();
}

bool glUI::hoverCallback(OpenGL& gl) {
  //bool b=
  checkMouse(gl.mouseposx, gl.mouseposy);
  //if(b) postRedrawEvent(false);
  return true;
}

bool glUI::clickCallback(OpenGL& gl) {
  bool b=checkMouse(gl.mouseposx, gl.mouseposy);
  if(b) gl.postRedrawEvent(true);
  int t=top;
  if(t!=-1) {
    cout <<"CLICK! on button #" <<t <<endl;
    gl.exitEventLoop();
    return false;
  }
  return true;
}

#ifdef MT_FREEGLUT
void glSelectWin(uint win) {
  if(!staticgl[win]) staticgl[win]=new OpenGL;
  glutSetWindow(staticgl[win]->s->windowID);
}
#endif

#else /// MT_GL
void glColor(int col) { NICO }
void glColor(float, float, float, float) { NICO }
void glDrawDiamond(float, float, float, float, float, float) { NICO }
// void glStandardLight(void*) { NICO }   // TOBIAS: das hier wird doch schon ueber opengl_void.cxx definiert
void glStandardScene(void*) { NICO }
uint glImageTexture(const byteA &img) { NICO }
void glDrawTexQuad(uint texture,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) { NICO }
void OpenGL::watchImage(const floatA &_img, bool wait, float _zoom) { NICO }
void OpenGL::watchImage(const byteA &_img, bool wait, float _zoom) { NICO }
void glDrawUI(void *p) { NICO }
bool glUI::hoverCallback(OpenGL& gl) { NICO }
bool glUI::clickCallback(OpenGL& gl) { NICO }
#endif

//===========================================================================
//
// standalone draw routines for large data structures
//

#ifdef MT_GL
void glDrawDots(void *dots) { glDrawPointCloud(*(arr*)dots, NoArr); }
void glDrawPointCloud(void *pc) { glDrawPointCloud(((const arr*)pc)[0], ((const arr*)pc)[1]); }

void glDrawPointCloud(const arr& pts, const arr& cols) {
  if(!pts.N) return;
  CHECK(pts.nd==2 && pts.d1==3, "wrong dimension");
  glDisable(GL_LIGHTING);
#if 0
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_DOUBLE, pts.d1-3, pts.p);
  if(&cols && cols.N==pts.N){
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_DOUBLE, cols.d1-3, cols.p );
  }else glDisableClientState(GL_COLOR_ARRAY);
  glDrawArrays(GL_POINTS, 0, pts.d0);
  glDisableClientState(GL_VERTEX_ARRAY);
#else
  glBegin(GL_POINTS);
  if(!&cols || cols.N!=pts.N){
    const double *p=pts.begin(), *pstop=pts.end();
    for(; p!=pstop; p+=pts.d1)
      glVertex3dv(p);
  }else{
    const double *p=pts.begin(), *pstop=pts.end(), *c=cols.begin();
    for(; p!=pstop; p+=pts.d1, c+=cols.d1){
      glVertex3dv(p);
      glColor3dv(c);
    }
  }
  glEnd();
#endif
}
#endif

//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* title,int w,int h,int posx,int posy)
  : s(NULL), reportEvents(false), width(0), height(0), captureImg(false), captureDep(false), fboId(0), rboColor(0), rboDepth(0){
  //MT_MSG("creating OpenGL=" <<this);
  initGlEngine();
  s=new sOpenGL(this,title,w,h,posx,posy); //this might call some callbacks (Reshape/Draw) already!
  init();
  processEvents();
}

OpenGL::OpenGL(void *container)
  : s(NULL), reportEvents(false), width(0), height(0), captureImg(false), captureDep(false), fboId(0), rboColor(0), rboDepth(0){
  initGlEngine();
  s=new sOpenGL(this,container); //this might call some callbacks (Reshape/Draw) already!
  init();
  processEvents();
}

OpenGL::~OpenGL() {
  delete s;
  s=NULL;
//  MT_MSG("destructing OpenGL=" <<this);
}

OpenGL* OpenGL::newClone() const {
  OpenGL* gl=new OpenGL;
  //*(gl->s) = *s; //don't clone internal stuff!
  gl->drawers = drawers;
  gl->camera = camera;
  return gl;
}

void OpenGL::init() {
  CHECK(width && height,"width and height are not set -- perhaps not initialized");
  camera.setPosition(0., 0., 10.);
  camera.focus(0, 0, 0);
  camera.setZRange(.1, 1000.);
  camera.setHeightAngle(12.);
  
  drawFocus=false;
  clearR=clearG=clearB=1.; clearA=0.;
  drawers.memMove=true;
  mouseposx=mouseposy=0;
  mouse_button=0;
  mouseIsDown=false;
  mouseView=-1;
  
  reportEvents=false;
  reportSelects=false;
  immediateExitLoop=false;
  exitkeys="";
  
  backgroundZoom=1;
};

/// add a draw routine
void OpenGL::add(void (*call)(void*), const void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  GLDrawer d; d.classP=(void*)classP; d.call=call;
  drawers.append(d);
}

void OpenGL::addView(uint v, void (*call)(void*), const void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  GLDrawer d; d.classP=(void*)classP; d.call=call;
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).drawers.append(d);
}

void OpenGL::setViewPort(uint v, double l, double r, double b, double t) {
  if(v>=views.N) views.resizeCopy(v+1);
  views(v).le=l;  views(v).ri=r;  views(v).bo=b;  views(v).to=t;
}

/// remove a draw routine
void OpenGL::remove(void (*call)(void*), const void* classP) {
  CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  uint i;
  for(i=0; i<drawers.N; i++) if(drawers(i).call==call && drawers(i).classP==classP) break;
  CHECK(i<drawers.N, "value to remove not found");
  drawers.remove(i, 1);
}

/// clear the list of all draw routines
void OpenGL::clear() {
  views.clear();
  drawers.clear();
  initCalls.clear();
  hoverCalls.clear();
  clickCalls.clear();
  keyCalls.clear();
}

/// add a hover callback
void OpenGL::addHoverCall(GLHoverCall *c) {
  //CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  //GLHoverCall c; c.classP=(void*)classP; c.call=call;
  hoverCalls.append(c);
}

/// clear hover callbacks
void OpenGL::clearHoverCalls() {
  hoverCalls.clear();
}

/// add a click callback
void OpenGL::addClickCall(GLClickCall *c) {
  //CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  //GLClickCall c; c.classP=(void*)classP; c.call=call;
  clickCalls.append(c);
}

/// clear click callbacks
void OpenGL::clearClickCalls() {
  clickCalls.clear();
}

/// add a click callback
void OpenGL::addKeyCall(GLKeyCall *c) {
  //CHECK(call!=0, "OpenGL: NULL pointer to drawing routine");
  //GLKeyCall c; c.classP=(void*)classP; c.call=call;
  keyCalls.append(c);
}

/// clear click callbacks
void OpenGL::clearKeyCalls() {
  keyCalls.clear();
}

void OpenGL::Draw(int w, int h, ors::Camera *cam) {
#ifdef MT_GL
  openglAccess().lock();

  //clear bufferer
  GLint viewport[4] = {0, 0, w, h};
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  glClearColor(clearR, clearG, clearB, clearA);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  //raster an image as background
  if(background.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glOrtho(0, 1., 1., 0, -1., 1.); //only affects the offset - the rest is done with raster zooms
    glDisable(GL_DEPTH_TEST);
    glRasterImage(0, 0, background, backgroundZoom); //.5*w/background.d1);
  }
  
  //OpenGL initialization
  glEnable(GL_DEPTH_TEST);  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);  glFrontFace(GL_CCW);
  glShadeModel(GL_FLAT);  //glShadeModel(GL_SMOOTH);

  //select mode?
  GLint mode;
  glGetIntegerv(GL_RENDER_MODE, &mode);
  
  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mode==GL_SELECT) gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
  if(!cam) camera.glSetProjectionMatrix();
  else     cam->glSetProjectionMatrix();
  //glLineWidth(2);
  
  //**extract the camera projection matrix
#if 0
  //this is the calibration matrix corresponding to OpenGL's ``viewport''
  intA view(4);
  arr Kview(3, 3);
  glGetIntegerv(GL_VIEWPORT, view.p);
  Kview.setZero();
  Kview(0, 0) = .5*view(2); Kview(0, 2) = view(0)+.5*view(2);
  Kview(1, 1) = .5*view(3); Kview(1, 2) = view(1)+.5*view(3);
  Kview(2, 2) = 1.;  //OpenGL's w coordinate is the negative of what we want...
  //the projection matrix (without viewport-calibration) from OpenGL:
  P.resize(4, 4);
  glGetDoublev(GL_PROJECTION_MATRIX, P.p);
  //cout <<"OpenGL's glP=" <<P <<"\nK=" <<Kview <<endl;
  //double sca=P.elem(0);
  P = ~P;      //OpenGL uses transposed matrix storage convention
  P.delRows(2); //We're not interested in OpenGL's ``z-culling-coordinate'', only in the perspective coordinate (divisor) w
  //P[2]() *=-1.;
  //the full camera projection matrix:
  P = Kview*P;
  //cout <<"OpenGL's P=" <<P <<endl;
  
  /*
  double zn=camera.zNear, zf=camera.zFar, f=1./tan(MT_PI/180.*camera.heightAngle/2.);
  arr Frust(4, 4); Frust.setZero();
  Frust(0, 0) = Frust(1, 1) = f;
  Frust(2, 2) = (zf+zn)/(zn-zf);
  Frust(3, 2) = -1.;
  Frust(2, 3) = 2.*zf*zn/(zn-zf);
  cout <<"OpenGL P=" <<P <<"K=" <<Kview <<"znear=" <<camera.zNear <<"zfar=" <<camera.zFar <<endl;
  cout <<"Frust=" <<Frust <<endl;;
  Frust.delRows(2); //We're not interested in OpenGL's ``z-coordinate'', only in the perspective coordinate (divisor) w
  cout <<"K=" <<Kview*Frust <<endl;
  */
#endif

  //draw focus?
  if(drawFocus && mode!=GL_SELECT) {
    glColor(1., .7, .3);
    double size = .005 * (camera.X->pos-*camera.foc).length();
    glDrawDiamond((*camera.foc).x, (*camera.foc).y, (*camera.foc).z, size, size, size);
  }
  /*if(topSelection && mode!=GL_SELECT){
    glColor(1., .7, .3);
    double size = .005 * (camera.X->pos-*camera.foc).length();
    glDrawDiamond(topSelection->x, topSelection->y, topSelection->z, size, size, size);
  }*/
  
  //std color: black:
  glColor(.3, .3, .5);
  
  lock.readLock(); //now accessing user data
  //cout <<"LOCK draw" <<endl;

  //draw central view
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if(mode==GL_SELECT) glInitNames();
  for(uint i=0; i<drawers.N; i++) {
    if(mode==GL_SELECT) glLoadName(i);
    (*drawers(i).call)(drawers(i).classP);
    glLoadIdentity();
  }

  //draw text
  if(text.N) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glOrtho(0., (double)w, (double)h, .0, -1., 1.);
    glDrawText(text, 10, 20, 0);
    glLoadIdentity();
  }
  
  //draw subviews
  for(uint v=0; v<views.N; v++) {
    GLView *vi=&views(v);
    glViewport(vi->le*w, vi->bo*h, (vi->ri-vi->le)*w, (vi->to-vi->bo)*h);
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(vi->img) {
      glDisable(GL_DEPTH_TEST);
      glRasterImage(-1., 1., *vi->img, backgroundZoom*(vi->ri-vi->le)*w/vi->img->d1);
      glEnable(GL_DEPTH_TEST);
    }
    vi->camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if(drawFocus) {
      glColor(1., .7, .3);
      double size = .005 * (camera.X->pos-*camera.foc).length();
      glDrawDiamond((*vi->camera.foc).x, (*vi->camera.foc).y, (*vi->camera.foc).z, size, size, size);
    }
    for(uint i=0; i<vi->drawers.N; i++)(*vi->drawers(i).call)(vi->drawers(i).classP);
    if(vi->text.N) {
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      if(clearR+clearG+clearB>1.) glColor(0.0, 0.0, 0.0, 1.0); else glColor(1.0, 1.0, 1.0, 1.0);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      //glOrtho(0., (vi->ri-vi->le)*w, (vi->to-vi->bo)*h, .0, -1., 1.);
      glDrawText(vi->text, -.95, .85, 0.);
    }
  }
  
  //cout <<"UNLOCK draw" <<endl;
  lock.unlock(); //now de-accessing user data

  if(captureImg){
    captureImage.resize(h, w, 3);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);
    captureImg=false;
  }
  if(captureDep){
    captureDepth.resize(h, w);
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, captureDepth.p);
    captureDep=false;
  }

  //check matrix stack
  GLint s;
  glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &s);
  if(s!=1) MT_MSG("OpenGL name stack has not depth 1 (pushs>pops) in DRAW mode:" <<s);
  //CHECK(s<=1, "OpenGL matrix stack has not depth 1 (pushs>pops)");
  
  //this->s->endGlContext();
  openglAccess().unlock();
#endif
}

void OpenGL::Select() {
#ifdef MT_GL
  uint i, j, k;
  
  s->beginGlContext();
  
  glSelectBuffer(1000, selectionBuffer);
  glRenderMode(GL_SELECT);
  
#if 1
  GLint w=width, h=height;
  
  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if(mouseView==-1) {
    GLint viewport[4] = {0, 0, w, h};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    camera.glSetProjectionMatrix();
  } else {
    GLView *vi=&views(mouseView);
    GLint viewport[4] = { (GLint)vi->le*w, (GLint)vi->bo*h, (GLint)(vi->ri-vi->le)*w, (GLint)(vi->to-vi->bo)*h};
    gluPickMatrix((GLdouble)mouseposx, (GLdouble)mouseposy, 2., 2., viewport);
    vi->camera.glSetProjectionMatrix();
  }
  
  lock.readLock(); //now accessing user data
  //cout <<"LOCK select" <<endl;

  //draw objects
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glInitNames();
  if(mouseView==-1) {
    for(i=0; i<drawers.N; i++) {
      glLoadName(i);
      (*drawers(i).call)(drawers(i).classP);
      GLint s;
      glGetIntegerv(GL_NAME_STACK_DEPTH, &s);
      if(s!=0) MT_MSG("OpenGL name stack has not depth 1 (pushs>pops) in SELECT mode:" <<s);
    }
  } else {
    GLView *vi=&views(mouseView);
    for(i=0; i<vi->drawers.N; i++) { glLoadName(i); (*vi->drawers(i).call)(vi->drawers(i).classP); }
  }

  //cout <<"UNLOCK select" <<endl;
  lock.unlock(); //deaccess user data

#else
  Draw(width(),height());
#endif
  glLoadIdentity();
  
  GLint n;
  n=glRenderMode(GL_RENDER);
  selection.resize(n);
  
  GLuint *obj, maxD=(GLuint)(-1);
  topSelection=NULL;
  for(j=0, i=0; i<(uint)n; i++) {
    obj=selectionBuffer+j;
    j+=3+obj[0];
    
    //get name as superposition of all names
    selection(i).name = 0;
    for(k=0; k<obj[0]; k++) selection(i).name |= obj[3+k];
    
    //get dim and dmax
    selection(i).dmin=(double)obj[1]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmin);
    selection(i).dmax=(double)obj[2]/maxD;  //camera.glConvertToTrueDepth(selection(i).dmax);
    
    //get top-most selection
    if(!topSelection || selection(i).dmin < topSelection->dmin) topSelection = &selection(i);
  }
  
  if(topSelection) {
    topSelection->x=0; topSelection->y=0; topSelection->z=topSelection->dmin;
    unproject(topSelection->x, topSelection->y, topSelection->z);
  }
  
  if(reportSelects) reportSelection();
  
  s->endGlContext();
#endif
}

/** @brief watch in interactive mode and wait for an exiting event
  (key pressed or right mouse) */
int OpenGL::watch(const char *txt) {
  update(STRING(txt<<" - press ENTER to continue"));
  if(MT::getInteractivity()) enterEventLoop();
  return pressedkey;
}

/// update the view (in Qt: also starts displaying the window)
int OpenGL::update(const char *txt, bool _captureImg, bool _captureDep, bool waitForCompletedDraw) {
  captureImg=_captureImg;
  captureDep=_captureDep;
  if(txt) text.clear() <<txt;
  isUpdating.waitForValueEq(0);
  isUpdating.setValue(1);
  postRedrawEvent(false);
  if(captureImg || captureDep || waitForCompletedDraw){ processEvents();  isUpdating.waitForValueEq(0);  processEvents(); }//{ MT::wait(.01); processEvents(); MT::wait(.01); }
  return pressedkey;
}

/// waits some msecons before updating
int OpenGL::timedupdate(double sec) {
  static double lasttime=-1;
  double now;
  now=MT::realTime();
  if(lasttime>0. && now-lasttime<sec) MT::wait(lasttime+sec-now);
  lasttime=now;
  return update();
#if 0//def MT_QTGL
  int i;
  quitLoopOnTimer=true;
  i=startTimer(msec);
  enterEventLoop();
  killTimer(i);
  return update();
#endif
}

/// set the four clear colors
void OpenGL::setClearColors(float r, float g, float b, float a) {
  clearR=r; clearG=g; clearB=b; clearA=a;
}

/** @brief inverse projection: given a 2D+depth coordinates in the
  camera view (e.g. as a result of selection) computes the world 3D
  coordinates */
void OpenGL::unproject(double &x, double &y, double &z,bool resetCamera) {
#ifdef MT_GL
  double _x, _y, _z;
  GLdouble modelMatrix[16], projMatrix[16];
  GLint viewPort[4];
  if(resetCamera) {
    GLint viewport[4] = {0, 0, (GLint)width, (GLint)height};
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    camera.glSetProjectionMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
  glGetIntegerv(GL_VIEWPORT, viewPort);
  gluUnProject(x, y, z, modelMatrix, projMatrix, viewPort, &_x, &_y, &_z);
  x=_x; y=_y; z=_z;
#else
  NICO
#endif
}


//void OpenGL::capture(byteA &img, int w, int h, ors::Camera *cam) {
////#ifdef MT_GLUT
////#ifdef MT_FREEGLUT
////  glutSetWindow(s->windowID);
////#elif defined MT_GTKGL
////  gtkLock();
////#endif
//  captureImg=true;
//  postRedrawEvent(false);
//  processEvents();
//  captureImg=false;
////  if(w==-1) w=width;
////  if(h==-1) h=height;
////  Draw(w, h, cam);
////  img.resize(h, w, 3);
////#if 1
////  glutSwapBuffers();
////  glGrabImage(img);
////#else
////  XImage *image = XGetImage(xdisplay(), xdraw(), 0, 0, w, h, 0, XYPixmap);
////  for (int x = 0; x < w; x++){
////    for (int y = 0; y < h; y++){
////       img(x,y,0) = (XGetPixel(image,x,y) & image->red_mask) >> 16;
////       img(x,y,1) = (XGetPixel(image,x,y) & image->green_mask) >> 8;
////       img(x,y,2) = (XGetPixel(image,x,y) & image->blue_mask);
////    }
////  }
////#endif
////#if defined MT_GTKGL
////  gtkUnlock();
////#endif
////#endif
//}

//void OpenGL::captureDepth(byteA &depth, int w, int h, ors::Camera *cam) {
//#ifdef MT_GLUT
//#ifdef MT_FREEGLUT
//  glutSetWindow(s->windowID);
//#endif
//  //postRedrawEvent(false);
//  //processEvents();
//  Draw(w, h, cam);
//  depth.resize(h, w);
//  glGrabDepth(depth);
//#endif
//}

//void OpenGL::captureDepth(floatA &depth, int w, int h, ors::Camera *cam) {
//#ifdef MT_GLUT
//#ifdef MT_FREEGLUT
//  glutSetWindow(s->windowID);
//#endif
//  //postRedrawEvent(false);
//  //processEvents();
//  Draw(w, h, cam);
//  depth.resize(h, w);
//  glGrabDepth(depth);
//#endif
//}

//void OpenGL::captureStereo(byteA &imgL, byteA &imgR, int w, int h, ors::Camera *cam, double baseline) {
//#ifdef MT_GLUT
//#ifdef MT_FREEGLUT
//  glutSetWindow(s->windowID);
//#endif
//  postRedrawEvent(false);
//  processEvents();
//  Draw(w, h, cam);
//  imgR.resize(h, w, 3);
//  glGrabImage(imgR);
//  double xorg=cam->X->pos.x;
//  cam->X->pos.x -= baseline;
//  Draw(w, h, cam);
//  imgL.resize(h, w, 3);
//  glGrabImage(imgL);
//  cam->X->pos.x = xorg;
//#endif
//}



/// print some info on the selection buffer
void OpenGL::reportSelection() {
  uint i;
  std::cout <<"selection report: mouse=" <<mouseposx <<" " <<mouseposy <<" -> #selections=" <<selection.N <<std::endl;
  for(i=0; i<selection.N; i++) {
    if(topSelection == &selection(i)) std::cout <<"  TOP: "; else std::cout <<"       ";
    std::cout
        <<"name = 0x" <<std::hex <<selection(i).name <<std::dec
        <<" min-depth:" <<selection(i).dmin <<" max-depth:" <<selection(i).dmax
        <<" 3D: (" <<selection(i).x <<',' <<selection(i).y <<',' <<selection(i).z <<')'
        <<endl;
  }
}

#ifdef MT_GL2PS
/** @brief generates a ps from the current OpenGL display, using gl2ps */
void OpenGL::saveEPS(const char *filename) {
  FILE *fp = fopen(filename, "wb");
  GLint buffsize = 0, state = GL2PS_OVERFLOW;
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  while(state==GL2PS_OVERFLOW) {
    buffsize+=1024*1024;
    gl2psBeginPage("Marc Toussaint", "MT", viewport,
                   GL2PS_EPS, GL2PS_BSP_SORT, GL2PS_SILENT |
                   GL2PS_SIMPLE_LINE_OFFSET | GL2PS_NO_BLENDING |
                   GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT,
                   GL_RGBA, 0, NULL, 0, 0, 0, buffsize,
                   fp, filename);
    Draw(width(), height());
    state = gl2psEndPage();
  }
  fclose(fp);
}
#else
void OpenGL::saveEPS(const char*) {
  MT_MSG("WARNING: OpenGL::saveEPS was called without MT_GL2PS configured!");
}
#endif

#ifndef MT_QTGL
/** @brief report on the OpenGL capabilities (the QGLFormat) */
void OpenGL::about(std::ostream& os) { MT_MSG("NICO"); }
#endif


//===========================================================================
//
// callbacks
//

#if 1
#  define CALLBACK_DEBUG(x) if(reportEvents) { cout <<MT_HERE <<s <<':'; x; }
#else
#  define CALLBACK_DEBUG(x)
#endif

void getSphereVector(ors::Vector& vec, int _x, int _y, int le, int ri, int bo, int to) {
  int w=ri-le, h=to-bo;
  int minwh = w<h?w:h;
  double x, y;
  x=(double)_x;  x=x-le-.5*w;   x*= 2./minwh;
  y=(double)_y;  y=y-bo-.5*h; y*= 2./minwh;
  vec.x=x;
  vec.y=y;
  vec.z=.5-(x*x+y*y);
  if(vec.z<0.) vec.z=0.;
}

void OpenGL::Reshape(int _width, int _height) {
  CALLBACK_DEBUG(printf("Window %d Reshape Callback:  %d %d\n", 0, _width, _height));
  width=_width;
  height=_height;
  if(width%8) width = 8*(width/8);
  if(height%2) height = 2*(height/2);
  camera.setWHRatio((double)width/height);
  for(uint v=0; v<views.N; v++) views(v).camera.setWHRatio((views(v).ri-views(v).le)*width/((views(v).to-views(v).bo)*height));
  //postRedrawEvent(true);
}

void OpenGL::Key(unsigned char key, int _x, int _y) {
  _y = height-_y;
  CALLBACK_DEBUG(printf("Window %d Keyboard Callback:  %d (`%c') %d %d\n", 0, key, (char)key, _x, _y));
  mouseposx=_x; mouseposy=_y;
  pressedkey=key;
  
  bool cont=true;
  for(uint i=0; i<keyCalls.N; i++) cont=cont && keyCalls(i)->keyCallback(*this);
  
  if(key==13 || key==27 || MT::contains(exitkeys, key)) exitEventLoop();
}

void OpenGL::Mouse(int button, int downPressed, int _x, int _y) {
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG(printf("Window %d Mouse Click Callback:  %d %d %d %d\n", 0, button, downPressed, _x, _y));
  mouse_button=1+button;
  if(downPressed) mouse_button=-1-mouse_button;
  mouseposx=_x; mouseposy=_y;
  lastEvent.set(mouse_button, -1, _x, _y, 0., 0.);
  
  GLView *v;
  ors::Camera *cam=&camera;
  ors::Vector vec;
  for(mouseView=views.N; mouseView--;) {
    v=&views(mouseView);
    if(_x<v->ri*w && _x>v->le*w && _y<v->to*h && _y>v->bo*h) {
      getSphereVector(vec, _x, _y, v->le*w, v->ri*w, v->bo*h, v->to*h);
      cam=&views(mouseView).camera;
      break;
    }
  }
  if(mouseView==-1) getSphereVector(vec, _x, _y, 0, w, 0, h);
  CALLBACK_DEBUG(cout <<"associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);
  
  if(!downPressed) {  //down press
    if(mouseIsDown) return;  //the button is already down (another button was pressed...)
    //CHECK(!mouseIsDown, "I thought the mouse is up...");
    mouseIsDown=true;
    drawFocus=true;
  } else {
    if(!mouseIsDown) return;  //the button is already up (another button was pressed...)
    //CHECK(mouseIsDown, "mouse-up event although the mouse is not down???");
    mouseIsDown=false;
    drawFocus=false;
  }
  //store where you've clicked
  s->downVec=vec;
  s->downRot=cam->X->rot;
  s->downPos=cam->X->pos;
  s->downFoc=*cam->foc;
  
  //check object clicked on
  if(!downPressed) {
    if(reportSelects) Select();
  }
  //step through all callbacks
  bool cont=true;
  if(!downPressed) {
    for(uint i=0; i<clickCalls.N; i++) cont=cont && clickCalls(i)->clickCallback(*this);
  }
  if(!cont) { postRedrawEvent(true); return; }
  
  //mouse scroll wheel:
  if(mouse_button==4 && !downPressed) cam->X->pos += s->downRot*Vector_z * (.2 * s->downPos.length());
  if(mouse_button==5 && !downPressed) cam->X->pos -= s->downRot*Vector_z * (.2 * s->downPos.length());
  
  if(mouse_button==3) {  //selection
    Select();
    if(topSelection)
      cam->focus(topSelection->x, topSelection->y, topSelection->z);
  }
  postRedrawEvent(true);
}

void OpenGL::MouseWheel(int wheel, int direction, int x, int y) {
  CALLBACK_DEBUG(printf("Window %d Mouse Wheel Callback:  %d %d %d %d\n", 0, wheel, direction, x, y));
  if(direction>0) camera.X->pos += camera.X->rot*Vector_z * (.1 * (camera.X->pos-*camera.foc).length());
  else            camera.X->pos -= camera.X->rot*Vector_z * (.1 * (camera.X->pos-*camera.foc).length());
  postRedrawEvent(true);
}


void OpenGL::Motion(int _x, int _y) {
#ifdef MT_GL
  int w=width, h=height;
  _y = h-_y;
  CALLBACK_DEBUG(printf("Window %d Mouse Motion Callback:  %d %d\n", 0, _x, _y));
  mouseposx=_x; mouseposy=_y;
  ors::Camera *cam;
  ors::Vector vec;
  if(mouseView==-1) {
    cam=&camera;
    getSphereVector(vec, _x, _y, 0, w, 0, h);
  } else {
    cam=&views(mouseView).camera;
    getSphereVector(vec, _x, _y, views(mouseView).le*w, views(mouseView).ri*w, views(mouseView).bo*h, views(mouseView).to*h);
  }
  CALLBACK_DEBUG(cout <<"associated to view " <<mouseView <<" x=" <<vec.x <<" y=" <<vec.y <<endl);
  lastEvent.set(mouse_button, -1, _x, _y, vec.x-s->downVec.x, vec.y-s->downVec.y);
#ifndef MT_Linux
  int modifiers=glutGetModifiers();
#else
  //int modifiers=0;
#endif
  if(!mouseIsDown) {  //passive motion -> hover callbacks
    mouseposx=_x; mouseposy=_y;
    bool ud=false;
    for(uint i=0; i<hoverCalls.N; i++) ud=ud || hoverCalls(i)->hoverCallback(*this);
    if(ud) postRedrawEvent(true);
    return;
  }
  //CHECK(mouseIsDown, "I thought the mouse is down...");
  if(mouse_button==1) {  //rotation // && !(modifiers&GLUT_ACTIVE_SHIFT) && !(modifiers&GLUT_ACTIVE_CTRL)){
    ors::Quaternion rot;
    if(s->downVec.z<.1) {
      rot.setDiff(vec, s->downVec);  //consider imagined sphere rotation of mouse-move
    } else {
      rot.setVec((vec-s->downVec) ^ Vector_z); //consider only xy-mouse-move
    }
#if 0 //rotate about origin
    cam->X->rot = s->downRot * rot;   //rotate camera's direction
    rot = s->downRot * rot / s->downRot; //interpret rotation relative to current viewing
    cam->X->pos = rot * s->downPos;   //rotate camera's position
#else //rotate about focus
    cam->X->rot = s->downRot * rot;   //rotate camera's direction
    rot = s->downRot * rot / s->downRot; //interpret rotation relative to current viewing
    cam->X->pos = s->downFoc + rot * (s->downPos - s->downFoc);   //rotate camera's position
    //cam->focus();
#endif
    postRedrawEvent(true);
    if(immediateExitLoop) exitEventLoop();
  }
  if(mouse_button==3) {  //translation || (mouse_button==1 && (modifiers&GLUT_ACTIVE_SHIFT) && !(modifiers&GLUT_ACTIVE_CTRL))){
    /*    ors::Vector trans = s->downVec - vec;
        trans.z=0.;
        trans = s->downRot*trans;
        cam->X->pos = s->downPos + trans;
        postRedrawEvent(true);*/
  }
  if(mouse_button==2) {  //zooming || (mouse_button==1 && !(modifiers&GLUT_ACTIVE_SHIFT) && (modifiers&GLUT_ACTIVE_CTRL))){
    double dy = s->downVec.y - vec.y;
    if(dy<-.99) dy = -.99;
    cam->X->pos = s->downPos + s->downRot*Vector_z * dy * s->downPos.length();
    postRedrawEvent(true);
  }
#else
  NICO
#endif
}

//===========================================================================
//
// GUI implementation
//

void glUI::addButton(uint x, uint y, const char *name, const char *img1, const char *img2) {
  Button &b = buttons.append();
  byteA img;
  b.hover=false;
  b.x=x; b.y=y; b.name=name;
  //read_png(img, tex1);
  if(img1) {
    read_ppm(img, img1, true);
  } else {
    img.resize(18, strlen(name)*9+10, 3);
    img=255;
  }
  b.w=img.d1; b.h=img.d0;
  b.img1=img;    add_alpha_channel(b.img1, 100);
  if(img2) {
    read_ppm(img, img1, true);
    CHECK(img.d1==b.w && img.d0==b.h, "mismatched size");
  }
  b.img2=img;    add_alpha_channel(b.img2, 200);
}

void glUI::glDraw() {
#ifdef MT_GL
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLint viewPort[4];
  glGetIntegerv(GL_VIEWPORT, viewPort);
  glOrtho(0., viewPort[2], viewPort[3], .0, -1., 1.);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  Button *b;
  float x, y, h;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    //w=b->w;
    h=b->h;
    glColor(0, 0, 0, 1);
    glDrawText(b->name, x+5, y+h-5, 0.);
    if((int)i==top) glRasterImage((int)x, (int)y, b->img2);
    else      glRasterImage((int)x, (int)y, b->img1);
  }
#else
  NICO
#endif
}

bool glUI::checkMouse(int _x, int _y) {
  float x, y, w, h;
  Button *b;
  int otop=top;
  top=-1;
  for(uint i=0; i<buttons.N; i++) {
    b = &buttons(i);
    x=b->x-b->w/2.;
    y=b->y-b->h/2.;
    w=b->w;
    h=b->h;
    if(_x>=x && _x <=x+w && _y>=y && _y<=y+h) top = i;
  }
  if(otop==top) return false;
  //postRedrawEvent(false);
  return true;
}

#ifdef MT_QTGL
#if   defined MT_MSVC
#  include"opengl_MSVC.moccpp"
#elif defined MT_SunOS
#  include"opengl_SunOS.moccpp"
#elif defined MT_Linux
#  include"opengl_qt_moc.cxx"
#elif defined MT_Cygwin
#  include"opengl_Cygwin.moccpp"
#endif
#endif
