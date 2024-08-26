void glStandardLight(void*, OpenGL& gl) {
  glEnable(GL_LIGHTING);
  if(!gl.drawOptions.enableLighting) {
    glDisable(GL_LIGHTING);
    return;
  }
  static GLfloat ambient[]   = { .5, .5, .5, 1.0 };
  static GLfloat diffuse[]   = { .3, .3, .3, 1.0 };
  static GLfloat specular[]  = { .4, .4, .4, 1.0 };
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
  {
    static GLfloat ambient2[]   = { .0, .0, .0, 1.0 };
    static GLfloat diffuse2[]   = { .3, .3, .3, 1.0 };
    static GLfloat specular2[]  = { .4, .4, .4, 1.0 };
    static GLfloat position2[]  = { 0.0, 0.0, -100.0, 1.0 };
    static GLfloat direction2[] = { .0, .0, 1.0 };
    glLightfv(GL_LIGHT1, GL_POSITION, position2);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction2);
    glLighti(GL_LIGHT1, GL_SPOT_CUTOFF,   90);
    glLighti(GL_LIGHT1, GL_SPOT_EXPONENT, 10);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  ambient2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  diffuse2);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular2);
    glEnable(GL_LIGHT1);
  }
}

void glStandardScene(void*, OpenGL& gl) {
  glPushAttrib(GL_CURRENT_BIT);
  glStandardLight(nullptr, gl);
  glDrawFloor(10, .4, .45, .5);
//   glDrawFloor(10, 1.5, 0.83, .0);
//   glDrawFloor(10., 108./255., 123./255., 139./255.);
  if(!gl.drawOptions.drawVisualsOnly) {
    glDrawAxes(.1);
  }
  glPopAttrib();
  glColor(.8, .8, .8);
}

void glStandardOriginAxes(void*, OpenGL&) {
  glDrawAxes(.1, true);
}

void glColor(int col) {
  static const GLfloat colorsTab[6][4] = {
    {0.2, 0.2, 1.0, 1.0}, // blue
    {1.0, 0.8, 0.0, 1.0}, // gold
    {1.0, 0.0, 0.0, 1.0}, // red
    {0.7, 0.7, 0.7, 1.0}, // gray
    {1.0, 1.0, 1.0, 1.0}, // white
    {0.2, 1.0, 0.2, 1.0}  // green
  };

  col = col%6; //if(col<0) col=0; if(col>5) col=5;
  glColor(colorsTab[col][0], colorsTab[col][1], colorsTab[col][2], colorsTab[col][3]);
}

void glColor(float r, float g, float b, float alpha, GLboolean lightingEnabled) {
  if(lightingEnabled>1) {
    glGetBooleanv(GL_LIGHTING, &lightingEnabled);
  }
  if(lightingEnabled==1) {
    float diff=1.f;
    GLfloat diffuse[4]  = { r*diff, g*diff, b*diff, alpha };
#if 1
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
#else
    float amb=1.f, spec=.25f;
    GLfloat ambient[4]  = { r*amb, g*amb, b*amb, alpha };
    GLfloat specular[4] = { spec* (1.f+r), spec* (1.f+g), spec* (1.f+b), alpha };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1.0f);
#endif
  } else {
    glColor4f(r, g, b, alpha);
  }
}

void glColor(float* rgb) { glColor(rgb[0], rgb[1], rgb[2], 1.); }

void glColor(const arr& col) {
  if(col.N==1) glColor(col.p[0], col.p[0], col.p[0], 1.);
  if(col.N==3) glColor(col.p[0], col.p[1], col.p[2], 1.);
  if(col.N==4) glColor(col.p[0], col.p[1], col.p[2], col.p[3]);
}

void glColorId(uint id) {
  byte rgb[3];
  glDisable(GL_LIGHTING);
  id2color(rgb, id);
  glColor3ubv(rgb);
}

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

void glTransform(const rai::Transformation& t) {
  double GLmatrix[16];
  t.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
}

void glRotate(const rai::Quaternion& rot) {
  double GLmatrix[16];
  rot.getMatrixGL(GLmatrix);
  glMultMatrixd(GLmatrix);
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

void glDrawText(const char* txt, float x, float y, float z, bool largeFont) {
  if(!txt) return;
  glDisable(GL_DEPTH_TEST);
  glPushLightOff();
  glRasterPos3f(x, y, z);
  void* font=GLUT_BITMAP_HELVETICA_12;
  if(largeFont) font = GLUT_BITMAP_HELVETICA_18;
  while(*txt) {
    switch(*txt) {
      case '\n':
        y+=15;
        if(largeFont) y+=4;
        glRasterPos3f(x, y, z);
        break;
      case '\b':
        if(font==GLUT_BITMAP_HELVETICA_12) font=GLUT_BITMAP_HELVETICA_18;
        else font=GLUT_BITMAP_HELVETICA_12;
        break;
      default: {
        glutBitmapCharacter(font, *txt);
      }
    }
    txt++;
  }
  glPopLight();
  glEnable(GL_DEPTH_TEST);
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

void glDrawPolygon(const arr& P) {
  CHECK_EQ(P.nd, 2, "");
  CHECK_EQ(P.d1, 3, "");
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glBegin(GL_POLYGON);
////  glColor(r, g, b);
//  for(uint i=0;i<P.d0;i++) glVertex3dv(&P(i,0));
//  glVertex3dv(P.p);
//  glEnd();
  glLineWidth(5);
  glBegin(GL_LINE_LOOP);
  for(uint i=0; i<P.d0; i++) glVertex3dv(&P(i, 0));
  glEnd();
}

void glDrawFloor(float x, float r, float g, float b) {
  x/=2.;

#if 1
  glColor(r+.15f, g+.15f, b+.15f);
  float eps=.002;
  for(int i=-5; i<=5; i++) {
    glBegin(GL_LINES);
    glVertex3f(i*x/5., -x, eps);
    glVertex3f(i*x/5., x, eps);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, i*x/5., eps);
    glVertex3f(x, i*x/5., eps);
    glEnd();
  }

//  glColor(.25, .25, .25);
//  glBegin(GL_LINE_STRIP);
//  glVertex3f(-x, -x, eps);
//  glVertex3f(-x, x, eps);
//  glVertex3f(x, x, eps);
//  glVertex3f(x, -x, eps);
//  glVertex3f(-x, -x, eps);
//  glEnd();
#endif

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
}

void glDrawBox(float x, float y, float z, bool linesOnly) {
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
  static GLint edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}
  };
  GLfloat v[8][3];
  GLint i;

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -x / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] =  x / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] =  -y / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] =  y / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] =  -z / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] =  z / 2;

  if(!linesOnly) {
    glBegin(GL_QUADS);
    for(i = 5; i >= 0; i--) {
      glNormal3fv(n[i]);
      glVertex3fv(v[faces[i][0]]);
      glVertex3fv(v[faces[i][1]]);
      glVertex3fv(v[faces[i][2]]);
      glVertex3fv(v[faces[i][3]]);
    }
    glEnd();
  } else {
    glBegin(GL_LINES);
    for(uint i=0; i<12; i++) {
      glVertex3fv(v[edges[i][0]]);
      glVertex3fv(v[edges[i][1]]);
    }
    glEnd();
  }
}

void glDrawDiamond(float x, float y, float z) {
//  glDisable(GL_CULL_FACE);
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
  glVertex3f(.0, -y, .0);
  glVertex3f(-x, .0, .0);
  glVertex3f(.0, y, .0);
  glVertex3f(x, .0, .0);
  glEnd();
//  glEnable(GL_CULL_FACE);
}

void glDrawDiamond(float x, float y, float z, float dx, float dy, float dz) {
  glPushMatrix();
  glTranslated(x, y, z);
  glDrawDiamond(dx, dy, dz);
  glPopMatrix();
}

void glDrawAxis(double scale) {
  glDisable(GL_CULL_FACE);
  if(scale>=0) glPushMatrix();
  if(scale>=0) glScalef(scale, scale, scale);
  GLUquadric* style=gluNewQuadric();
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(.95, 0, 0);
  glEnd();
  glTranslatef(.9, 0, 0);
  glRotatef(90, 0, 1, 0);
  gluCylinder(style, .04, 0, .1, 16, 1);
  gluDeleteQuadric(style);
  if(scale>=0) glPopMatrix();
  glEnable(GL_CULL_FACE);
}

void glDrawAxes(double scale, bool colored) {
  for(uint i=0; i<3; i++) {
    glPushMatrix();
    glScalef(scale, scale, scale);

    switch(i) {
      case 0:  if(colored) glColor(.7, 0, 0);  break;
      case 1:  if(colored) glColor(0, .7, 0);  glRotatef(90, 0, 0, 1);  break;
      case 2:  if(colored) glColor(0, 0, .7);  glRotatef(90, 0, -1, 0);  break;
    }
    glDrawAxis();
    glPopMatrix();
  }
}

void glDrawCamera(const rai::Camera& cam) {
  glDrawAxes(.1);

  double dxFar, dyFar, zFar;
  double dxNear, dyNear, zNear;
  zNear = cam.zNear;
  zFar = cam.zFar;
  if(zFar-zNear > 1.) zFar = 0.; //zNear + .1;
  if(cam.focalLength) {
    dyNear = zNear * .5/cam.focalLength;
    dyFar = zFar * .5/cam.focalLength;
    dxNear = cam.whRatio * dyNear;
    dxFar = cam.whRatio * dyFar;
  } else {
    CHECK(cam.heightAbs, "");
    dyFar = dyNear = cam.heightAbs/2.;
    dxFar = dxNear = cam.whRatio * dyNear;
  }
  glColor(.5, .5, .5);
  glBegin(GL_LINE_STRIP);
  glVertex3f(-dxNear, -dyNear, zNear);
  glVertex3f(-dxNear, dyNear, zNear);
  glVertex3f(dxNear, dyNear, zNear);
  glVertex3f(dxNear, -dyNear, zNear);
  glVertex3f(-dxNear, -dyNear, zNear);
  glEnd();
  if(zFar) {
    glBegin(GL_LINE_STRIP);
    glVertex3f(-dxFar, -dyFar, zFar);
    glVertex3f(-dxFar, dyFar, zFar);
    glVertex3f(dxFar, dyFar, zFar);
    glVertex3f(dxFar, -dyFar, zFar);
    glVertex3f(-dxFar, -dyFar, zFar);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxFar, -dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxFar, dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(dxFar, -dyFar, zFar);
    glVertex3f(0, 0, 0);
    glVertex3f(dxFar, dyFar, zFar);
    glEnd();
    glEnd();
  } else {
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxNear, -dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(-dxNear, dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(dxNear, -dyNear, zNear);
    glVertex3f(0, 0, 0);
    glVertex3f(dxNear, dyNear, zNear);
    glEnd();
  }
}

void glDrawDisk(float radius) {
  GLUquadric* style=gluNewQuadric();
  gluDisk(style, 0, radius, 10, 1);
  gluDeleteQuadric(style);
}

void glDrawProxy(const arr& p1, const arr& p2, double diskSize, int colorCode, const arr& norm, double _rad1, double _rad2) {
  glLoadIdentity();
  if(!colorCode) glColor(.8, .2, .2);
  else glColor(colorCode);
  glBegin(GL_LINES);
  glVertex3dv(p1.p);
  glVertex3dv(p2.p);
  glEnd();
  glDisable(GL_CULL_FACE);
  rai::Transformation f;
  f.pos=p1;
  if(!!norm) {
    f.rot.setDiff(rai::Vector(0, 0, 1), rai::Vector(norm));
  } else {
    f.rot.setDiff(rai::Vector(0, 0, 1), rai::Vector(p1-p2));
  }
  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(diskSize);

  f.pos=p2;
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glDrawDisk(diskSize);
  glEnable(GL_CULL_FACE);

  glLoadIdentity();
  if(!!norm && _rad1>0.) {
    arr p = p1 - _rad1*norm;
    glColor(0., 1., 0., 1.);
    glDrawDiamond(p(0), p(1), p(2), .01, .01, .01);
  }
  if(!!norm && _rad1>0.) {
    arr p = p2 + _rad2*norm;
    glColor(0., 0., 1., 1.);
    glDrawDiamond(p(0), p(1), p(2), .01, .01, .01);
  }
}

void glDrawSphere(float radius) {
  GLUquadric* style=gluNewQuadric();
  gluSphere(style, radius, 10, 10); // last two value for detail
  gluDeleteQuadric(style);
}

void glDrawCylinder(float radius, float length, bool closed) {
  GLUquadric* style=gluNewQuadric();
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
  GLUquadric* style1=gluNewQuadric();
  GLUquadric* style2=gluNewQuadric();
  GLUquadric* style3=gluNewQuadric();

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
  GLUquadric* style=gluNewQuadric();
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
  glVertex3f(-1, -1, 0.);
  glVertex3f(-1, +1, 0.);
  glVertex3f(+1, +1, 0.);
  glVertex3f(+1, -1, 0.);
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
    theta1 = (float)i * 2.0 * RAI_PI / rings;
    theta2 = (float)(i + 1) * 2.0 * RAI_PI / rings;
    for(j=0; j<sides; j++) {
      phi1 = (float)j * 2.0 * RAI_PI / sides;
      phi2 = (float)(j + 1) * 2.0 * RAI_PI / sides;

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

uint glImageTexture(const byteA& img) {
  GLuint texName;

  glEnable(GL_TEXTURE_2D);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &texName);

//  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texName);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  switch(img.d2) {
    case 0:
    case 1:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.d1, img.d0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.p);
      break;
    case 2:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, img.d1, img.d0, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, img.p);
      break;
    case 3:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.d1, img.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, img.p);
      break;
    case 4:
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.d1, img.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.p);
      break;
    default:
      HALT("no image fomat");
  }

  return texName;
}

void glDrawTexQuad(const byteA& texImg,
                   float x1, float y1, float z1, float x2, float y2, float z2,
                   float x3, float y3, float z3, float x4, float y4, float z4,
                   float mulX, float mulY) {
  glDisable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL); //GL_MODULATE);
  if(texImg.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
  if(texImg.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.d1, texImg.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImg.p);

//  glBindTexture(GL_TEXTURE_2D, texture);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0,  mulY); glVertex3f(x1, y1, z1);
  glTexCoord2f(mulX, mulY); glVertex3f(x2, y2, z2);
  glTexCoord2f(mulX, 0.0);  glVertex3f(x3, y3, z3);
  glTexCoord2f(0.0,  0.0);  glVertex3f(x4, y4, z4);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);

}

#ifdef RAI_GLUT
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
  glPixelStorei(GL_PACK_ALIGNMENT, 4);
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
#if defined RAI_SunOS
      glReadPixels(0, 0, w, h, GL_ABGR_EXT, GL_UNSIGNED_BYTE, image.p);
#else
#if defined RAI_Cygwin
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
  CHECK_EQ(depth.nd, 2, "depth buffer has to be either 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, depth.p);
}

/** @brief return the depth map of the scenery drawn just before; the depth
    buffer has to be a 2-dimensional [width, height] and is filled with
    depth values between 0 and 1. */
void glGrabDepth(floatA& depth) {
  if(!depth.N) depth.resize(glutGet(GLUT_WINDOW_HEIGHT), glutGet(GLUT_WINDOW_WIDTH));
  CHECK_EQ(depth.nd, 2, "depth buffer has to be 2-dimensional");
  GLint w=depth.d1, h=depth.d0;
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth.p);
}

void glRasterImage(float x, float y, byteA& img, float zoom) {
  glRasterPos3f(x, y, 0.); //(int)(y+zoom*img.d0)); (the latter was necessary for other pixel/raster coordinates)
  glPixelZoom(zoom, -zoom);
  if(img.d1%4) {  //necessary: extend the image to have width mod 4
    uint P=img.d2;
    if(!P) P=1;
    uint add=4-(img.d1%4);
    img.reshape(img.d0, img.d1*P);
    img.insColumns(-1, add*P);
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

void rai::Shape::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  //set name (for OpenGL selection)
  glPushName((frame.ID <<2) | 1);

  if(gl.drawOptions.drawMode_idColor) {
    glColorId(frame.ID);
    CHECK(!gl.drawOptions.drawColors, "must be disabled..");
  } else if(gl.drawOptions.drawColors) {
    if(mesh().C.N) glColor(mesh().C); //color[0], color[1], color[2], color[3]*world.orsDrawAlpha);
    else glColor(.5, .5, .5);
  }

  double GLmatrix[16];
  frame.ensure_X().getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!gl.drawOptions.drawShapes) {
    double scale=.33*(.02+::sum(size)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }

  //default!
  if(gl.drawOptions.drawShapes) {
    CHECK(_type!=rai::ST_none, "shape type is not initialized");

    if(_type==rai::ST_marker) {
      if(!gl.drawOptions.drawVisualsOnly) {
#if 0
        if(frame.parent) { //little line to parent
          rai::Vector p=frame.parent->ensure_X().pos;
          p = p / frame.ensure_X();
          glBegin(GL_LINES);
          glVertex3f(0, 0, 0);
          glVertex3f(p.x, p.y, p.z);
          glEnd();
        }
#endif
        double s=1.;
        if(size.N) s = size.last();
        if(s>0.) {
          glDrawDiamond(s/5., s/5., s/5.);
          glDrawAxes(s, gl.drawOptions.drawColors);
        } else if(s<0.) {
          glDrawAxis(-s);
        }
      }
    } else if(_type==rai::ST_camera) {
      if(!gl.drawOptions.drawVisualsOnly) {
        rai::Camera cam;
        cam.read(*frame.ats);
        glDrawCamera(cam); //gl.camera);
      }
    } else if(_type==rai::ST_density) {
      auto gridSdf = std::dynamic_pointer_cast<SDF_GridData>(_sdf);
      if(gridSdf) {
        gridSdf->_densityDisplayData->glDraw(gl);
      }
    } else {
      if(!mesh().V.N) {
        LOG(-1) <<"trying to draw empty mesh (shape type:" <<_type <<")";
      } else {
        if(!gl.drawOptions.drawVisualsOnly || mesh().T.d1==3) { //visual -> surface meshes only
          if(!mesh().T.N && size.N) glPointSize(size.last());
          if(!mesh().C.N) glColor(.8, .8, .8);
          if(_type==rai::ST_mesh || _type==rai::ST_pointCloud || _type==rai::ST_ssCvx) {
#if 1 //use list
            NIY; //glDrawAsList(mesh(), gl);
#else
            mesh().glDraw(gl);
#endif
          } else {
            mesh().glDraw(gl);
          }
          if(!mesh().T.N && size.N) glPointSize(1.);
        }
      }
    }
  }

  if(gl.drawOptions.drawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -frame.ensure_X().pos.z);
    glEnd();
  }

  if(gl.drawOptions.drawFrameNames) {
    glColor(1, 1, 1);
    glDrawText(frame.name, 0, 0, 0);
  }

  glPopName();
#endif
}

void rai::PlotModule::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  sPlotModule& data=*self;
//  uint a, i, j;

  uint idx = 0;

  rai::Color c;

  double x=0., y=0., z=0.;

  //light?
  if(light) glStandardLight(nullptr, gl);

  if(drawBox) {
    glColor3f(.7, .7, .7);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, 1, -1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, -1, -1);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(-1, -1, 1);
    glVertex3f(-1, 1, 1);
    glVertex3f(1, 1, 1);
    glVertex3f(1, -1, 1);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, -1, -1);
    glVertex3f(1, -1, 1);
    glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1, 1);
    glVertex3f(1, 1, -1);
    glVertex3f(1, 1, 1);
    glVertex3f(-1, 1, -1);
    glVertex3f(-1, 1, 1);
    glEnd();
  }

  //draw images
  //for(byteA& i:data.images) {
  //}

  //draw arrays
  for(arr& a:data.array) {
    CHECK_LE(a.nd, 2, "can't display 3(or higher)-dim arrays");
    //1D function, just series of y
    if(a.nd==1 || (a.nd==2 && a.d1==1)) {
      c.setIndex(idx);
      glColor(c.r, c.g, c.b);

      for(uint i=1; i<a.N; i++) {
        glBegin(GL_LINES);
        glVertex3f(2.*(i-1)/(a.N-1)-1., a.elem(i-1), 0);
        glVertex3f(2.*(i)/(a.N-1)-1., a.elem(i), 0);
        glEnd();
      }
      glBegin(GL_LINE_LOOP);
      glColor3f(0., 0., 0.);
      glVertex3f(-1, -1, 0);
      glVertex3f(-1, 1, 0);
      glVertex3f(1, 1, 0);
      glVertex3f(1, -1, 0);
      glEnd();
    }
    //2D function x -> y
    if(a.nd==2 && a.d1==2) { //2D path
      c.setIndex(idx);
      glColor(c.r, c.g, c.b);
      glBegin(GL_LINE_STRIP);
      for(uint i=0; i<a.d0; i++) {
        glVertex3f(2.*i/(a.d0-1)-1., a.operator()(i, 0), a.operator()(i, 1));
      }
      glEnd();
    }
    if(a.nd==2 && a.d1>2) { //2D landscapes
      uint i, j, X=a.d1, Y=a.d0;
      c.setIndex(idx);
      if(!grid) { //as a mesh
        c.whiten(.5);
        CHECK_EQ(Y*X, data.mesh.V.d0, "you must recall display(data.array) when dimensions changed");
        for(j=0; j<Y; j++) for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y= 2.*(double)j/(Y-1.)-1.;
            z=a(j, i);
            c.setTemp2(z);
            data.mesh.V(j*X+i, 0)=x;    data.mesh.V(j*X+i, 1)=y;    data.mesh.V(j*X+i, 2)=z;
            data.mesh.C(j*X+i, 0)=c.r;  data.mesh.C(j*X+i, 1)=c.g;  data.mesh.C(j*X+i, 2)=c.b;
          }
        data.mesh.computeNormals();
        glDisable(GL_CULL_FACE);
        data.mesh.glDraw(gl);
        glEnable(GL_CULL_FACE);
      } else { //as a grid
        c.blacken(.5);
        for(j=0; j<Y; j++) { //along the x-axis
          glBegin(GL_LINE_STRIP);
          for(i=0; i<X; i++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=a(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
        for(i=0; i<X; i++) { //along the y-axis
          glBegin(GL_LINE_STRIP);
          for(j=0; j<Y; j++) {
            x= 2.*(double)i/(X-1.)-1.;
            y=-2.*(double)j/(Y-1.)+1.;
            z=a(j, i);
            //c.setTemp2(z);
            glColor3f(c.r, c.g, c.b);
            glColor(c.r, c.g, c.b);
            glVertex3f(x, y, z);
          }
          glEnd();
        }
      }
    }
    idx++;
  }

  //draw points
  for(arr& p:data.points) {
    c.setIndex(idx);
    glColor(c.r, c.g, c.b);
    //glBegin(GL_LINES);
    if(drawDots) glBegin(GL_POINTS);
    if(p.nd==2) {
      for(uint j=0; j<p.d0; j++) {
        if(p.d1==1) { x=(double)j; y=p(j, 0); z=0.; }
        if(p.d1==2) { x=p(j, 0); y=p(j, 1); z=0.; }
        if(p.d1>=3) { x=p(j, 0); y=p(j, 1); z=p(j, 2); }
        if(!drawDots) {
          glPushMatrix();
          glTranslatef(x, y, z);
          glDrawDiamond(.01, .01, .01);
          glPopMatrix();
        } else {
          glVertex3d(x, y, z);
        }
      }
    } else {
      if(p.d0==1) { x=p(0); y=0.; z=0.; }
      if(p.d0==2) { x=p(0); y=p(1); z=0.; }
      if(p.d0>=3) { x=p(0); y=p(1); z=p(2); }
      if(!drawDots) {
        glPushMatrix();
        glTranslatef(x, y, z);
        glDrawDiamond(.02, .02, .02);
        glPopMatrix();
      } else {
        glVertex3d(x, y, z);
      }
    }
    if(drawDots) glEnd();
    idx++;
  }

  //draw lines
  for(arr& l:data.lines) {
    if(colors) c.setIndex(idx); else c.setIndex(0);
    glColor(c.r, c.g, c.b);

    if(thickLines) {
      glLineWidth(thickLines);
    }

    glBegin(GL_LINE_STRIP);
    for(uint j=0; j<l.d0; j++) {
      if(l.d1==1) glVertex3d((double)j, l(j, 0), 0.);
      if(l.d1==2) glVertex3d(l(j, 0), l(j, 1), 0.);
      if(l.d1>=3) glVertex3d(l(j, 0), l(j, 1), l(j, 2));
    }
    glEnd();
    idx++;
  }

  //draw planes
  for(uint i=0; i<data.planes.N; i+=4) {
    c.setIndex(i/4+1);
    glColor(c.r, c.g, c.b);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    glVertex3f(data.planes(i).x, data.planes(i).y, data.planes(i).z);
    glVertex3f(data.planes(i+1).x, data.planes(i+1).y, data.planes(i+1).z);
    glVertex3f(data.planes(i+2).x, data.planes(i+2).y, data.planes(i+2).z);
    glVertex3f(data.planes(i+3).x, data.planes(i+3).y, data.planes(i+3).z);
    glEnd();
  }
#else
  NIY;
#endif
}

void rai::ForceExchange::glDraw(OpenGL& gl) {
  double scale = 2.;
  arr _poa, _torque, _force;
  kinPOA(_poa, NoArr);
  kinForce(_force, NoArr);
  kinTorque(_torque, NoArr);
  if(b.joint && b.joint->type==JT_hingeX) {
    arr x = b.ensure_X().rot.getX().getArr();
    _torque = x * scalarProduct(x, torque);
    _force = 0.;
  }

#ifdef RAI_GL
  glLoadIdentity();
  glColor(1., 0., 1., 1.);
  glDrawDiamond(_poa(0), _poa(1), _poa(2), .02, .02, .02); //POA dimons
  glLineWidth(3.f);
  glBegin(GL_LINES);
  glColor(1., 0., 1., 1.);
  glVertex3dv(_poa.p);
  glVertex3dv((_poa+scale*_torque).p); //pink: torque
  glColor(1., 1., 1., 1.);
  glVertex3dv(_poa.p);
  glVertex3dv((_poa+scale*_force).p); //white: force
  glEnd();
  glLineWidth(1.f);
  glColor(.0, .0, .0, 1.);

//  glBegin(GL_LINES);
//  glVertex3dv(&a.ensure_X().pos.x);
//  glVertex3dv(_poa.p);
//  glColor(.8, .5, .8, 1.);
//  glVertex3dv(_poa.p);
//  glVertex3dv(&b.ensure_X().pos.x);
//  glEnd();

  glLoadIdentity();

//    f.pos=.5*(posA+posB);
//    f.getAffineMatrixGL(GLmatrix);
//    glLoadMatrixd(GLmatrix);
//    glDrawText(STRING(a <<'-' <<b <<':' <<d), 0.,0.,0.);
#endif
}

void Configuration::glDraw(OpenGL& gl) {
  glDraw_frames(gl, frames);
}

/// GL routine to draw a Configuration
void Configuration::glDraw_frames(OpenGL& gl, const FrameL& F, int drawOpaqueOrTransparanet) {
#ifdef RAI_GL
  Transformation f;
  double GLmatrix[16];

  glPushMatrix();

//  glColor(.5, .5, .5);

  if(drawOpaqueOrTransparanet!=2) {
    if(gl.drawOptions.drawVisualsOnly) {
      gl.drawOptions.drawProxies=gl.drawOptions.drawJoints=false;
      drawOpaqueOrTransparanet=1;
    }

    //proxies
    if(gl.drawOptions.drawProxies) for(const Proxy& p: proxies) {
        if(p.a && p.b && p.d<=.1) {
          CHECK_EQ(&p.a->C, this, "");
          if(F.N==frames.N || F.contains(p.a) || F.contains(p.b)){
            ((Proxy*)&p)->glDraw(gl);
          }
        }
      }

    for(Frame* fr: F) for(ForceExchange* f:fr->forces) {
        if(f->sign(fr)>0.) f->glDraw(gl);
      }

    //joints
    Joint* e;
    if(gl.drawOptions.drawJoints) for(Frame* fr: F) if((e=fr->joint)) {
          //set name (for OpenGL selection)
          glPushName((fr->ID <<2) | 2);

          //    double s=e->A.pos.length()+e->B.pos.length(); //some scale
          double s=.1;

          //    //from body to joint
          //    f=e->from->X;
          //    f.getAffineMatrixGL(GLmatrix);
          //    glLoadMatrixd(GLmatrix);
          //    glColor(1, 1, 0);
          //    //glDrawSphere(.1*s);
          //    glBegin(GL_LINES);
          //    glVertex3f(0, 0, 0);
          //    glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
          //    glEnd();

          //joint frame A
          //    f.appendTransformation(e->A);
          f.getAffineMatrixGL(GLmatrix);
          glLoadMatrixd(GLmatrix);
          glDrawAxes(s);
          glColor(1, 0, 0);
          glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

          //joint frame B
          f.appendTransformation(fr->get_Q());
          f.getAffineMatrixGL(GLmatrix);
          glLoadMatrixd(GLmatrix);
          glDrawAxes(s);

          //    //from joint to body
          //    glColor(1, 0, 1);
          //    glBegin(GL_LINES);
          //    glVertex3f(0, 0, 0);
          //    glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
          //    glEnd();
          //    glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
          //    //glDrawSphere(.1*s);

          glPopName();
        }
  }

  //shapes
  if(drawOpaqueOrTransparanet==0 || drawOpaqueOrTransparanet==1) {
    //first non-transparent
    for(Frame* f: F) if(f->shape && f->shape->alpha()==1.) {
        if(F.nd==2 && f->ID>F.d1 && f->shape->_mesh==F.elem(f->ID-F.d1)->shape->_mesh && f->X==F.elem(f->ID-F.d1)->X) { //has the same shape and pose as previous time slice frame
          continue;
        }
        f->shape->glDraw(gl);
      }
  }
  if(drawOpaqueOrTransparanet==0 || drawOpaqueOrTransparanet==2) {
    for(Frame* f: F) if(f->shape && f->shape->alpha()<1.) {
        f->shape->glDraw(gl);
      }
  }

  glPopMatrix();
#endif
}

void PairCollision::glDraw(OpenGL&) {
#ifdef RAI_GL
  arr P1=p1, P2=p2;
  if(rad1>0.) P1 -= rad1*normal;
  if(rad2>0.) P2 += rad2*normal;

  glColor(0., 1., 0., 1.);
  glDrawDiamond(P1(0), P1(1), P1(2), .005, .005, .005);
  if(simplex1.N) {
    for(uint i=0; i<simplex1.d0; i++) simplex1[i] -= rad1*normal;
    glDrawPolygon(simplex1);
    for(uint i=0; i<simplex1.d0; i++) simplex1[i] += rad1*normal;
  }

  glColor(0., 0., 1., 1.);
  glDrawDiamond(P2(0), P2(1), P2(2), .005, .005, .005);
  if(simplex2.N) {
    for(uint i=0; i<simplex2.d0; i++) simplex2[i] += rad2*normal;
    glDrawPolygon(simplex2);
    for(uint i=0; i<simplex2.d0; i++) simplex2[i] -= rad2*normal;
  }

  glColor(1., 0., 0., 1.);
  glLineWidth(2.f);
  glDrawProxy(P1, P2, .02);
  glLineWidth(1.f);
  glLoadIdentity();

  if(poly.N) {
    glColor(0., 1., 1., 1.);
    glLineWidth(1.f);
    glDrawPolygon(poly);
    uint n=poly.d0;
    for(uint i=0; i<n; i++) {
      rai::Transformation T;
      T.pos = .5 *(poly[(i+1)%n] + poly[i]);
      T.rot.setDiff(Vector_x, polyNorm[i]);
//      cout <<polyNorm[i] <<' ' <<T.rot <<' ' <<T.rot.getDeg() <<endl;
      glTransform(T);
      glDrawAxis(.02);
    }
  }
#endif
}

void F_PairFunctional::glDraw(OpenGL&) {
#ifdef RAI_GL
  glColor(0., 1., 0., 1.);
  glDrawDiamond(x(0), x(1), x(2), .05, .05, .05);
  if(P) {
    glColor(0., 1., 1., 1.);
    glDrawDiamond(P->z1(0), P->z1(1), P->z1(2), .05, .05, .05);
    glColor(0., 0., 1., 1.);
    glDrawDiamond(P->z2(0), P->z2(1), P->z2(2), .05, .05, .05);
  }

  glColor(1., 0., 0., 1.);
  glLineWidth(2.f);
  glDrawProxy(x-d1*g1, x, .02);
  glDrawProxy(x, x-d2*g2, .02);
  glLineWidth(1.f);
  glLoadIdentity();
#endif
}


void rai::Proxy::glDraw(OpenGL& gl) {
#ifdef RAI_GL
  if(collision) {
    glLoadIdentity();
    collision->glDraw(gl);
  } else {
    glLoadIdentity();
    if(!colorCode) {
      if(d>0.) glColor(.2, .8, .2);
      else glColor(1, 0, 0);
    } else glColor(colorCode);
    glBegin(GL_LINES);
    glVertex3dv(posA.p());
    glVertex3dv(posB.p());
    glEnd();
    glDisable(GL_CULL_FACE);
    rai::Transformation f;
    f.pos=posA;
    f.rot.setDiff(rai::Vector(0, 0, 1), posA-posB);
    double GLmatrix[16];
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);

    f.pos=posB;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);

#if 0 //write text
    f.pos=.5*(posA+posB);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawText(STRING(a->name <<'-' <<b->name <<':' <<d), 0., 0., 0.);
#endif

    glEnable(GL_CULL_FACE);
  }
#endif
}

void rai::CameraView::glDraw(OpenGL& gl) {
  if(renderMode==all || renderMode==visuals) {
    glStandardScene(nullptr, gl);
    gl.drawOptions.drawMode_idColor = false;
    gl.drawOptions.drawColors = true;
    if(renderMode==visuals) {
      gl.drawOptions.drawVisualsOnly=true;
    } else {
      gl.drawOptions.drawVisualsOnly=false;
    }

    NIY; //C.glDraw(gl);

    if(renderMode!=visuals) {
      for(Sensor& sen:sensors) {
        NIY;
//        glTransform(sen.cam.X);
//        glDrawCamera(sen.cam);
//        glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
      }
    }
  }

  if(renderMode==seg) {
    gl.clearColor=1.;
    gl.drawOptions.drawMode_idColor = true;
    gl.drawOptions.drawColors=false;
    gl.drawOptions.drawVisualsOnly=true;
    NIY; //C.glDraw(gl);
    gl.drawOptions.drawMode_idColor = false;
    gl.drawOptions.drawColors=true;
  }
}

void DrawActor(PxRigidActor* actor, rai::Frame* frame, OpenGL& gl) {
  PxU32 nShapes = actor->getNbShapes();
  PxShape** shapes=new PxShape*[nShapes];
  //cout <<"#shapes=" <<nShapes;

  actor->getShapes(shapes, nShapes);
  while(nShapes--) {
    PxShape* shape = shapes[nShapes];

    // use the color of the first shape of the body for the entire body
    rai::Shape* s = frame->shape;
    if(!s) for(rai::Frame* ch:frame->children) {
        if(ch->shape && ch->shape->alpha()==1.) {
          s = ch->shape;
          break;
        }
      }
    if(s) glColor(s->mesh().C);

    rai::Transformation f;
    double mat[16];
    PxTrans2raiTrans(f, PxShapeExt::getGlobalPose(*shape, *actor));
    glLoadMatrixd(f.getAffineMatrixGL(mat));
    //cout <<"drawing shape " <<body->name <<endl;
    switch(shape->getGeometryType()) {
      case PxGeometryType::eBOX: {
        PxBoxGeometry g;
        shape->getBoxGeometry(g);
        //glutSolidCube(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
        glDrawBox(g.halfExtents.x*2, g.halfExtents.y*2, g.halfExtents.z*2);
      } break;
      case PxGeometryType::eSPHERE: {
        PxSphereGeometry g;
        shape->getSphereGeometry(g);
        glutSolidSphere(g.radius, 10, 10);
      } break;
      case PxGeometryType::eCAPSULE: {
        PxCapsuleGeometry g;
        shape->getCapsuleGeometry(g);
        glDrawCappedCylinder(g.radius, g.halfHeight*2);
      } break;
      case PxGeometryType::eCONVEXMESH: {
#if 1
        PxConvexMeshGeometry g;
        shape->getConvexMeshGeometry(g);
        floatA Vfloat;
        Vfloat.referTo((float*)g.convexMesh->getVertices(), 3*g.convexMesh->getNbVertices()); //reference!
        rai::Mesh mesh;
        copy(mesh.V, Vfloat);
        mesh.V.reshape(g.convexMesh->getNbVertices(), 3);
        mesh.makeConvexHull();
        mesh.glDraw(gl);
#else
        self->mesh.glDraw();
#endif
      } break;
      case PxGeometryType::eTRIANGLEMESH: {
        PxTriangleMeshGeometry g;
        shape->getTriangleMeshGeometry(g);
        floatA Vfloat;
        Vfloat.referTo((float*)g.triangleMesh->getVertices(), 3*g.triangleMesh->getNbVertices()).reshape(-1, 3);
        rai::Mesh mesh;
        mesh.V = rai::convert<double>(Vfloat);
        if(g.triangleMesh->getTriangleMeshFlags()&PxTriangleMeshFlag::e16_BIT_INDICES) {
          rai::Array<uint16_t> T16;
          T16.referTo((uint16_t*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
          mesh.T = rai::convert<uint>(T16);
        } else {
          mesh.T.referTo((uint*)g.triangleMesh->getTriangles(), 3*g.triangleMesh->getNbTriangles()).reshape(-1, 3);
        }
        mesh.glDraw(gl);
      } break;

      default:
        RAI_MSG("can't draw this type");
    }
  }
  delete [] shapes;
}

void RRT_SingleTree::glDraw(OpenGL& gl) {
  glColor(.0, .0, .0);
  glLineWidth(2.f);
  glBegin(GL_LINES);
  drawMutex.lock(RAI_HERE);
  for(uint i=1; i<getNumberNodes(); i++) {
    glVertex3dv(&disp3d(parent(i), 0));
    glVertex3dv(&disp3d(i, 0));
  }
  drawMutex.unlock();
  glEnd();
  glLineWidth(1.f);
}

/// GL routine to draw a Mesh
void Mesh::glDraw(struct OpenGL& gl) {
  GLboolean lightingEnabled=true;
  glGetBooleanv(GL_LIGHTING, &lightingEnabled);

  if(glDrawOptions(gl).drawColors) {
    if(C.nd==1) {
      CHECK(C.N>=1 && C.N<=4, "need a basic color");
      GLfloat col[4];
      if(C.N>=3) {
        col[0] = C.elem(0);
        col[1] = C.elem(1);
        col[2] = C.elem(2);
        col[3] = (C.N==4?C.elem(3):1.);
      } else {
        col[0] = col[1] = col[2] = C.elem(0);
        col[3] = (C.N==2?C.elem(1):1.);
      }
      if(lightingEnabled && T.N) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);
      else glColor4fv(col);
    }
  }

  if(!T.N && V.N) { //-- draw point cloud
    CHECK_EQ(V.nd, 2, "wrong dimension");
    CHECK_EQ(V.d1, 3, "wrong dimension");
    glDisable(GL_LIGHTING);

    glEnableClientState(GL_VERTEX_ARRAY);
    if(C.d0==V.d0) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.d0==V.d0) glColorPointer(C.d1, GL_DOUBLE, 0, C.p);

    glDrawArrays(GL_POINTS, 0, V.d0);

    if(Vn.N) { //draw normals
      CHECK_EQ(Vn.N, V.N, "");
      arr p, n;
      glColor4d(.5, .5, .5, .2);
      glBegin(GL_LINES);
      for(uint i=0; i<V.d0; i++) {
        //if(C.N==V.N) glColor3dv(&C(i,0));
        p.setCarray(&V(i, 0), 3);
        n.setCarray(&Vn(i, 0), 3);
        glVertex3dv(p.p);
        glVertex3dv((p+.01*n).p);
      }
      glEnd();
      if(C.N==1) glColor3d(*C.p, *C.p, *C.p);
      else if(C.N==3) glColor3dv(C.p);
      else if(C.N==4) glColor4dv(C.p);
    }

    if(lightingEnabled) glEnable(GL_LIGHTING);
  }

  if(T.d1==2) { //-- draw lines
    //    glLineWidth(3.f);
    //    glShadeModel(GL_FLAT);
    glShadeModel(GL_SMOOTH);
#if 0
    uint v;
    glBegin(GL_LINES);
    for(uint i=0; i<T.d0; i++) {
      if(C.d0==T.d0) {
        if(C.d1==3) glColor(C(i, 0), C(i, 1), C(i, 2), 1.);
        if(C.d1==1) glColorId(C(i, 0));
      }
      v=T(i, 0);  if(C.nd==2 && C.d0==V.d0) glColor(C(v, 0), C(v, 1), C(v, 2), 1.f);  glVertex3dv(&V(v, 0));
      v=T(i, 1);  if(C.nd==2 && C.d0==V.d0) glColor(C(v, 0), C(v, 1), C(v, 2), 1.f);  glVertex3dv(&V(v, 0));
    }
    glEnd();
#else
    glEnableClientState(GL_VERTEX_ARRAY);
    if(glDrawOptions(gl).drawColors) {
      if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
      if(C.N==V.N) glDisable(GL_LIGHTING); //because lighting requires ambiance colors to be set..., not just color..
    }

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(glDrawOptions(gl).drawColors) {
      if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    }

    glDrawElements(GL_LINES, T.N, GL_UNSIGNED_INT, T.p);

#if 1 //points as well
    glPointSize(3);
    glDrawArrays(GL_POINTS, 0, V.d0);
    glPointSize(1);
#endif

    if(C.N==V.N) glEnable(GL_LIGHTING);
#endif
  }

  //-- draw a mesh
  if(T.d0 && (T.d0!=Tn.d0)) computeNormals();

  //-- if not yet done, GenTexture
  if(texImg.N && glDrawOptions(gl).drawColors) {
    if(texture<0) {
      GLuint texName;
      glGenTextures(1, &texName);
      texture = texName;
      glBindTexture(GL_TEXTURE_2D, texture);

      if(texImg.d2==4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImg.d1, texImg.d0, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImg.p);
      else if(texImg.d2==3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImg.d1, texImg.d0, 0, GL_RGB, GL_UNSIGNED_BYTE, texImg.p);
      else NIY;
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
      glBindTexture(GL_TEXTURE_2D, texture);
    }
  }

  //-- draw the mesh
  if(false && (!C.N || C.nd==1 || !glDrawOptions(gl).drawColors || (C.d0==V.d0 && !lightingEnabled))  //we have colors for each vertex
      && (!tex.N || !Tt.N)) { //we have no tex or tex coords for each vertex -> use index arrays

    glShadeModel(GL_FLAT); //triangles with constant reflection
//    glShadeModel(GL_SMOOTH); //smoothed over vertices

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    if(glDrawOptions(gl).drawColors) {
      if(tex.N) CHECK_EQ(tex.d0, V.d0, "this needs tex coords for each vertex; if you have it face wise, render the slow way..");
      if(tex.N) glEnable(GL_TEXTURE_2D);

      if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);
      if(C.N==V.N) glDisable(GL_LIGHTING); //because lighting requires ambiance colors to be set..., not just color..
      if(tex.N) glEnableClientState(GL_TEXTURE_COORD_ARRAY); else glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    glNormalPointer(GL_DOUBLE, 0, Vn.p);
    if(glDrawOptions(gl).drawColors) {
      if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
      if(tex.N) glTexCoordPointer(2, GL_DOUBLE, 0, tex.p);
    }

    glDrawElements(GL_TRIANGLES, T.N, GL_UNSIGNED_INT, T.p);

    if(C.N==V.N) glEnable(GL_LIGHTING);

    if(tex.N) glDisable(GL_TEXTURE_2D);

//  } else if(C.d0==T.d0){ //we have colors for each tri -> render tris directly and with tri-normals

//    CHECK_EQ(C.d0, T.d0, "");
//    CHECK_EQ(Tn.d0, T.d0, "");
//    glShadeModel(GL_FLAT);
//    glBegin(GL_TRIANGLES);
//    GLboolean light=true;
//    glGetBooleanv(GL_LIGHTING, &light); //this doesn't work!!?? even when disabled, returns true; never changes 'light'
//    for(uint t=0; t<T.d0; t++) {
//      uint   *tri  = T.p  + 3*t; //&T(t, 0);
//      double *col  = C.p  + 3*t; //&C(t, 0);
//      double *norm = Tn.p + 3*t; //&Tn(t, 0);

//      GLfloat ambient[4] = { (float)col[0], (float)col[1], (float)col[2], 1.f };
//      if(!light) glColor4fv(ambient);
//      else       glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient);

//      glNormal3dv(norm);
//      glVertex3dv(V.p + 3*tri[0]); //&V(tri[0],0);
//      glVertex3dv(V.p + 3*tri[1]);
//      glVertex3dv(V.p + 3*tri[2]);
//    }
//    glEnd();
  } else { //basic vertex-wise
    if(tex.N) CHECK_EQ(Tt.d0, T.d0, "this needs tex coords for each tri");
    if(tex.N && glDrawOptions(gl).drawColors) glEnable(GL_TEXTURE_2D);

    glShadeModel(GL_FLAT); //triangles with constant reflection
//    glShadeModel(GL_SMOOTH); //smoothed over vertices

    glBegin(GL_TRIANGLES);
    for(uint i=0; i<T.d0; i++) {
      glNormal3dv(Tn.p+3*i);
      if(C.nd==2 && C.d0==T.d0) {
        if(C.d1==3) { double* c = C.p+C.d1*i; glColor(c[0], c[1], c[2], 1.f, lightingEnabled); }
        if(C.d1==1) glColorId(C(i, 0));
      }
      uint* t = T.p+3*i;
      for(uint j=0; j<3; j++) {
        if(C.nd==2 && C.d0==V.d0) { double* c = C.p+C.d1*t[j]; glColor(c[0], c[1], c[2], 1.f, lightingEnabled); }
        if(Tt.N) glTexCoord2dv(&tex(Tt(i, 0), 0));
        glVertex3dv(V.p+3*t[j]);
      }
    }
    glEnd();
    if(Tt.N && texImg.N && glDrawOptions(gl).drawColors) {
      glDisable(GL_TEXTURE_2D);
    }
#if 0 //draw normals //simple with triangle normals
    glColor(.5, 1., .0);
    Vector a, b, c, x;
    for(i=0; i<T.d0; i++) {
      glBegin(GL_LINES);
      a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
      x.setZero(); x+=a; x+=b; x+=c; x/=3;
      glVertex3dv(x.v);
      a.set(&Tn(i, 0));
      x+=.05*a;
      glVertex3dv(x.v);
      glEnd();
    }
#endif
  }

  if(glDrawOptions(gl).drawWires) { //on top of mesh
#if 1
    glColor(0, 0, 0, 1, 2);
    uint t;
    for(t=0; t<T.d0; t++) {
      glBegin(GL_LINE_LOOP);
      glVertex3dv(&V(T(t, 0), 0));
      glVertex3dv(&V(T(t, 1), 0));
      glVertex3dv(&V(T(t, 2), 0));
      glEnd();
    }
#else
    glColor(0., 0., 0., 1.);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    if(C.N==V.N) glEnableClientState(GL_COLOR_ARRAY); else glDisableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_DOUBLE, 0, V.p);
    if(C.N==V.N) glColorPointer(3, GL_DOUBLE, 0, C.p);
    glDrawElements(GL_LINE_STRIP, T.N, GL_UNSIGNED_INT, T.p);
#endif
  }

//  glEndList();
//  }

}

void DensityDisplayData::glDraw(OpenGL& gl) {
  box.glDraw(gl);
  gl.drawOptions.enableLighting=false;
  //get view direction
  arr view = gl.camera.X.rot.getZ().getArr();
  glDisable(GL_CULL_FACE);
  uint side = argmax(fabs(view));
  switch(side) {
    case 0: {
      if(view(0)<0.) {
        for(uint i=0; i<volumeX.N; i++) volumeX(i).glDraw(gl);
      } else {
        for(uint i=volumeX.N; i--;) volumeX(i).glDraw(gl);
      }
    } break;
    case 1: {
      if(view(1)<0.) {
        for(uint i=0; i<volumeY.N; i++) volumeY(i).glDraw(gl);
      } else {
        for(uint i=volumeY.N; i--;) volumeY(i).glDraw(gl);
      }
    } break;
    case 2: {
      if(view(2)<0.) {
        for(uint i=0; i<volumeZ.N; i++) volumeZ(i).glDraw(gl);
      } else {
        for(uint i=volumeZ.N; i--;) volumeZ(i).glDraw(gl);
      }
    } break;
  }

  glEnable(GL_CULL_FACE);
  gl.drawOptions.enableLighting=true;
}

void PhysXInterface::glDraw(OpenGL& gl) {
  NIY; //gl.text.clear() <<self->stepCount;
  for(PxRigidActor* a: self->actors) {
    if(a) {
      rai::Frame* f = (rai::Frame*)a->userData;
      NIY; //DrawActor(a, f, gl);
    }
  }
}

