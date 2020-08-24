/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin.h"
#include "../Geo/mesh.h"
#include "../Gui/opengl.h"

#include <iomanip>

//global options
bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true, orsDrawColors=true, orsDrawIndexColors=false;
bool orsDrawMeshes=true, orsDrawCores=false, orsDrawZlines=false;
bool orsDrawBodyNames=false;
double orsDrawAlpha=0.50;
uint orsDrawLimit=0;

#ifdef RAI_GL
#  include <GL/gl.h>
#  include <GL/glu.h>

extern void glDrawRect(float, float, float, float, float, float,
                       float, float, float, float, float, float);

extern void glDrawText(const char* txt, float x, float y, float z);

//void glColor(float *rgb);//{ glColor(rgb[0], rgb[1], rgb[2], 1.); }

#ifndef RAI_ORS_ONLY_BASICS

/**
 * @brief Bind ors to OpenGL.
 * Afterwards OpenGL can show the ors graph.
 *
 * @param graph the ors graph.
 * @param gl OpenGL which shows the ors graph.
 */
void bindOrsToOpenGL(rai::Configuration& graph, OpenGL& gl) {
  gl.add(glStandardScene, 0);
  gl.add(rai::glDrawGraph, &graph);
//  gl.setClearColors(1., 1., 1., 1.);

  rai::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    gl.camera.X = glCamera->X;
    gl.resize(500, 500);
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}
#endif

#ifndef RAI_ORS_ONLY_BASICS

/// static GL routine to draw a rai::Configuration
void rai::glDrawGraph(void* classP) {
  ((rai::Configuration*)classP)->glDraw(NoOpenGL);
}

void rai::Shape::glDraw(OpenGL& gl) {
  //set name (for OpenGL selection)
  glPushName((index <<2) | 1);
  if(orsDrawColors && !orsDrawIndexColors) glColor(color[0], color[1], color[2], orsDrawAlpha);
  if(orsDrawIndexColors) glColor3b((index>>16)&0xff, (index>>8)&0xff, index&0xff);

  double GLmatrix[16];
  X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!orsDrawShapes) {
    double scale=.33*(size(0)+size(1)+size(2) + 2.*size(3)); //some scale
    if(!scale) scale=1.;
    scale*=.3;
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(orsDrawShapes) {
    switch(type) {
      case rai::ST_none: LOG(-1) <<"Shape '" <<name <<"' has no joint type";  break;
      case rai::ST_box:
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawBox(size(0), size(1), size(2));
        break;
      case rai::ST_sphere:
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawSphere(size(3));
        break;
      case rai::ST_cylinder:
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCylinder(size(3), size(2));
        break;
      case rai::ST_capsule:
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(orsDrawMeshes && mesh.V.N) mesh.glDraw(gl);
        else glDrawCappedCylinder(size(3), size(2));
        break;
      case rai::ST_retired_SSBox:
        HALT("deprecated??");
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else if(orsDrawMeshes) {
          if(!mesh.V.N) mesh.setSSBox(size(0), size(1), size(2), size(3));
          mesh.glDraw(gl);
        } else NIY;
        break;
      case rai::ST_marker:
        if(orsDrawMarkers) {
          glDrawDiamond(size(0)/5., size(0)/5., size(0)/5.); glDrawAxes(size(0));
        }
        break;
      case rai::ST_mesh:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case rai::ST_ssCvx:
        CHECK(sscCore.V.N, "sscCore needs to be loaded to draw mesh object");
        if(!mesh.V.N) mesh.setSSCvx(sscCore, size(3));
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case rai::ST_ssBox:
        if(!mesh.V.N || !sscCore.V.N) {
          sscCore.setBox();
          sscCore.scale(size(0), size(1), size(2));
          mesh.setSSCvx(sscCore, size(3));
        }
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;
      case rai::ST_pointCloud:
        CHECK(mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        if(orsDrawCores && sscCore.V.N) sscCore.glDraw(gl);
        else mesh.glDraw(gl);
        break;

      default: HALT("can't draw that geom yet");
    }
  }
  if(orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -X.pos.z);
    glEnd();
  }

  if(orsDrawBodyNames && body) {
    glColor(1, 1, 1);
    glDrawText(body->name, 0, 0, 0);
  }

  glPopName();
}

/// GL routine to draw a rai::Configuration
void rai::Configuration::glDraw(OpenGL& gl) {
  uint i=0;
  rai::Transformation f;
  double GLmatrix[16];

  glPushMatrix();

  //bodies
  if(orsDrawBodies) for(Shape* s: shapes) {
      s->glDraw(gl);
      i++;
      if(orsDrawLimit && i>=orsDrawLimit) break;
    }

  //joints
  if(orsDrawJoints) for(Joint* e: joints) {
      //set name (for OpenGL selection)
      glPushName((e->index <<2) | 2);

      double s=e->A.pos.length()+e->B.pos.length(); //some scale
      s*=.25;

      //from body to joint
      f=e->from->X;
      f.getAffineMatrixGL(GLmatrix);
      glLoadMatrixd(GLmatrix);
      glColor(1, 1, 0);
      //glDrawSphere(.1*s);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
      glEnd();

      //joint frame A
      f.appendTransformation(e->A);
      f.getAffineMatrixGL(GLmatrix);
      glLoadMatrixd(GLmatrix);
      glDrawAxes(s);
      glColor(1, 0, 0);
      glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

      //joint frame B
      f.appendTransformation(e->Q);
      f.getAffineMatrixGL(GLmatrix);
      glLoadMatrixd(GLmatrix);
      glDrawAxes(s);

      //from joint to body
      glColor(1, 0, 1);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
      glEnd();
      glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
      //glDrawSphere(.1*s);

      glPopName();
      i++;
      if(orsDrawLimit && i>=orsDrawLimit) break;
    }

  //proxies
  if(orsDrawProxies) for(Proxy* proxy: proxies) {
      glLoadIdentity();
      if(!proxy->colorCode) {
        if(proxy->d>0.) glColor(.75, .75, .75);
        else glColor(.75, .5, .5);
      } else glColor(proxy->colorCode);
      glBegin(GL_LINES);
      glVertex3dv(proxy->posA.p());
      glVertex3dv(proxy->posB.p());
      glEnd();
      rai::Transformation f;
      f.pos=proxy->posA;
      f.rot.setDiff(rai::Vector(0, 0, 1), proxy->posA-proxy->posB);
      f.getAffineMatrixGL(GLmatrix);
      glLoadMatrixd(GLmatrix);
      glDisable(GL_CULL_FACE);
      glDrawDisk(.02);
      glEnable(GL_CULL_FACE);

      f.pos=proxy->posB;
      f.getAffineMatrixGL(GLmatrix);
      glLoadMatrixd(GLmatrix);
      glDrawDisk(.02);
    }

  glPopMatrix();
}

void displayState(const arr& x, rai::Configuration& G, const char* tag) {
  G.setJointState(x);
  G.watch(true, tag);
}

void displayTrajectory(const arr& _x, int steps, rai::Configuration& G, const KinematicSwitchL& switches, const char* tag, double delay, uint dim_z, bool copyG) {
  if(!steps) return;
  for(rai::Shape* s : G.shapes) if(s->mesh.V.d0!=s->mesh.Vn.d0 || s->mesh.T.d0!=s->mesh.Tn.d0) {
      s->mesh.computeNormals();
    }
  rai::Configuration* Gcopy;
  if(switches.N) copyG=true;
  if(!copyG) Gcopy=&G;
  else {
    Gcopy = new rai::Configuration;
    Gcopy->copy(G, true);
  }
  arr x, z;
  if(dim_z) {
    x.referToRange(_x, 0, -dim_z-1);
    z.referToRange(_x, -dim_z, -1);
  } else {
    x.referTo(_x);
  }
  uint n=Gcopy->getJointStateDimension()-dim_z;
  x.reshape(x.N/n, n);
  uint num, T=x.d0-1;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(uint k=0; k<=(uint)num; k++) {
    uint t = (T?(k*T/num):0);
    if(switches.N) {
      for(rai::KinematicSwitch* sw: switches)
        if(sw->timeOfApplication==t)
          sw->apply(*Gcopy);
    }
    if(dim_z) Gcopy->setJointState(cat(x[t], z));
    else Gcopy->setJointState(x[t]);
    if(delay<0.) {
      if(delay<-10.) FILE("z.graph") <<*Gcopy;
      Gcopy->watch(true, STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    } else {
      Gcopy->watch(false, STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) rai::wait(delay);
    }
  }
  if(steps==1)
    Gcopy->watch(true, STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
  if(copyG) delete Gcopy;
}

/* please don't remove yet: code for displaying edges might be useful...

void glDrawOdeWorld(void *classP){
  _glDrawOdeWorld((dWorldID)classP);
}

void _glDrawOdeWorld(dWorldID world)
{
  glStandardLight();
  glColor(3);
  glDrawFloor(4);
  uint i;
  Color c;
  dVector3 vec, vec2;
  dBodyID b;
  dGeomID g, gg;
  dJointID j;
  dReal a, al, ah, r, len;
  glPushName(0);
  int t;

  //bodies
  for(i=0, b=world->firstbody;b;b=(dxBody*)b->next){
    i++;
    glPushName(i);

    //if(b->userdata){ glDrawBody(b->userdata); }
    c.setIndex(i); glColor(c.r, c.g, c.b);
    glShadeModel(GL_FLAT);

    //bodies
    for(g=b->geom;g;g=dGeomGetBodyNext(g)){
      if(dGeomGetClass(g)==dGeomTransformClass){
  ((dxGeomTransform*)g)->computeFinalTx();
        glTransform(((dxGeomTransform*)g)->final_pos, ((dxGeomTransform*)g)->final_R);
  gg=dGeomTransformGetGeom(g);
      }else{
  glTransform(g->pos, g->R);
  gg=g;
      }
      b = dGeomGetBody(gg);
      // set the color of the body, 4. Mar 06 (hh)
      c.r = ((Body*)b->userdata)->cr;
      c.g = ((Body*)b->userdata)->cg;
      c.b = ((Body*)b->userdata)->cb;
      glColor(c.r, c.g, c.b);

      switch(dGeomGetClass(gg))
  {
  case dSphereClass:
    glDrawSphere(dGeomSphereGetRadius(gg));
    break;
  case dBoxClass:
    dGeomBoxGetLengths(gg, vec);
    glDrawBox(vec[0], vec[1], vec[2]);
    break;
  case dCCylinderClass: // 6. Mar 06 (hh)
    dGeomCCylinderGetParams(gg, &r, &len);
    glDrawCappedCylinder(r, len);
    break;
  default: HALT("can't draw that geom yet");
  }
      glPopMatrix();
    }

    // removed shadows,  4. Mar 06 (hh)

    // joints

      dxJointNode *n;
      for(n=b->firstjoint;n;n=n->next){
      j=n->joint;
      t=dJointGetType(j);
      if(t==dJointTypeHinge){
      dJointGetHingeAnchor(j, vec);
      a=dJointGetHingeAngle(j);
      al=dJointGetHingeParam(j, dParamLoStop);
      ah=dJointGetHingeParam(j, dParamHiStop);
      glPushMatrix();
      glTranslatef(vec[0], vec[1], vec[2]);
      dJointGetHingeAxis(j, vec);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glEnd();
      //glDrawText(STRING(al <<'<' <<a <<'<' <<ah), LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glPopMatrix();
      }
      if(t==dJointTypeAMotor){
  glPushMatrix();
  glTranslatef(b->pos[0], b->pos[1], b->pos[2]);
  dJointGetAMotorAxis(j, 0, vec);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
  glEnd();
  glPopMatrix();
      }
      if(t==dJointTypeBall){
  dJointGetBallAnchor(j, vec);
  dJointGetBallAnchor2(j, vec2);
  glPushMatrix();
  glTranslatef(vec[0], vec[1], vec[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
  glPushMatrix();
  glTranslatef(vec2[0], vec2[1], vec2[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
      }
    }
      glPopName();
  }
  glPopName();
}
*/

void animateConfiguration(rai::Configuration& C, Inotify* ino) {
  arr x, x0;
  uint t, i;
  C.getJointState(x0);
  arr lim = C.getLimits();
  C.gl().pressedkey=0;
  const int steps = 50;
  for(i=x0.N; i--;) {
    x=x0;
    const double upper_lim = lim(i, 1);
    const double lower_lim = lim(i, 0);
    const double delta = upper_lim - lower_lim;
    const double center = lower_lim + delta / 2.;
    const double offset = acos(2. * (x0(i) - center) / delta);

    for(t=0; t<steps; t++) {
      if(C.gl().pressedkey==13 || C.gl().pressedkey==27) return;
      if(ino && ino->pollForModification()) return;
      if(lim(i, 0)==lim(i, 1))
        break;

      x(i) = center + (delta*(0.5*cos(RAI_2PI*t/steps + offset)));
      // Joint limits
      C.setJointState(x);
      C.watch(false, STRING("DOF = " <<i), false, false, true);
      rai::wait(0.01);
    }
  }
  C.setJointState(x0);
  C.watch(false, "", false, false, true);
}

rai::Body* movingBody=nullptr;
rai::Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationClickCall:OpenGL::GLClickCall {
  rai::Configuration* ors;
  EditConfigurationClickCall(rai::Configuration& _ors) { ors=&_ors; }
  bool clickCallback(OpenGL& gl) {
    OpenGL::GLSelect* top=gl.topSelection;
    if(!top) return false;
    uint i=top->name;
    cout <<"CLICK call: id = 0x" <<std::hex <<gl.topSelection->name <<" : ";
    gl.text.clear();
    if((i&3)==1) {
      rai::Shape* s=ors->shapes(i>>2);
      gl.text <<"shape selection: shape=" <<s->name <<" body=" <<s->body->name <<" X=" <<s->X <<endl;
//      listWrite(s->ats, gl.text, "\n");
      cout <<gl.text;
    }
    if((i&3)==2) {
      rai::Joint* j=ors->joints(i>>2);
      gl.text
          <<"edge selection: " <<j->from->name <<' ' <<j->to->name
          <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
//      listWrite(j->ats, gl.text, "\n");
      cout <<gl.text;
    }
    cout <<endl;
    return true;
  }
};

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  rai::Configuration* ors;
  EditConfigurationHoverCall(rai::Configuration& _ors);// { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
//    if(!movingBody) return false;
    if(!movingBody) {
      rai::Joint* j=nullptr;
      rai::Shape* s=nullptr;
      rai::timerStart(true);
      gl.Select(true);
      OpenGL::GLSelect* top=gl.topSelection;
      if(!top) return false;
      uint i=top->name;
      cout <<rai::timerRead() <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->shapes(i>>2);
      if((i&3)==2) j=ors->joints(i>>2);
      gl.text.clear();
      if(s) {
        gl.text <<"shape selection: body=" <<s->body->name <<" X=" <<s->body->X <<" ats=" <<endl;
        listWrite(s->ats, gl.text, "\n");
      }
      if(j) {
        gl.text
            <<"edge selection: " <<j->from->name <<' ' <<j->to->name
            <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->X.pos = selpos + ARR(x-selx, y-sely, z-selz);
    }
    return true;
  }
};

EditConfigurationHoverCall::EditConfigurationHoverCall(rai::Configuration& _ors) {
  ors=&_ors;
}

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  rai::Configuration& ors;
  bool& exit;
  EditConfigurationKeyCall(rai::Configuration& _ors, bool& _exit): ors(_ors), exit(_exit) {}
  bool keyCallback(OpenGL& gl) {
    if(gl.pressedkey==' ') { //grab a body
      if(movingBody) { movingBody=nullptr; return true; }
      rai::Joint* j=nullptr;
      rai::Shape* s=nullptr;
      gl.Select();
      OpenGL::GLSelect* top=gl.topSelection;
      if(!top) { cout <<"No object below mouse!" <<endl;  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors.shapes(i>>2);
      if((i&3)==2) j=ors.joints(i>>2);
      if(s) {
        cout <<"selected shape " <<s->name <<" of body " <<s->body->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = s->body->X.pos;
        movingBody=s->body;
      }
      if(j) {
        cout <<"selected joint " <<j->index <<" connecting " <<j->from->name <<"--" <<j->to->name <<endl;
      }
      return true;
    } else switch(gl.pressedkey) {
        case '1':  orsDrawBodies^=1;  break;
        case '2':  orsDrawShapes^=1;  break;
        case '3':  orsDrawJoints^=1;  orsDrawMarkers^=1; break;
        case '4':  orsDrawProxies^=1;  break;
        case '5':  gl.reportSelects^=1;  break;
        case '6':  gl.reportEvents^=1;  break;
        case '7':  ors.writePlyFile("z.ply");  break;
        case 'j':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(0, 0, .1);  break;
        case 'k':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(0, 0, .1);  break;
        case 'i':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(0, .1, 0);  break;
        case ',':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(0, .1, 0);  break;
        case 'l':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(.1, .0, 0);  break;
        case 'h':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(.1, 0, 0);  break;
        case 'a':  gl.camera.focus(
            (gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
             ^ gl.camera.X.rot*rai::Vector(1, 0, 0)) * .001
            + gl.camera.foc);
          break;
        case 's':  gl.camera.X.pos +=
            (
              gl.camera.X.rot*(gl.camera.foc - gl.camera.X.pos)
              ^(gl.camera.X.rot * rai::Vector(1., 0, 0))
            ) * .01;
          break;
        case 'q' :
          cout <<"EXITING" <<endl;
          exit=true;
          break;
      }
    gl.postRedrawEvent(true);
    return true;
  }
};

void editConfiguration(const char* filename, rai::Configuration& C) {
//  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  bool exit=false;
//  C.gl().addHoverCall(new EditConfigurationHoverCall(C));
  C.gl().addKeyCall(new EditConfigurationKeyCall(C, exit));
  C.gl().addClickCall(new EditConfigurationClickCall(C));
  Inotify ino(filename);
  for(; !exit;) {
    cout <<"reloading `" <<filename <<"' ... " <<std::endl;
    rai::Configuration W;
    try {
      rai::lineCount=1;
      W <<FILE(filename);
      C.gl().lock.writeLock();
      C = W;
      C.gl().lock.unlock();
    } catch(const char* msg) {
      cout <<"line " <<rai::lineCount <<": " <<msg <<" -- please check the file and press ENTER" <<endl;
      C.watch(true,);
      continue;
    }
    C.watch(false,);
    cout <<"animating.." <<endl;
    //while(ino.pollForModification());
    animateConfiguration(C, &ino);
    cout <<"watching..." <<endl;
#if 0
    ino.waitForModification();
#else
    C.watch(true,);
#endif
    if(!rai::getInteractivity()) {
      exit=true;
    }
  }
}

#if 0 //RAI_ODE
void testSim(const char* filename, rai::Configuration* C, Ode* ode) {
  C.watch(true,);
  uint t, T=200;
  arr x, v;
  createOde(*C, *ode);
  ors->getJointState(x, v);
  for(t=0; t<T; t++) {
    ode->step();

    importStateFromOde(*C, *ode);
    ors->setJointState(x, v);
    ors->calcBodyFramesFromJoints();
    exportStateToOde(*C, *ode);

    C.gl().text.clear() <<"time " <<t;
    C.gl().timedupdate(10);
  }
}
#endif
#endif

#else ///RAI_GL
#ifndef RAI_ORS_ONLY_BASICS
void bindOrsToOpenGL(rai::Configuration&, OpenGL&) { NICO };
void rai::Configuration::glDraw(OpenGL&) { NICO }
void rai::glDrawGraph(void* classP) { NICO }
void editConfiguration(const char* orsfile, rai::Configuration& C) { NICO }
void animateConfiguration(rai::Configuration& C, Inotify*) { NICO }
void glTransform(const rai::Transformation&) { NICO }
void displayTrajectory(const arr&, int, rai::Configuration&, const char*, double) { NICO }
void displayState(const arr&, rai::Configuration&, const char*) { NICO }
#endif
#endif
