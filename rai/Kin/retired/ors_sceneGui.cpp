/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "kin_ode.h"
#include "kin_sceneGui.h"

enum EditMode { emNone, emMove, emOde };

struct sOrsSceneGui:OpenGL::GLKeyCall, OpenGL::GLHoverCall, OpenGL::GLClickCall {
  OpenGL* gl;
  rai::Configuration* ors;
  EditMode mode;
  rai::Body* movingBody;
  rai::Vector selpos;
  double seld, selx, sely, selz;
  sOrsSceneGui() {
    mode = emNone;
  }

  static void drawBase(void*) {
    glStandardLight(nullptr);
    glDrawFloor(10, .8, .8, .8);
    glColor(1., .5, 0.);
  }

  bool keyCallback(OpenGL&);
  bool hoverCallback(OpenGL&);
  bool clickCallback(OpenGL&);

};

bool sOrsSceneGui::clickCallback(OpenGL&) {
  if(mode==emMove) {
    rai::Vector delta;
    if(gl->mouse_button==4 && gl->mouseIsDown) delta = -.01*(gl->camera.X.pos - movingBody->X.pos);
    if(gl->mouse_button==5 && gl->mouseIsDown) delta = +.01*(gl->camera.X.pos - movingBody->X.pos);
    if(delta.length()) {
      selpos += delta;
      movingBody->X.pos += delta;
      return false;
    }
  }
  return true;
}

bool sOrsSceneGui::hoverCallback(OpenGL&) {
  switch(mode) {
    case emNone: {
      rai::Joint* j=nullptr;
      rai::Shape* s=nullptr;
      gl->Select(true);
      OpenGL::GLSelect* top=gl->topSelection;
      if(!top) { gl->text.clear();  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl->topSelection->name <<endl;
      if((i&3)==1) s=ors->shapes(i>>2);
      if((i&3)==2) j=ors->joints(i>>2);
      if(s) {
        gl->text.clear()
            <<"shape selection: body=" <<s->body->name <<" X=" <<s->body->X <<" ats=" <<endl;
        listWrite(s->ats, gl->text, "\n");
      }
      if(j) {
        gl->text.clear()
            <<"edge selection: " <<j->from->name <<' ' <<j->to->name
            <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
        listWrite(j->ats, gl->text, "\n");
      }
      if(!j && !s) gl->text.clear();
      break;
    }
    case emMove: {
      //gl->Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl->mouseposx/gl->width(), y=(double)gl->mouseposy/gl->height(), z=seld;
      double x=gl->mouseposx, y=gl->mouseposy, z=seld;
      gl->unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->X.pos = selpos + ARR(x-selx, y-sely, z-selz);
      break;
    }
    case emOde: {
      cout <<"ODE step" <<endl;
      ors->ode().exportStateToOde();
      ors->ode().step(.01);
      ors->ode().importStateFromOde();
      break;
    }
  }
  return true;
}

bool sOrsSceneGui::keyCallback(OpenGL&) {
  switch(gl->pressedkey) {
    case ' ': { //move x-y the object
      if(mode==emMove) { mode=emNone; movingBody=nullptr; cout <<"move off" <<endl; return true; }
      cout <<"move mode" <<endl;
      mode=emMove;
      gl->Select();
      OpenGL::GLSelect* top=gl->topSelection;
      if(!top) {
        cout <<"No object below mouse!" <<endl;
        return nullptr;
      }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl->topSelection->name <<endl;
      rai::Body* b=nullptr;
      if((i&3)==1) b=ors->shapes(i>>2)->body;
      if(b) {
        cout <<"selected body " <<b->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = b->X.pos;
        movingBody=b;
      }
      return true;
    }
    case 'y': {
      //depth moving
      break;
    }
    case 'z': {
      //rotation
      break;
    }
    case 'v': {
      NIY;
//      if(mode==emOde) { mode=emNone; ors->ode.clear();  cout <<"ODE off" <<endl;  return true; }
//      cout <<"ODE mode" <<endl;
//      mode=emOde;
//      ode.createOde(*ors);
      return true;
    }
  }
  return true;
}

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  rai::Configuration* ors;
  EditConfigurationHoverCall(rai::Configuration& _ors);
  bool hoverCallback(OpenGL& gl);
};

OrsSceneGui::OrsSceneGui(rai::Configuration& ors, OpenGL* gl) {
  s=new sOrsSceneGui();
  s->ors = &ors;
  orsDrawZlines=true;
  if(!gl) {
    gl = new OpenGL();
    gl->add(sOrsSceneGui::drawBase, 0);
    gl->add(rai::glDrawGraph, &ors);
  }
  s->gl = gl;
//  gl->addHoverCall(s);
  gl->addHoverCall(new EditConfigurationHoverCall(ors));
//  gl->addKeyCall(s);
//  gl->addClickCall(s);

}

void OrsSceneGui::edit() {
  s->gl->watch();
}

/*
    while(rai::contains(gl.exitkeys, gl.pressedkey)){
      switch(gl.pressedkey){
        case '1':  orsDrawBodies^=1;  break;
        case '2':  orsDrawShapes^=1;  break;
        case '3':  orsDrawJoints^=1;  break;
        case '4':  orsDrawProxies^=1;  break;
        case '5':  gl.reportSelects^=1;  break;
        case '6':  gl.reportEvents^=1;  break;
        case 'j':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(0, 0, .1);  break;
        case 'k':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(0, 0, .1);  break;
        case 'i':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(0, .1, 0);  break;
        case ',':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(0, .1, 0);  break;
        case 'l':  gl.camera.X.pos += gl.camera.X.rot*rai::Vector(.1, .0, 0);  break;
        case 'h':  gl.camera.X.pos -= gl.camera.X.rot*rai::Vector(.1, 0, 0);  break;
        case 'a':  gl.camera.focus(
            (gl.camera.X.rot*(*gl.camera.foc - gl.camera.X.pos)
             ^ gl.camera.X.rot*rai::Vector(1, 0, 0)) * .001
            + *gl.camera.foc);
          break;
        case 's':  gl.camera.X.pos +=
            (
              gl.camera.X.rot*(*gl.camera.foc - gl.camera.X.pos)
              ^(gl.camera.X.rot * rai::Vector(1., 0, 0))
            ) * .01;
          break;
      }
      gl.watch();
    }
*/
