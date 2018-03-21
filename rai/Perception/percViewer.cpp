/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "percViewer.h"
#include <Gui/opengl.h>
#include <Kin/frame.h>

PercViewer::PercViewer(const char* percepts_name)
  : Thread(STRING("PercViewer_"<<percepts_name), -1.),
    percepts(this, percepts_name, true),
    modelWorld(this, "modelWorld"){
  threadOpen();
}

PercViewer::~PercViewer(){
  threadClose();
}

void glDrawPercepts(void *P){
  PerceptL& percepts = *((PerceptL*)P);
  for(Percept *p:percepts){
    glPushMatrix();
    glTransform(p->transform);
    glColor3f(0,0,0);
    glDrawText(STRING(p->id),0,0,0, true);
    p->glDraw(NoOpenGL);
    glPopMatrix();
  }
}

void PercViewer::open(){
  gl = new OpenGL(STRING("PercViewer: "<<percepts.name));
  gl->add(glStandardScene);
  gl->add(glDrawMeshes, &modelCopy);
  gl->add(glDrawPercepts, &copy);
  gl->camera.setDefault();
  gl->camera.setPosition(2., -3., 3.);
  gl->camera.focus(.5, 0, .6);
  gl->camera.upright();

  modelWorld.writeAccess();
  modelCopy.resize(modelWorld().frames.N);
  for(rai::Frame *f: modelWorld().frames){
    rai::Shape *s = f->shape;
    if(s){
      rai::Mesh& m=modelCopy(f->ID);
      m = s->mesh();
      if(!m.C.N) m.C = {.6, .6, .6, .3};
      if(m.C.N==3) m.C.append(.3);
      if(m.C.N==4) m.C(3)=.3;
      m.glX = f->X;
    }
  }
  modelWorld.deAccess();
}

void PercViewer::close(){
  delete gl;
}

void PercViewer::step(){
  percepts.readAccess();
  if(!percepts().N){ percepts.deAccess(); return; }
  gl->dataLock.writeLock();
  listClone(copy, percepts.get()());
  gl->dataLock.unlock();
  percepts.deAccess();

  rai::Array<rai::Transformation> X;
  modelWorld.readAccess();
  X.resize(modelWorld().frames.N);
  for(rai::Frame *f:modelWorld().frames) X(f->ID) = f->X;
  modelWorld.deAccess();

  gl->dataLock.writeLock();
  if(X.N==modelCopy.N) for(uint i=0;i<X.N;i++) modelCopy(i).glX = X(i);
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

