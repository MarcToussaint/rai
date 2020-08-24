/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "perceptViewer.h"
#include "../Gui/opengl.h"
#include "../Kin/frame.h"

PerceptViewer::PerceptViewer(Var<PerceptL>& _percepts, Var<rai::Configuration> _kin)
  : Thread(STRING("PercViewer_"<<_percepts.name()), -1.),
    percepts(this, _percepts, true),
    kin(this, _kin, false) {
  threadOpen();
}

PerceptViewer::~PerceptViewer() {
  threadClose();
}

void glDrawPercepts(void* P, OpenGL&) {
  PerceptL& percepts = *((PerceptL*)P);
  for(std::shared_ptr<Percept>& p:percepts) {
    glTransform(p->pose);
    glPushMatrix();
    p->glDraw(NoOpenGL);
    glPopMatrix();
    glTranslated(p->com.x, p->com.y, p->com.z);
    glColor3f(0, 0, 0);
    glDrawText(STRING(p->id), 0, 0, 0, true);
    glColor3f(0, 1, 0);
  }
}

void PerceptViewer::open() {
  gl = new OpenGL(STRING("PercViewer "<<percepts.name()));
  gl->add(glStandardScene);
//  gl->add(glDrawMeshes, &modelCopy);
  gl->add(glDrawPercepts, &copy);

//  modelWorld.writeAccess();
//  modelCopy.resize(modelWorld().frames.N);
//  for(rai::Frame *f: modelWorld().frames) {
//    rai::Shape *s = f->shape;
//    if(s) {
//      rai::Mesh& m=modelCopy(f->ID);
//      m = s->mesh();
//      if(!m.C.N) m.C = {.6, .6, .6, .3};
//      if(m.C.N==3) m.C.append(.3);
//      if(m.C.N==4) m.C(3)=.3;
//      m.glX = f->X;
//    }
//  }
//  modelWorld.deAccess();
}

void PerceptViewer::close() {
  delete gl;
}

void PerceptViewer::step() {
  percepts.readAccess();
  if(!percepts().N) { percepts.deAccess(); return; }
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    copy = percepts.get(); //this copies shared pointers!
    //  listClone(copy, percepts.get()());
  }
  percepts.deAccess();

//  rai::Array<rai::Transformation> X;
//  modelWorld.readAccess();
//  X.resize(modelWorld().frames.N);
//  for(rai::Frame *f:modelWorld().frames) X(f->ID) = f->X;
//  modelWorld.deAccess();

//  gl->dataLock.writeLock();
//  if(X.N==modelCopy.N) for(uint i=0; i<X.N; i++) modelCopy(i).glX = X(i);
//  gl->dataLock.unlock();

  gl->update(nullptr, false); //nullptr, false, false, true);
}

