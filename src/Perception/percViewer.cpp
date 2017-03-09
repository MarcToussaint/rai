#include "percViewer.h"
#include <Gui/opengl.h>

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
  modelCopy.resize(modelWorld().shapes.N);
  for(mlr::Shape *s:modelWorld().shapes){
    mlr::Mesh& m=modelCopy(s->index);
    m = s->mesh;
    m.glX = s->X;

  }
  modelWorld.deAccess();
}

void PercViewer::close(){
  delete gl;
}

void PercViewer::step(){
  mlr::Array<mlr::Transformation> X;
  modelWorld.readAccess();
  X.resize(modelWorld().shapes.N);
  for(mlr::Shape *s:modelWorld().shapes) X(s->index) = s->X;
  modelWorld.deAccess();

  gl->dataLock.writeLock();
  listClone(copy, percepts.get()());
  if(X.N==modelCopy.N) for(uint i=0;i<X.N;i++) modelCopy(i).glX = X(i);
  gl->dataLock.unlock();

  gl->update(NULL, false, false, true);
}

