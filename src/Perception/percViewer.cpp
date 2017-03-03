#include "percViewer.h"
#include <Gui/opengl.h>

PercViewer::PercViewer(const char* percepts_name)
  : Thread(STRING("PercViewer_"<<percepts_name), -1.),
    percepts(this, percepts_name, true) {
  threadOpen();
}

PercViewer::~PercViewer(){
  threadClose();
}

void glDrawPercepts(void *P){
  PerceptL& percepts = *((PerceptL*)P);
  for(Percept *p:percepts) p->glDraw(NoOpenGL);
}

void PercViewer::open(){
  gl = new OpenGL(STRING("PercViewer: "<<percepts.name));
  gl->add(glStandardScene);
  gl->add(glDrawPercepts, &copy);
  gl->camera.setDefault();
}

void PercViewer::close(){
  delete gl;
}

void PercViewer::step(){
  gl->dataLock.writeLock();
  listClone(copy, percepts.get()());
  gl->dataLock.unlock();
  gl->update();
}

