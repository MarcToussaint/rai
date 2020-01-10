#include "viewer.h"
#include "frame.h"
#include <iomanip>
#include <Gui/opengl.h>

void rai::ConfigurationViewer::ensure_gl() {
  if(!gl) {
    gl = make_shared<OpenGL>("ConfigurationViewer");
    gl->camera.setDefault();
    gl->add(*this);
  }
}

int rai::ConfigurationViewer::update(bool watch) {
  ensure_gl();

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    gl->text = drawText;
    if(watch) gl->text <<"\n[ENTER]";
  }

  if(watch) {
    gl->watch();
    gl->text = drawText();
  }
  gl->update(nullptr, true);
  return gl->pressedkey;
}

void rai::ConfigurationViewer::setConfiguration(rai::Configuration& C, const char* text, bool watch){
  if(C.frames.N!=meshes.N) recopyMeshes(C);

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    framePath.resize(C.frames.N, 7);
    for(uint i=0;i<C.frames.N;i++) framePath[i] = C.frames(i)->getPose();
    framePath.reshape(1, C.frames.N, 7);
    drawTimeSlice=0;
    if(text) drawText = text;
  }

  update(watch);
}

void rai::ConfigurationViewer::setPath(ConfigurationL& Cs, const char* text, bool watch) {
  uintA frames;
  frames.setStraightPerm(Cs.first()->frames.N);

  arr X(Cs.N, frames.N, 7);
  for(uint t=0; t<X.d0; t++) {
    for(uint i=0; i<X.d1; i++) {
      X(t, i, {}) = Cs(t)->frames(frames(i))->getPose();
    }
  }

  setPath(X, text, watch);
}

void rai::ConfigurationViewer::setPath(rai::Configuration& C, const arr& jointPath, const char* text, bool watch, bool full){
  arr X(jointPath.d0, C.frames.N, 7);
  for(uint t=0; t<X.d0; t++) {
    C.setJointState(jointPath[t]);
    for(uint i=0; i<X.d1; i++) {
      X(t, i, {}) = C.frames.elem(i)->getPose();
    }
  }

  setPath(X, text, watch);
}

void rai::ConfigurationViewer::setPath(const arr& _framePath, const char* text, bool watch, bool full) {
  CHECK_EQ(_framePath.nd, 3, "");
  CHECK_EQ(_framePath.d2, 7, "");

  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    framePath = _framePath;
    drawFullPath=full;
    drawTimeSlice=-1;
    if(text) drawText = text;
  }

  update(watch);
}

bool rai::ConfigurationViewer::playVideo(bool watch, double delay, const char* saveVideoPath) {
  const rai::String tag = drawText;

  if(saveVideoPath) {
    rai::system(STRING("mkdir -p " <<saveVideoPath));
    rai::system(STRING("rm -f " <<saveVideoPath <<"*.ppm"));
  }

  for(uint t=0;t<framePath.d0;t++) {
    {
      auto _dataLock = gl->dataLock(RAI_HERE);
      drawTimeSlice=t;
      drawText.clear() <<tag <<" (config:" <<t <<'/' <<framePath.d0 <<")";
    }

    if(delay<0.) {
      update(true);
    } else {
      update(false);
      if(delay) rai::wait(delay / framePath.d0);
    }

    {
      auto _dataLock = gl->dataLock(RAI_HERE);
      if(saveVideoPath) write_ppm(gl->captureImage, STRING(saveVideoPath<<std::setw(4)<<std::setfill('0')<<t<<".ppm"));
    }
  }
  drawText = tag;
  if(watch) {
    int key = update(true);
    return !(key==27 || key=='q');
  }
  return false;
}

rai::Camera& rai::ConfigurationViewer::displayCamera() {
  ensure_gl();
  return gl->camera;
}

void rai::ConfigurationViewer::recopyMeshes(rai::Configuration& C){
  ensure_gl();

  uint n=C.frames.N;
  {
    auto _dataLock = gl->dataLock(RAI_HERE);
    meshes.resize(n);
    for(uint i=0; i<n; i++) {
      if(C.frames.elem(i)->shape) meshes.elem(i) = C.frames.elem(i)->shape->mesh();
      else meshes.elem(i).clear();
    }
  }
}

void rai::ConfigurationViewer::glDraw(OpenGL& gl) {
  glStandardScene(NULL, gl);

  glPushMatrix();

  rai::Transformation T;
  double GLmatrix[16];


//  //proxies
//  if(orsDrawProxies) for(const Proxy& p: proxies) {
//      ((Proxy*)&p)->glDraw(gl);
//    }

//  //contacts
////  if(orsDrawProxies)
//  for(const Frame* fr: frames) for(rai::Contact* c:fr->contacts) if(&c->a==fr) {
//        c->glDraw(gl);
//      }

  if(drawTimeSlice>=0){
    uint t=drawTimeSlice;
    CHECK_LE(t+1, framePath.d0, "");
    CHECK_EQ(framePath.d1, meshes.N, "");
    CHECK_EQ(framePath.d2, 7, "");

    //first non-transparent
    for(uint i=0;i<framePath.d1;i++){
      if(meshes.elem(i).C.N!=4 || meshes.elem(i).C.elem(3)==1.){
        T.set(&framePath(t, i, 0));
        glTransform(T);
        meshes.elem(i).glDraw(gl);
      }
    }
    for(uint i=0;i<framePath.d1;i++){
      if(meshes.elem(i).C.N==4 && meshes.elem(i).C.elem(3)<1.){
        T.set(&framePath(t, i, 0));
        glTransform(T);
        meshes.elem(i).glDraw(gl);
      }
    }

    //draw frame paths
    glColor(0., 0., 0.);
    glLoadIdentity();
    for(uint i=0; i<framePath.d1; i++) {
      glBegin(GL_LINES);
      for(uint t=0; t<framePath.d0; t++) {
        T.set(&framePath(t, i, 0));
//          glTransform(pose);
        glVertex3d(T.pos.x, T.pos.y, T.pos.z);
      }
      glEnd();
    }

  }else{
    if(drawFullPath){
      CHECK_EQ(framePath.d1, meshes.N, "");
      CHECK_EQ(framePath.d2, 7, "");
      for(uint i=0;i<framePath.d1;i++){
        if(meshes.elem(i).C.N!=4 || meshes.elem(i).C.elem(3)==1.){
          for(uint t=0;t<framePath.d0;t++){
            T.set(&framePath(t,i,0));
            glTransform(T);
            meshes.elem(i).glDraw(gl);
          }
        }
      }
      for(uint i=0;i<framePath.d1;i++){
        if(meshes.elem(i).C.N==4 && meshes.elem(i).C.elem(3)<1.){
          for(uint t=0;t<framePath.d0;t++){
            T.set(&framePath(t,i,0));
            glTransform(T);
            meshes.elem(i).glDraw(gl);
          }
        }
      }
    }else{
      NIY;
    }
//    gl->addDrawer(configurations.last());
//    gl->add(drawX);
  }

//  if(overlayPaths) gl->add(drawX);
  glPopMatrix();

}

