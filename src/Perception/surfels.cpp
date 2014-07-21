#include "surfels.h"
#include <Core/geo.h>
#include <Perception/kinect2pointCloud.h>

void glDrawSurfels(void *classP){ ((Surfels*)classP)->glDraw(); }

void Surfels::setRandom(uint N){
  pos.resize(N,3);
  rndGauss(pos, .3);
  for(uint i=0;i<N;i++) pos(i,2) += 1.f;
  norm.resize(N,3);
  for(uint i=0;i<N;i++) norm[i]() = {0,0,-1};
//  rndGauss(norm);
  for(uint i=0;i<norm.d0;i++) norm[i]() /= length(norm[i]);
  col.resize(N,3);
  rndUniform(col);
  rad.resize(N);
  rad=.01;
}

void Surfels::glDraw(){
  mx.lock();
  if(renderIndex){
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }
  glDisable(GL_CULL_FACE);
  float h=.5*sqrt(3);
  double tmp[12];
  ors::Transformation T;
  glNewList(1, GL_COMPILE);
  glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.,0.,0.);
    glVertex3f(1.,0.,0.);
    glVertex3f( .5,h,0.);
    glVertex3f(-.5,h,0.);
    glVertex3f(-1.,0.,0.);
    glVertex3f(-.5,-h,0.);
    glVertex3f( .5,-h,0.);
    glVertex3f(1.,0.,0.);
  glEnd();
  glEndList();

  for(uint i=0;i<pos.d0;i++){
//    if(!renderIndex && D(i).n<1.) continue; //don't draw
    if(!renderIndex){
      glColor3fv(&col(i,0));
      glNormal3fv(&norm(i, 0));
    }else{
      uint j=i+1;
      glColor3f(float((j>>16)&0xff)/256.f, float((j>>8)&0xff)/256.f, float(j&0xff)/256.f);
    }
    T.pos.set(pos(i,0), pos(i,1), pos(i,2));
    T.rot.setDiff(Vector_z, ors::Vector(norm[i]));
    glLoadMatrixd(T.getAffineMatrixGL(tmp));
    glScaled(rad(i),rad(i),rad(i));
    glCallList(1);
  }
  glLoadIdentity();
  glEnable(GL_CULL_FACE);
  mx.unlock();
}

void Surfels::recomputeSurfelIndices(OpenGL& gl){
  mx.lock();
  renderIndex=true;
//  ors::Camera saveCam = gl.camera;
  gl.camera = kinectCam;
//  gl.update(NULL, true);
  gl.renderInBack();
  flip_image(gl.captureImage);
  const byteA& img = gl.captureImage;
  surfelIdx.resize(img.d0,img.d1);
  mask.resize(img.d0,img.d1);
  for(uint i=0;i<surfelIdx.N;i++){
    surfelIdx.elem(i) = img.elem(3*i+0)<<16 | img.elem(3*i+1)<<8 | img.elem(3*i+2);
    mask.elem(i) = (surfelIdx.elem(i)==0?0:255);
  }
//  gl.camera = saveCam;
  renderIndex=false;
  mx.unlock();
}

void Surfels::pointCloud2Surfels(const arr& pts, const arr& cols, OpenGL& gl){
  recomputeSurfelIndices(gl);
  CHECK(pts.d0==surfelIdx.N,"mismatch in #pixels");
  mx.lock();
  if(rndPerm.N!=surfelIdx.N) rndPerm.setRandomPerm(surfelIdx.N);
  for(uint i=0;i<1;i++){ //pts.d0;i++){
//    uint p = rndPerm(i);
    uint p = rnd(pts.d0);
    uint s = surfelIdx.elem(p);
    if(pts(p,2)<0. || sum(cols[p])>2.9) continue; //not a legible point!
    if(s==0){ //no surfel hit
      CHECK(mask.elem(p)==0,"");
      pos.append(ARRAY<float>(pts(p,0), pts(p,1), pts(p,2)));
      col.append(ARRAY<float>(cols(p,0),cols(p,1),cols(p,2)));
      norm.append(ARRAY<float>(0,0,-1));
      rad.append(0.03f);
      pos.reshape(pos.N/3,3);
      col.reshape(pos.N/3,3);
      norm.reshape(pos.N/3,3);
      D.resizeCopy(pos.N/3);
      i=pts.d0;
      cout <<"ADDED surfel: p=" <<p <<" pos=" <<pts[p] <<" col=" <<cols[p] <<endl;
    }else{
      s -= 1;
      CHECK(mask.elem(p)==255,"");
//      if(sqrt(sumOfSqr(ARRAY<float>(pts(p,0), pts(p,1), pts(p,2)) - pos[2]))<.1){
//        D(s).add(pts(p,0), pts(p,1), pts(p,2),
//                 cols(p,0),cols(p,1),cols(p,2));
//      }
//      if(D(s).n>5.){
//        float tmp;
//        D(s).mean(pos(s,0), pos(s,1), pos(s,2));
//        D(s).meanRGB(col(s,0), col(s,1), col(s,2));
////        D(s).norm(norm(s,0), norm(s,1), norm(s,2));
//      }
//      D(s).discount(.95);
    }
//    gl.update();
  }
  mx.unlock();
}
